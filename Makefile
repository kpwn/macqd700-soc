# m68k-ooo Makefile
# Targets: sim build, verilator lint, vivado synth/impl, coverage tools
#
# Usage:
#   make sim          — build Verilator simulation binary
#   make test         — run all directed tests
#   make test TEST=x  — run a specific test
#   make lint         — lint all RTL
#   make synth        — Vivado synthesis (timing estimate)
#   make impl         — Vivado full impl (place + route)
#   make timing       — print WNS summary from latest impl
#   make coverage     — ISA opcode coverage report
#   make clean        — remove build artefacts

# ──────────────────────────────────────────────────────────────────────────────
# Paths
# ──────────────────────────────────────────────────────────────────────────────
PROJ_ROOT  := $(shell pwd)
RTL_DIR    := $(PROJ_ROOT)/rtl
TB_DIR     := $(PROJ_ROOT)/tb
SYNTH_DIR  := $(PROJ_ROOT)/synth
TOOLS_DIR  := $(PROJ_ROOT)/tools
BUILD_DIR  := $(PROJ_ROOT)/build
FPGA_VIDEO_TOOL := $(TOOLS_DIR)/fpga_video_capture.sh

VIVADO     ?= vivado
VERILATOR  ?= verilator
TCLSH      ?= tclsh
PCIE_TEST_DIR ?= /offlinenas/share/FPGA/pcie_test
DDR4_MIG_DIR ?= $(BUILD_DIR)/ddr4_mig
DDR4_MIG_DCP ?= $(DDR4_MIG_DIR)/design_1_ddr4_0_1.dcp
DDR4_MIG_GEN_TCL := $(SYNTH_DIR)/gen_ddr4_mig.tcl
DDR4_MIG_CACHE_TOOL := $(TOOLS_DIR)/ddr4_mig_cache.py
PCIE_XDMA_DIR ?= $(BUILD_DIR)/pcie_xdma
PCIE_XDMA_XCI ?= $(PCIE_XDMA_DIR)/design_1_xdma_0_0.xci
PCIE_XDMA_DCP ?= $(PCIE_XDMA_DIR)/design_1_xdma_0_0.dcp
# Keep generated-model builds serialized by default: the project has
# historically hit a Verilator PCH race with parallel make.  Override
# locally when iterating on a trusted target, e.g. VERILATOR_JOBS=8.
VERILATOR_JOBS    ?= 1
VERILATOR_THREADS ?= 4

# Host-specific m68k toolchain default: Debian/Ubuntu ships m68k-linux-gnu-*
# (apt install binutils-m68k-linux-gnu); macOS via homebrew ships m68k-elf-*.
# Any invocation can still override with `make M68K_AS=... M68K_LD=...`.
ifeq ($(shell uname),Linux)
  M68K_AS      ?= m68k-linux-gnu-as
  M68K_LD      ?= m68k-linux-gnu-ld
  M68K_OBJCOPY ?= m68k-linux-gnu-objcopy
else
  M68K_AS      ?= m68k-elf-as
  M68K_LD      ?= m68k-elf-ld
  M68K_OBJCOPY ?= m68k-elf-objcopy
endif

# ──────────────────────────────────────────────────────────────────────────────
# CPU RTL note (split-repo build truth)
# ──────────────────────────────────────────────────────────────────────────────
# This is the SoC-only split repo: the m68k pipeline (rtl/core/* in the old
# monorepo layout) now lives in the `cpu/` git submodule (cpu/rtl/core/*).
# There is no local rtl/core/* tree here any more, so a whole-design
# `mac_top` Verilator top (CPU + peripherals verilated together from a flat
# RTL_SRCS list) is not buildable from this repo alone.  `make sim` below is
# retired for exactly that reason — see the CPU build select section
# ("CPU=stub | CPU=m68k") further down for the real full-stack build
# (`lint-fpga-top`, `tb-fpga-top-rom`), which pulls cpu/rtl/core/* in via
# CPU_M68K_SRCS when CPU=m68k.
# ──────────────────────────────────────────────────────────────────────────────
# Retired core-track unit tbs — single stub mechanism
# ──────────────────────────────────────────────────────────────────────────────
# Every one of these verilated an rtl/core/* file (or an rtl/core/*.vh
# helper) that moved into the cpu/ submodule during the SoC split; they are
# 1:1-duplicated (same target name, same behaviour) in cpu/Makefile.  Rather
# than hand-writing N near-identical ".PHONY: X / X: ; @echo ... ; @exit 2"
# stanzas (or, worse, leaving them with no rule at all — invoking one used
# to print Make's own generic "No rule to make target 'X'.  Stop.", exactly
# the silent-staleness this cleanup exists to kill), every name below gets
# an identical loud stub generated from CORE_TRACK_RETIRED_STUB once.  Add
# new retired core-track target names to this list — do not hand-roll
# another one-off stanza.
CORE_TRACK_RETIRED_TBS := \
	tb-debug ras_sim ras_test \
	tb-lsu tb-mac-top-smc tb-icache tb-if-stage tb-reset-vectors \
	tb-dcache tb-dcache-burst \
	tb-mmu tb-mmu-walker tb-mmu-walker-boot \
	tb-bpu tb-alu tb-fpu tb-fp-rat \
	tb-rat tb-rob tb-iq-int tb-iq-fp tb-iq-mem tb-commit tb-exception \
	tb-predecode decode-probe decode-ea-helper-check tb-decode-fpu \
	tb-decode-shadow tb-exception-uop-gen tb-mul-div

define CORE_TRACK_RETIRED_STUB
.PHONY: $(1)
$(1):
	@echo "$(1) was removed from this repo's Makefile: it verilates rtl/core/* (or an rtl/core/*.vh helper), which moved to the cpu/ git submodule in the SoC split.  Run: make -C cpu $(1)" >&2
	@exit 2
endef

$(foreach t,$(CORE_TRACK_RETIRED_TBS),$(eval $(call CORE_TRACK_RETIRED_STUB,$(t))))

# ──────────────────────────────────────────────────────────────────────────────
# Simulation targets
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: sim
sim:
	@echo "make sim was removed: rtl/core/* moved to the cpu/ submodule, so the old flat mac_top Verilator top no longer builds from this repo. Use 'make tb-fpga-top-rom ROM=<path>' (full CPU+SoC sim, CPU=m68k) or 'make lint-fpga-top' (lint); for CPU-only unit tbs, cd cpu/ && make sim." >&2
	@exit 2

# `make test` is the repo-root gate.  The old body of this target ran the
# tb/tests/asm/*.s directed suite against the retired flat `mac_top` `sim`
# build (rtl/core/* — see the CPU RTL note above); that suite now lives in
# the cpu/ submodule (`cd cpu && make test`).  Here, `test` repoints to
# `tb-all`: the full platform/peripheral unit-tb suite (fabric, video, SD,
# async-fifo, peripherals — see ALL_TBS below), which already builds +
# prints a per-target PASS/FAIL/XFAIL summary and exits nonzero on any
# regression.
.PHONY: test
test: tb-all

# Compile all .s files in tb/tests/asm/ to binaries (best-effort, skips errors)
.PHONY: compile-tests
compile-tests:
	@mkdir -p $(BUILD_DIR)/tests
	@ok=0; fail=0; \
	for s in $(TB_DIR)/tests/asm/*.s; do \
		name=$$(basename $$s .s); \
		if $(M68K_AS) -m68040 -o $(BUILD_DIR)/tests/$$name.o $$s 2>/dev/null && \
		   $(M68K_LD) -Ttext 0x40800000 -o $(BUILD_DIR)/tests/$$name.elf $(BUILD_DIR)/tests/$$name.o 2>/dev/null && \
		   $(M68K_OBJCOPY) -O binary $(BUILD_DIR)/tests/$$name.elf $(BUILD_DIR)/tests/$$name.bin 2>/dev/null; then \
			ok=$$((ok+1)); \
		else \
			fail=$$((fail+1)); \
			echo "  [SKIP] $$name (assemble/link error)"; \
		fi; \
	done; \
	echo "compile-tests: $$ok assembled, $$fail skipped"

.PHONY: isa-width-audit
isa-width-audit:
	@python3 tools/isa_coverage.py --asm-width-audit tb/tests/asm

.PHONY: asm-test
asm-test:
	@mkdir -p $(BUILD_DIR)/tests
	$(M68K_AS) -m68040 -o $(BUILD_DIR)/tests/$(NAME).o $(ASM)
	$(M68K_LD) -Ttext 0x40800000 -o $(BUILD_DIR)/tests/$(NAME).elf $(BUILD_DIR)/tests/$(NAME).o
	$(M68K_OBJCOPY) -O binary $(BUILD_DIR)/tests/$(NAME).elf $(BUILD_DIR)/tests/$(NAME).bin

# ──────────────────────────────────────────────────────────────────────────────
# SingleStepTests/m68000 corpus runner
#
# The `make sst` target runs SingleStepTests (https://github.com/SingleStepTests/m68000)
# against the OoO core.  Per-test architectural state is poked into PRF +
# u_commit shadows, the test instruction is executed for exactly one
# macro retire, then final state is diffed against the SST expected.
#
# The corpus targets m68000 microcode; our core is m68040 OoO, so many
# tests are expected to diverge for legitimate ISA-revision reasons
# (CHK flags, ABCD/SBCD V/C handling, address-error exceptions, etc.).
# The runner only reports differences — user inspects and decides.
#
# Usage:
#   make sst                       — run every test in every .sstpack
#   make sst SST_FILE=ADD.b        — run a single op's tests
#   make sst SST_COUNT=10          — run first 10 tests of each pack
#   make sst SST_VERBOSE=1         — print register diffs on failures
#   make sst SST_CHECK_PC=1        — include PC in the diff
#
# The .sstpack files are generated from upstream JSON via
# `tools/sst/sst_pack.py`, which in turn depends on the upstream
# `decode.py` having been run on `third_party/singlesteptests/v1/`.
# ──────────────────────────────────────────────────────────────────────────────
SST_DIR        ?= $(PROJ_ROOT)/third_party/singlesteptests
SST_JSON_DIR   := $(SST_DIR)/v1
SST_PACK_DIR   := $(BUILD_DIR)/sst/packs
SST_BIN        := $(BUILD_DIR)/sst/Vmac_top
SST_FILE       ?=
SST_START      ?= 0
SST_COUNT      ?=
SST_TIMEOUT    ?= 5000
SST_WARMUP     ?= 32
SST_VERBOSE    ?=
SST_CHECK_PC   ?=
SST_CHECK_SR   ?=

# SST harness binary was the flat `mac_top` build (rtl/core/*, now in the
# cpu/ submodule) — same as the retired `sim` target.  Fail loud rather
# than silently miscompiling with an empty RTL list.
$(SST_BIN):
	@echo "make sst was removed: rtl/core/* moved to the cpu/ submodule, so the flat mac_top SST harness no longer builds from this repo. cd cpu/ && make sst instead." >&2
	@exit 2

.PHONY: sst-build
sst-build: $(SST_BIN)

# Clone the SST corpus repo if missing, decode .json.bin -> .json, then
# pack into .sstpack files for the C++ harness.
.PHONY: sst-clone
sst-clone:
	@if [ ! -d "$(SST_DIR)/v1" ]; then \
		echo "[sst] cloning SST repo into $(SST_DIR) ..."; \
		mkdir -p $(dir $(SST_DIR)); \
		git clone --depth 1 https://github.com/SingleStepTests/m68000.git $(SST_DIR); \
	fi

.PHONY: sst-packs
sst-packs: sst-clone
	@mkdir -p $(SST_PACK_DIR)
	@if ! ls $(SST_JSON_DIR)/*.json >/dev/null 2>&1; then \
		echo "[sst] decoding upstream .json.bin -> .json ..."; \
		( cd $(SST_DIR) && python3 decode.py ); \
	fi
	@python3 $(TOOLS_DIR)/sst/sst_pack.py $(SST_JSON_DIR) -o $(SST_PACK_DIR)

.PHONY: sst
sst: sst-build sst-packs
	@python3 $(TOOLS_DIR)/sst/sst_run.py \
		--bin $(SST_BIN) \
		--pack-dir $(SST_PACK_DIR) \
		$(if $(SST_FILE),--file $(SST_FILE),) \
		--start $(SST_START) \
		$(if $(SST_COUNT),--count $(SST_COUNT),) \
		--timeout $(SST_TIMEOUT) \
		--warmup $(SST_WARMUP) \
		$(if $(SST_VERBOSE),--verbose,) \
		$(if $(SST_CHECK_PC),--check-pc,) \
		$(if $(SST_CHECK_SR),--check-sr,)

# ──────────────────────────────────────────────────────────────────────────────
# AXI crossbar unit testbench (4-master × 2-slave system bus)
#
# Standalone Verilator build — axi_xbar.v has no dependencies on the core.
# ──────────────────────────────────────────────────────────────────────────────
AXI_XBAR_RTL   := $(RTL_DIR)/soc/axi_xbar.v
AXI_XBAR_BUILD := $(BUILD_DIR)/axi_xbar

.PHONY: tb-axi-xbar
DBG_BUS_BUILD := $(BUILD_DIR)/axi_dbg_bus

.PHONY: tb-axi-dbg-bus
W_SKID_BUILD := $(BUILD_DIR)/axi_w_skid

.PHONY: tb-axi-w-skid
tb-axi-w-skid: $(W_SKID_BUILD)/Vtb_axi_w_skid
	@echo "Running axi_w_skid unit tb..."
	$(W_SKID_BUILD)/Vtb_axi_w_skid

$(W_SKID_BUILD)/Vtb_axi_w_skid: $(RTL_DIR)/soc/axi_w_skid.v $(TB_DIR)/tb_axi_w_skid.cpp
	@mkdir -p $(W_SKID_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		-Wno-fatal \
		-I$(RTL_DIR)/soc -I$(RTL_DIR) \
		--Mdir $(W_SKID_BUILD) \
		--top-module axi_w_skid \
		$(RTL_DIR)/soc/axi_w_skid.v \
		$(TB_DIR)/tb_axi_w_skid.cpp \
		-o Vtb_axi_w_skid

IFG_BUILD := $(BUILD_DIR)/ifetch_window_guard
tb-ifetch-window-guard: $(IFG_BUILD)/Vifetch_window_guard
	@echo "Running ifetch_window_guard unit tb..."
	$(IFG_BUILD)/Vifetch_window_guard

$(IFG_BUILD)/Vifetch_window_guard: $(RTL_DIR)/soc/ifetch_window_guard.v $(TB_DIR)/tb_ifetch_window_guard.cpp
	@mkdir -p $(IFG_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		-Wno-fatal \
		-I$(RTL_DIR)/soc -I$(RTL_DIR) \
		--Mdir $(IFG_BUILD) \
		--top-module ifetch_window_guard \
		$(RTL_DIR)/soc/ifetch_window_guard.v \
		$(TB_DIR)/tb_ifetch_window_guard.cpp \
		-o Vifetch_window_guard

tb-axi-dbg-bus: $(DBG_BUS_BUILD)/Vaxi_dbg_bus
	@echo "Running axi_dbg_bus unit tb..."
	$(DBG_BUS_BUILD)/Vaxi_dbg_bus

$(DBG_BUS_BUILD)/Vaxi_dbg_bus: $(RTL_DIR)/soc/axi_dbg_bus.v $(TB_DIR)/tb_axi_dbg_bus.cpp
	@mkdir -p $(DBG_BUS_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		-Wno-fatal \
		-I$(RTL_DIR)/soc -I$(RTL_DIR) \
		-Mdir $(DBG_BUS_BUILD) \
		--top-module axi_dbg_bus \
		$(RTL_DIR)/soc/axi_dbg_bus.v \
		$(TB_DIR)/tb_axi_dbg_bus.cpp \
		-CFLAGS "-std=c++17"

tb-axi-xbar: $(AXI_XBAR_BUILD)/Vaxi_xbar
	@echo "Running axi_xbar unit tb..."
	$(AXI_XBAR_BUILD)/Vaxi_xbar

$(AXI_XBAR_BUILD)/Vaxi_xbar: $(AXI_XBAR_RTL) $(RTL_DIR)/soc/axi_defs.vh $(TB_DIR)/tb_axi_xbar.cpp
	@mkdir -p $(AXI_XBAR_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board \
		-Mdir $(AXI_XBAR_BUILD) \
		--top-module axi_xbar \
		-GWD_LOG2=12 -GWD_LOG2_S1=12 -GB_HOLD_LOG2=6 \
		-GENABLE_WD=1 \
		-GS3_BACKEND_SURVIVES_FLUSH=1 \
		-GS1_RST_TAIL_LOG2=5 \
		$(AXI_XBAR_RTL) \
		$(TB_DIR)/tb_axi_xbar.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# VRAM unit testbench (URAM-backed framebuffer)
#
# Standalone Verilator build — vram.v has no dependencies on the core.
# Exercises AXI4 slave writes/reads + streaming pixel-read port with
# async clocks.
# ──────────────────────────────────────────────────────────────────────────────
VRAM_RTL   := $(RTL_DIR)/board/vram.v
VRAM_BUILD := $(BUILD_DIR)/vram

.PHONY: tb-vram
tb-vram: $(VRAM_BUILD)/Vvram
	@echo "Running vram unit tb..."
	$(VRAM_BUILD)/Vvram

$(VRAM_BUILD)/Vvram: $(VRAM_RTL) $(TB_DIR)/tb_vram.cpp
	@mkdir -p $(VRAM_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board \
		-Mdir $(VRAM_BUILD) \
		--top-module vram \
		$(VRAM_RTL) \
		$(TB_DIR)/tb_vram.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# CPU→VRAM→scanner E2E unit testbench (tb/tb_vram_cpu_write.{v,cpp}).
#
# Proves the last-mile mac-logo path: a CPU-LSU-shaped AXI4 write burst
# into the `vram` slave is byte-visible on the streaming scanner read
# port.  Today the axi_xbar routes FB_BASE (0x6000_0000) to the DDR
# slave, and the vram AXI slave is only driven by `vram_smoke.v` when
# VIDEO_SMOKE=1 — i.e. there is NO CPU→VRAM-AXI path wired yet.  The
# tb therefore drives the vram slave directly with the traffic the
# future axi-xbar-vram retune will emit.  Covers BPP ∈ {8, 16}.
# ──────────────────────────────────────────────────────────────────────────────
VRAM_CPU_WRITE_RTL   := \
	$(RTL_DIR)/board/vram.v \
	$(TB_DIR)/tb_vram_cpu_write.v
VRAM_CPU_WRITE_BUILD := $(BUILD_DIR)/vram_cpu_write

.PHONY: tb-vram-cpu-write
tb-vram-cpu-write: $(VRAM_CPU_WRITE_BUILD)/Vtb_vram_cpu_write
	@echo "Running CPU→VRAM→scanner E2E unit tb..."
	$(VRAM_CPU_WRITE_BUILD)/Vtb_vram_cpu_write

$(VRAM_CPU_WRITE_BUILD)/Vtb_vram_cpu_write: $(VRAM_CPU_WRITE_RTL) $(TB_DIR)/tb_vram_cpu_write.cpp
	@mkdir -p $(VRAM_CPU_WRITE_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board \
		-Mdir $(VRAM_CPU_WRITE_BUILD) \
		--top-module tb_vram_cpu_write \
		$(VRAM_CPU_WRITE_RTL) \
		$(TB_DIR)/tb_vram_cpu_write.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# VRAM + xbar full-stack E2E tb (task #147 — CPU → axi_xbar → vram → scanner).
#
# Proves the CPU-visible VRAM aperture (0xF900_0000..0xF90F_FFFF) routes
# through the real xbar S3 port and lands in the URAM-backed vram slave
# with VRAM_BASE stripped.  Complements tb-vram-cpu-write (which drives
# the vram slave directly with no xbar in between).
# ──────────────────────────────────────────────────────────────────────────────
VRAM_XBAR_E2E_RTL   := \
	$(RTL_DIR)/soc/axi_xbar.v \
	$(RTL_DIR)/board/vram.v \
	$(TB_DIR)/tb_vram_xbar_e2e.v
VRAM_XBAR_E2E_BUILD := $(BUILD_DIR)/vram_xbar_e2e

N2W_VRAM_BYTE_RTL   := \
	$(RTL_DIR)/soc/axi_narrow_to_wide.v \
	$(RTL_DIR)/soc/axi_xbar.v \
	$(RTL_DIR)/board/vram.v \
	$(TB_DIR)/tb_vram_xbar_e2e.v \
	$(TB_DIR)/tb_n2w_vram_byte.v
N2W_VRAM_BYTE_BUILD := $(BUILD_DIR)/n2w_vram_byte

.PHONY: tb-vram-xbar-e2e
tb-vram-xbar-e2e: $(VRAM_XBAR_E2E_BUILD)/Vtb_vram_xbar_e2e
	@echo "Running CPU→xbar→VRAM→scanner E2E unit tb..."
	$(VRAM_XBAR_E2E_BUILD)/Vtb_vram_xbar_e2e

$(VRAM_XBAR_E2E_BUILD)/Vtb_vram_xbar_e2e: $(VRAM_XBAR_E2E_RTL) $(TB_DIR)/tb_vram_xbar_e2e.cpp $(RTL_DIR)/soc/axi_defs.vh
	@mkdir -p $(VRAM_XBAR_E2E_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board \
		-Mdir $(VRAM_XBAR_E2E_BUILD) \
		--top-module tb_vram_xbar_e2e \
		$(VRAM_XBAR_E2E_RTL) \
		$(TB_DIR)/tb_vram_xbar_e2e.cpp \
		-CFLAGS "-std=c++17"

.PHONY: tb-n2w-vram-byte
tb-n2w-vram-byte: $(N2W_VRAM_BYTE_BUILD)/Vtb_n2w_vram_byte
	@echo "Running narrow→xbar→VRAM byte roundtrip tb..."
	$(N2W_VRAM_BYTE_BUILD)/Vtb_n2w_vram_byte

$(N2W_VRAM_BYTE_BUILD)/Vtb_n2w_vram_byte: $(N2W_VRAM_BYTE_RTL) $(TB_DIR)/tb_n2w_vram_byte.cpp $(RTL_DIR)/soc/axi_defs.vh
	@mkdir -p $(N2W_VRAM_BYTE_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board \
		-Mdir $(N2W_VRAM_BYTE_BUILD) \
		--top-module tb_n2w_vram_byte \
		$(N2W_VRAM_BYTE_RTL) \
		$(TB_DIR)/tb_n2w_vram_byte.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# axi_narrow_to_wide abandonment-watchdog unit tb.
#
# axi_narrow_to_wide sits between every narrow master and the fabric and holds
# ONE outstanding transaction per direction, gating n_awready/n_arready on that
# latch — so a wide side that never answers wedges the master permanently.  The
# abandonment watchdog is the only thing that turns that into a diagnosable
# SLVERR instead of a permanent SoC hang, and until this tb landed nothing in
# the repo exercised it (tb_l2c_wstream.v's TIMEOUT_CYCLES override exists to
# STOP it firing; tb_n2w_vram_byte.v drives a chain that cannot be stalled).
#
# It ships DISABLED (ENABLE_ABANDON_TIMEOUT=0) because the progress term feeds
# a 20-second timer from the live L2C/xbar WREADY.  Both states are pinned
# here, as two separate Verilated builds from one source:
#
#   n2w_watchdog_on   -GENABLE_ABANDON_TIMEOUT=1 -GTIMEOUT_CYCLES=64
#       Scenarios 1-5 + 7 — expiry, SLVERR shape, beat accounting, the
#       no-false-trip guard, the exact boundary, rearm, and an adversarial
#       progress-on-the-expiry-cycle case.
#   n2w_watchdog_off  -GENABLE_ABANDON_TIMEOUT=0
#       Scenario 6 — with the timeout compiled out a stalled transaction must
#       hang CLEANLY: no counter, no synthesized SLVERR, no state teardown,
#       and a very late wide-side answer still completes normally.
#
# TIMEOUT_CYCLES=64 must match kTimeout in tb/tb_n2w_watchdog.cpp.
# ──────────────────────────────────────────────────────────────────────────────
N2W_WATCHDOG_RTL := \
	$(RTL_DIR)/soc/axi_narrow_to_wide.v \
	$(TB_DIR)/tb_n2w_watchdog.v
N2W_WATCHDOG_ON_BUILD  := $(BUILD_DIR)/n2w_watchdog_on
N2W_WATCHDOG_OFF_BUILD := $(BUILD_DIR)/n2w_watchdog_off

N2W_WATCHDOG_VFLAGS := --cc --exe --build --assert \
	--x-assign fast --x-initial fast -O3 \
	-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
	-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
	-I$(RTL_DIR)/soc \
	--top-module tb_n2w_watchdog

# Split into two runnable halves so the SHIPPING configuration
# (ENABLE_ABANDON_TIMEOUT=0) can be exercised on its own even while the
# enabled half is red.
.PHONY: tb-n2w-watchdog
tb-n2w-watchdog: tb-n2w-watchdog-off tb-n2w-watchdog-on

.PHONY: tb-n2w-watchdog-off
tb-n2w-watchdog-off: $(N2W_WATCHDOG_OFF_BUILD)/Vtb_n2w_watchdog
	@echo "Running axi_narrow_to_wide watchdog tb (ENABLE_ABANDON_TIMEOUT=0, shipping config)..."
	$(N2W_WATCHDOG_OFF_BUILD)/Vtb_n2w_watchdog

.PHONY: tb-n2w-watchdog-on
tb-n2w-watchdog-on: $(N2W_WATCHDOG_ON_BUILD)/Vtb_n2w_watchdog
	@echo "Running axi_narrow_to_wide watchdog tb (ENABLE_ABANDON_TIMEOUT=1)..."
	$(N2W_WATCHDOG_ON_BUILD)/Vtb_n2w_watchdog

$(N2W_WATCHDOG_ON_BUILD)/Vtb_n2w_watchdog: $(N2W_WATCHDOG_RTL) $(TB_DIR)/tb_n2w_watchdog.cpp
	@mkdir -p $(N2W_WATCHDOG_ON_BUILD)
	$(VERILATOR) $(N2W_WATCHDOG_VFLAGS) \
		-Mdir $(N2W_WATCHDOG_ON_BUILD) \
		-GENABLE_ABANDON_TIMEOUT=1 -GTIMEOUT_CYCLES=64 \
		$(N2W_WATCHDOG_RTL) \
		$(TB_DIR)/tb_n2w_watchdog.cpp \
		-CFLAGS "-std=c++17 -DN2W_WATCHDOG_ENABLED=1" \
		-o Vtb_n2w_watchdog

$(N2W_WATCHDOG_OFF_BUILD)/Vtb_n2w_watchdog: $(N2W_WATCHDOG_RTL) $(TB_DIR)/tb_n2w_watchdog.cpp
	@mkdir -p $(N2W_WATCHDOG_OFF_BUILD)
	$(VERILATOR) $(N2W_WATCHDOG_VFLAGS) \
		-Mdir $(N2W_WATCHDOG_OFF_BUILD) \
		-GENABLE_ABANDON_TIMEOUT=0 -GTIMEOUT_CYCLES=64 \
		$(N2W_WATCHDOG_RTL) \
		$(TB_DIR)/tb_n2w_watchdog.cpp \
		-CFLAGS "-std=c++17 -DN2W_WATCHDOG_ENABLED=0" \
		-o Vtb_n2w_watchdog

# ──────────────────────────────────────────────────────────────────────────────
# axi_narrow_to_wide MULTI-OUTSTANDING unit tb + depth curve.
#
# The adapter used to hold one transaction per direction; every narrow master
# behind it was serialized at one fabric round trip per transaction.  Nothing
# in the tree could measure that: tb_l2c_wstream's own master waits for B
# before the next AW ("single-outstanding narrow master" is in its banner),
# tb_axi_widen{,_watchdog} drive one transaction at a time by construction,
# and tb_sd_boot_top does not contain this module at all.
#
# tb_n2w_pipe drives a genuinely pipelined narrow master against a
# multi-outstanding wide slave with a 60-cycle response latency (what the
# shipping n2w -> l2c -> async bridge -> MIG chain measures) and reports
# narrow-side cycles/word.  One Verilated build per depth, so the curve is a
# real parameter sweep and not a master-side throttle standing in for one.
#
# Depth 1 additionally pins that the historical single-outstanding contract
# is exactly reproducible (scenario F), which is the safety net for anyone
# who needs to fall back.
#
# The watchdog is left ENABLED here (it ships disabled) with a 512-cycle
# window so the multi-outstanding abandonment scenarios are reachable; the
# perf scenarios never stall, so the timer never gets near it.
# ──────────────────────────────────────────────────────────────────────────────
N2W_PIPE_RTL := \
	$(RTL_DIR)/soc/axi_narrow_to_wide.v \
	$(TB_DIR)/tb_n2w_pipe.v

N2W_PIPE_VFLAGS := --cc --exe --build --assert \
	--x-assign fast --x-initial fast -O3 \
	-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
	-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND \
	-I$(RTL_DIR)/soc \
	--top-module tb_n2w_pipe

N2W_PIPE_DEPTHS := 1 2 4 8

.PHONY: tb-n2w-pipe
tb-n2w-pipe: $(foreach d,$(N2W_PIPE_DEPTHS),tb-n2w-pipe-$(d))

define N2W_PIPE_RULE
.PHONY: tb-n2w-pipe-$(1)
tb-n2w-pipe-$(1): $$(BUILD_DIR)/n2w_pipe_$(1)/Vtb_n2w_pipe
	@echo "Running axi_narrow_to_wide multi-outstanding tb (DEPTH=$(1))..."
	$$(BUILD_DIR)/n2w_pipe_$(1)/Vtb_n2w_pipe

$$(BUILD_DIR)/n2w_pipe_$(1)/Vtb_n2w_pipe: $$(N2W_PIPE_RTL) $$(TB_DIR)/tb_n2w_pipe.cpp
	@mkdir -p $$(BUILD_DIR)/n2w_pipe_$(1)
	$$(VERILATOR) $$(N2W_PIPE_VFLAGS) \
		-Mdir $$(BUILD_DIR)/n2w_pipe_$(1) \
		-GDEPTH=$(1) \
		$$(N2W_PIPE_RTL) \
		$$(TB_DIR)/tb_n2w_pipe.cpp \
		-CFLAGS "-std=c++17 -DN2W_PIPE_DEPTH=$(1)" \
		-o Vtb_n2w_pipe
endef
$(foreach d,$(N2W_PIPE_DEPTHS),$(eval $(call N2W_PIPE_RULE,$(d))))

# ──────────────────────────────────────────────────────────────────────────────
# First-hardware checkerboard smoke program + focused CPU→xbar→VRAM tb.
#
# Hardware image:
#   tools/hw_smoke/checkerboard.s writes a 1024x768 8bpp checkerboard into
#   the CPU-visible VRAM aperture.  The image target emits a one-sector flat
#   binary for first-board smoke flows that load short programs at reset.
#
# Fast sim:
#   Builds the same source with FB_BLOCKS_X=4, FB_ROWS=48 so it fills the
#   existing 128x48 tb_vram_xbar_e2e framebuffer.  The wrapper runs the CPU
#   program and routes D-side writes through axi_narrow_to_wide + xbar S3
#   into vram, then samples the scanner port for the expected checkerboard.
# ──────────────────────────────────────────────────────────────────────────────
HW_SMOKE_DIR      := $(BUILD_DIR)/hw_smoke
HW_SMOKE_ASM      := $(TOOLS_DIR)/hw_smoke/checkerboard.s
HW_SMOKE_HW_O     := $(HW_SMOKE_DIR)/checkerboard_hw.o
HW_SMOKE_HW_ELF   := $(HW_SMOKE_DIR)/checkerboard_hw.elf
HW_SMOKE_HW_BIN   := $(HW_SMOKE_DIR)/checkerboard.bin
HW_SMOKE_SD_IMG   := $(HW_SMOKE_DIR)/checkerboard_sd.img
HW_SMOKE_SIM_O    := $(HW_SMOKE_DIR)/checkerboard_sim.o
HW_SMOKE_SIM_ELF  := $(HW_SMOKE_DIR)/checkerboard_sim.elf
HW_SMOKE_SIM_BIN  := $(HW_SMOKE_DIR)/checkerboard_sim.bin
HW_DAFB_VRAM_ASM  := $(TOOLS_DIR)/hw_smoke/dafb_vram_pattern.s
HW_DAFB_VRAM_O    := $(HW_SMOKE_DIR)/dafb_vram_pattern.o
HW_DAFB_VRAM_ELF  := $(HW_SMOKE_DIR)/dafb_vram_pattern.elf
HW_DAFB_VRAM_BIN  := $(HW_SMOKE_DIR)/dafb_vram_pattern.bin

HW_CHECKER_RTL := \
	$(RTL_SRCS) \
	$(RTL_DIR)/soc/axi_narrow_to_wide.v \
	$(RTL_DIR)/soc/axi_xbar.v \
	$(RTL_DIR)/board/vram.v \
	$(TB_DIR)/tb_vram_xbar_e2e.v \
	$(TB_DIR)/tb_hw_checkerboard_path.v
HW_CHECKER_BUILD := $(BUILD_DIR)/hw_checkerboard_path

$(HW_SMOKE_HW_BIN): $(HW_SMOKE_ASM)
	@mkdir -p $(HW_SMOKE_DIR)
	$(M68K_AS) -m68040 -o $(HW_SMOKE_HW_O) $(HW_SMOKE_ASM)
	$(M68K_LD) -Ttext 0x40000000 -o $(HW_SMOKE_HW_ELF) $(HW_SMOKE_HW_O)
	$(M68K_OBJCOPY) -O binary $(HW_SMOKE_HW_ELF) $(HW_SMOKE_HW_BIN)
	@size=$$(stat -c%s $(HW_SMOKE_HW_BIN)); \
	if [ $$size -gt 512 ]; then \
		echo "checkerboard program is $$size bytes; must fit in one SD sector" >&2; \
		exit 1; \
	fi

$(HW_SMOKE_SD_IMG): $(HW_SMOKE_HW_BIN)
	@cp $(HW_SMOKE_HW_BIN) $(HW_SMOKE_SD_IMG)
	truncate -s 512 $(HW_SMOKE_SD_IMG)

.PHONY: hw-smoke-checkerboard-image
hw-smoke-checkerboard-image: $(HW_SMOKE_SD_IMG)
	@echo "Checkerboard SD sector image: $(HW_SMOKE_SD_IMG)"
	@echo "Program binary:               $(HW_SMOKE_HW_BIN)"

$(HW_DAFB_VRAM_BIN): $(HW_DAFB_VRAM_ASM)
	@mkdir -p $(HW_SMOKE_DIR)
	$(M68K_AS) -m68040 -o $(HW_DAFB_VRAM_O) $(HW_DAFB_VRAM_ASM)
	$(M68K_LD) -Ttext 0x40000000 -o $(HW_DAFB_VRAM_ELF) $(HW_DAFB_VRAM_O)
	$(M68K_OBJCOPY) -O binary $(HW_DAFB_VRAM_ELF) $(HW_DAFB_VRAM_BIN)
	@size=$$(stat -c%s $(HW_DAFB_VRAM_BIN)); \
	if [ $$size -gt 4096 ]; then \
		echo "DAFB/VRAM pattern ROM is $$size bytes; expected a small JTAG-load image" >&2; \
		exit 1; \
	fi

.PHONY: hw-smoke-dafb-vram-rom
hw-smoke-dafb-vram-rom: $(HW_DAFB_VRAM_BIN)
	@echo "DAFB/VRAM JTAG ROM image: $(HW_DAFB_VRAM_BIN)"

$(HW_SMOKE_SIM_BIN): $(HW_SMOKE_ASM)
	@mkdir -p $(HW_SMOKE_DIR)
	$(M68K_AS) -m68040 --defsym FB_BLOCKS_X=4 --defsym FB_ROWS=48 \
		-o $(HW_SMOKE_SIM_O) $(HW_SMOKE_ASM)
	$(M68K_LD) -Ttext 0x40800000 -o $(HW_SMOKE_SIM_ELF) $(HW_SMOKE_SIM_O)
	$(M68K_OBJCOPY) -O binary $(HW_SMOKE_SIM_ELF) $(HW_SMOKE_SIM_BIN)

.PHONY: tb-hw-checkerboard-path
tb-hw-checkerboard-path: $(HW_CHECKER_BUILD)/Vtb_hw_checkerboard_path $(HW_SMOKE_SIM_BIN)
	@echo "Running checkerboard CPU→xbar→VRAM smoke tb..."
	$(HW_CHECKER_BUILD)/Vtb_hw_checkerboard_path +bin=$(HW_SMOKE_SIM_BIN)

# HW_CHECKER_RTL pulled in the retired flat RTL_SRCS list (rtl/core/* +
# rtl/mac_top.v) to get a real CPU driving the checkerboard write — that
# tree moved to the cpu/ submodule.  Fail loud rather than silently
# building with an empty CPU source list.
$(HW_CHECKER_BUILD)/Vtb_hw_checkerboard_path:
	@echo "tb-hw-checkerboard-path was removed: rtl/core/* moved to the cpu/ submodule, so the flat-RTL_SRCS CPU->xbar->VRAM harness no longer builds from this repo. Use tb-vram-xbar-e2e / tb-n2w-vram-byte (xbar-only) or tb-fpga-top-rom (full CPU+SoC) instead." >&2
	@exit 2

# ──────────────────────────────────────────────────────────────────────────────
# VRAM smoke-preload unit tb (rtl/mac/video/vram_smoke.v + rtl/board/vram.v).
#
# Validates the VIDEO_SMOKE=1 reset-time bitmap preload path: the smoke
# writer issues AXI writes that fill VRAM with SMPTE bars, then we walk
# the streaming read port and compare every pixel against the golden.
# Also dumps build/video_smoke/frame.ppm for visual inspection.
# ──────────────────────────────────────────────────────────────────────────────
VIDEO_SMOKE_RTL   := \
	$(RTL_DIR)/board/vram.v \
	$(RTL_DIR)/board/video_phy/vram_smoke.v \
	$(TB_DIR)/tb_video_smoke.v
VIDEO_SMOKE_BUILD := $(BUILD_DIR)/video_smoke

.PHONY: tb-video-smoke
tb-video-smoke: $(VIDEO_SMOKE_BUILD)/Vtb_video_smoke
	@echo "Running VRAM smoke-preload unit tb..."
	$(VIDEO_SMOKE_BUILD)/Vtb_video_smoke

$(VIDEO_SMOKE_BUILD)/Vtb_video_smoke: $(VIDEO_SMOKE_RTL) $(TB_DIR)/tb_video_smoke.cpp
	@mkdir -p $(VIDEO_SMOKE_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED -Wno-BLKSEQ \
		$(if $(VRAM_SMOKE_DEBUG),+define+VRAM_SMOKE_DEBUG,) \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR)/board/video_phy -I$(RTL_DIR) \
		-Mdir $(VIDEO_SMOKE_BUILD) \
		--top-module tb_video_smoke \
		$(VIDEO_SMOKE_RTL) \
		$(TB_DIR)/tb_video_smoke.cpp \
		-CFLAGS "-std=c++17"

# User-facing wrapper: `make video-smoke` runs the tb and points at the PPM.
.PHONY: video-smoke
video-smoke: tb-video-smoke
	@echo ""
	@echo "──────────────────────────────────────────────────────────────"
	@echo " PPM framebuffer dumped to: build/video_smoke/frame.ppm"
	@echo " View with: eog build/video_smoke/frame.ppm"
	@echo " (or feh / display / xdg-open)"
	@echo " To convert to PNG: convert build/video_smoke/frame.ppm out.png"
	@echo "──────────────────────────────────────────────────────────────"

# ──────────────────────────────────────────────────────────────────────────────
# DMA controller unit testbench (programmable multi-channel AXI-master DMA)
#
# Standalone Verilator build — dma_ctrl.v has no dependencies on the core.
# Exercises the 4-channel DMA engine with descriptor chaining, burst
# boundaries, errors, back-pressure, and config-register readback.
# ──────────────────────────────────────────────────────────────────────────────
DMA_CTRL_RTL   := $(RTL_DIR)/soc/dma_ctrl.v
DMA_CTRL_BUILD := $(BUILD_DIR)/dma_ctrl
# dma_engine.sv registers its W channel through axi_w_skid (see the
# "W-channel output register slice" note in dma_engine.sv), so the skid
# must be in every dma_engine build or the module is a black box.
DMA_ENGINE_RTL := $(RTL_DIR)/soc/dma_engine.sv $(RTL_DIR)/soc/axi_w_skid.v
DMA_ENGINE_BUILD := $(BUILD_DIR)/dma_engine
DMA_ENGINE_WIDTHS := 128 256 512
DMA_L2C_BUILD := $(BUILD_DIR)/dma_l2c
DMA_L2C_RTL := \
	$(RTL_DIR)/soc/l2c_pri8.v \
	$(RTL_DIR)/soc/l2c_reset.v \
	$(RTL_DIR)/soc/l2c_tags.v \
	$(RTL_DIR)/soc/l2c_data.v \
	$(RTL_DIR)/soc/l2c_victim_sel.v \
	$(RTL_DIR)/soc/l2c_mshr.v \
	$(RTL_DIR)/soc/l2c_victim.v \
	$(RTL_DIR)/soc/l2c_bypass.v \
	$(RTL_DIR)/soc/l2c_ctrl.v \
	$(RTL_DIR)/soc/l2c.v

.PHONY: lint-dma-engine
lint-dma-engine:
	$(VERILATOR) --lint-only --cc -Wall -Wno-fatal -Wno-DECLFILENAME \
		--top-module dma_engine $(DMA_ENGINE_RTL)

.PHONY: tb-dma-engine
tb-dma-engine: $(foreach w,$(DMA_ENGINE_WIDTHS),$(DMA_ENGINE_BUILD)/$(w)/Vdma_engine)
	@set -e; for width in $(DMA_ENGINE_WIDTHS); do \
		$(DMA_ENGINE_BUILD)/$$width/Vdma_engine; \
	done

define DMA_ENGINE_WIDTH_RULE
$(DMA_ENGINE_BUILD)/$(1)/Vdma_engine: $(DMA_ENGINE_RTL) $(TB_DIR)/tb_dma_engine.cpp
	@mkdir -p $(DMA_ENGINE_BUILD)/$(1)
	$(VERILATOR) --cc --exe --build --assert -Wall -Wno-fatal -Wno-DECLFILENAME \
		--top-module dma_engine -GDATA_WIDTH=$(1) \
		-Mdir $(DMA_ENGINE_BUILD)/$(1) $(DMA_ENGINE_RTL) $(TB_DIR)/tb_dma_engine.cpp \
		-CFLAGS "-std=c++17 -DDATA_WIDTH_BUILD=$(1)"
endef
$(foreach w,$(DMA_ENGINE_WIDTHS),$(eval $(call DMA_ENGINE_WIDTH_RULE,$(w))))

.PHONY: tb-dma-engine-swap
tb-dma-engine-swap: $(DMA_ENGINE_BUILD)/swap/Vdma_engine
	@$(DMA_ENGINE_BUILD)/swap/Vdma_engine
	@echo "PASS: dma_engine BYTE_SWAP32 lane permutation"

$(DMA_ENGINE_BUILD)/swap/Vdma_engine: $(DMA_ENGINE_RTL) $(TB_DIR)/tb_dma_engine.cpp
	@mkdir -p $(DMA_ENGINE_BUILD)/swap
	$(VERILATOR) --cc --exe --build --assert -Wall -Wno-fatal -Wno-DECLFILENAME \
		--top-module dma_engine -GDATA_WIDTH=128 -GBYTE_SWAP32=1 \
		-Mdir $(DMA_ENGINE_BUILD)/swap $(DMA_ENGINE_RTL) $(TB_DIR)/tb_dma_engine.cpp \
		-CFLAGS "-std=c++17 -DDATA_WIDTH_BUILD=128 -DBYTE_SWAP32_BUILD"

# RED gate for the above: the same swapped memory model against an UNswapped
# engine must fail.  Without this, tb-dma-engine-swap would also pass on RTL
# that ignores BYTE_SWAP32 entirely.
.PHONY: tb-dma-engine-swap-mut
tb-dma-engine-swap-mut: $(DMA_ENGINE_BUILD)/swapmut/Vdma_engine
	@if $(DMA_ENGINE_BUILD)/swapmut/Vdma_engine >/dev/null 2>&1; then \
		echo "ERROR: dma_engine byte-swap test passed against an unswapped engine"; exit 1; \
	else \
		echo "PASS: dma_engine test rejects the unswapped-lane mutant"; \
	fi

$(DMA_ENGINE_BUILD)/swapmut/Vdma_engine: $(DMA_ENGINE_RTL) $(TB_DIR)/tb_dma_engine.cpp
	@mkdir -p $(DMA_ENGINE_BUILD)/swapmut
	$(VERILATOR) --cc --exe --build --assert -Wall -Wno-fatal -Wno-DECLFILENAME \
		--top-module dma_engine -GDATA_WIDTH=128 -GBYTE_SWAP32=0 \
		-Mdir $(DMA_ENGINE_BUILD)/swapmut $(DMA_ENGINE_RTL) $(TB_DIR)/tb_dma_engine.cpp \
		-CFLAGS "-std=c++17 -DDATA_WIDTH_BUILD=128 -DBYTE_SWAP32_BUILD"

.PHONY: tb-dma-engine-mut
tb-dma-engine-mut: $(DMA_ENGINE_BUILD)/mut/Vdma_engine
	@if $(DMA_ENGINE_BUILD)/mut/Vdma_engine >/dev/null 2>&1; then \
		echo "ERROR: dma_engine drop-last-byte mutant unexpectedly passed"; exit 1; \
	else \
		echo "PASS: dma_engine test rejects drop-last-byte mutant"; \
	fi

$(DMA_ENGINE_BUILD)/mut/Vdma_engine: $(DMA_ENGINE_RTL) $(TB_DIR)/tb_dma_engine.cpp
	@mkdir -p $(DMA_ENGINE_BUILD)/mut
	$(VERILATOR) --cc --exe --build --assert -Wall -Wno-fatal -Wno-DECLFILENAME \
		-DDMA_ENGINE_MUTANT_DROP_LAST_BYTE --top-module dma_engine -GDATA_WIDTH=128 \
		-Mdir $(DMA_ENGINE_BUILD)/mut $(DMA_ENGINE_RTL) $(TB_DIR)/tb_dma_engine.cpp \
		-CFLAGS "-std=c++17 -DDATA_WIDTH_BUILD=128"

.PHONY: tb-dma-l2c
tb-dma-l2c: $(DMA_L2C_BUILD)/Vtb_dma_l2c
	$(DMA_L2C_BUILD)/Vtb_dma_l2c

$(DMA_L2C_BUILD)/Vtb_dma_l2c: $(DMA_ENGINE_RTL) $(DMA_L2C_RTL) $(TB_DIR)/tb_dma_l2c.sv $(TB_DIR)/tb_dma_engine.cpp
	@mkdir -p $(DMA_L2C_BUILD)
	$(VERILATOR) --cc --exe --build --assert -Wall -Wno-fatal -Wno-DECLFILENAME \
		-I$(RTL_DIR)/soc --top-module tb_dma_l2c -Mdir $(DMA_L2C_BUILD) \
		$(DMA_ENGINE_RTL) $(DMA_L2C_RTL) \
		$(TB_DIR)/tb_dma_l2c.sv $(TB_DIR)/tb_dma_engine.cpp \
		-CFLAGS "-std=c++17 -DDATA_WIDTH_BUILD=128 -DDMA_L2C_BUILD"

.PHONY: tb-dma-ctrl
tb-dma-ctrl: $(DMA_CTRL_BUILD)/Vdma_ctrl
	@echo "Running dma_ctrl unit tb..."
	$(DMA_CTRL_BUILD)/Vdma_ctrl

$(DMA_CTRL_BUILD)/Vdma_ctrl: $(DMA_CTRL_RTL) $(TB_DIR)/tb_dma_ctrl.cpp
	@mkdir -p $(DMA_CTRL_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board \
		-Mdir $(DMA_CTRL_BUILD) \
		--top-module dma_ctrl \
		$(DMA_CTRL_RTL) \
		$(TB_DIR)/tb_dma_ctrl.cpp \
		-CFLAGS "-std=c++17"

SDBOOT_RTL   := \
	$(RTL_DIR)/board/sd_spi.v \
	$(RTL_DIR)/board/sd_spi_mux.v \
	$(RTL_DIR)/board/sd_ctrl.v \
	$(RTL_DIR)/soc/boot_fsm.v \
	$(TB_DIR)/tb_sd_boot_top.v
SDBOOT_BUILD := $(BUILD_DIR)/sd_boot

.PHONY: tb-sd-boot
tb-sd-boot: $(SDBOOT_BUILD)/Vtb_sd_boot_top
	@echo "Running sd boot FSM unit tb..."
	$(SDBOOT_BUILD)/Vtb_sd_boot_top

$(SDBOOT_BUILD)/Vtb_sd_boot_top: $(SDBOOT_RTL) $(TB_DIR)/tb_sd_boot.cpp
	@mkdir -p $(SDBOOT_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(SDBOOT_BUILD) \
		--top-module tb_sd_boot_top \
		$(SDBOOT_RTL) \
		$(TB_DIR)/tb_sd_boot.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SD boot FSM — RAM pre-zero pass + BRESP-error handling (task #167)
#
# Same sources and wrapper as tb-sd-boot, but elaborated with ZERO_BYTES != 0
# so boot_fsm's ST_ZERO_AW/W/B pass actually runs.  A separate binary because
# the zero pass adds AW transactions that tb-sd-boot's ROM-copy scenarios
# count exactly.  SDBOOT_ZERO_BYTES drives BOTH the Verilog parameter and the
# C++ TB_ZERO_BYTES define so the two cannot drift apart.
#
# Guards: a zeroing write that comes back SLVERR must drive boot_fsm into
# ST_ERROR, keep it there, and never raise rom_loaded — rom_loaded is what
# releases the CPU from reset (boot_rom_ready, fpga_top_clocks.vh), so
# swallowing the error puts the CPU on RAM that was never fully zeroed.
# ──────────────────────────────────────────────────────────────────────────────
SDBOOT_ZERO_BYTES ?= 4096
# Beats-minus-one per zero-pass AXI write burst.  Mirrors boot_fsm's own
# default (= the shape the SoC actually issues).  Override to re-measure the
# pass at another burst shape, e.g. the pre-2026-08-19 16-beat one:
#   make tb-sd-boot-zero SDBOOT_ZERO_AWLEN=15 SDBOOT_ZERO_BYTES=262144
# ZERO_BYTES must stay a whole number of bursts -- boot_fsm has an
# elaboration guard for that.  The build dir carries both knobs so switching
# shapes cannot silently re-run a stale binary.
SDBOOT_ZERO_AWLEN ?= 63
SDBOOT_ZERO_BUILD := $(BUILD_DIR)/sd_boot_zero_$(SDBOOT_ZERO_BYTES)_$(SDBOOT_ZERO_AWLEN)

.PHONY: tb-sd-boot-zero
tb-sd-boot-zero: $(SDBOOT_ZERO_BUILD)/Vtb_sd_boot_top
	@echo "Running sd boot FSM RAM-zero-pass unit tb (ZERO_BYTES=$(SDBOOT_ZERO_BYTES), ZERO_AWLEN=$(SDBOOT_ZERO_AWLEN))..."
	$(SDBOOT_ZERO_BUILD)/Vtb_sd_boot_top +zero_pass

$(SDBOOT_ZERO_BUILD)/Vtb_sd_boot_top: $(SDBOOT_RTL) $(TB_DIR)/tb_sd_boot.cpp
	@mkdir -p $(SDBOOT_ZERO_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(SDBOOT_ZERO_BUILD) \
		--top-module tb_sd_boot_top \
		-GZERO_BYTES=$(SDBOOT_ZERO_BYTES) \
		-GZERO_AWLEN=$(SDBOOT_ZERO_AWLEN) \
		$(SDBOOT_RTL) \
		$(TB_DIR)/tb_sd_boot.cpp \
		-CFLAGS "-std=c++17 -DTB_ZERO_BYTES=$(SDBOOT_ZERO_BYTES)u"

# ──────────────────────────────────────────────────────────────────────────────
# SD boot FSM — ROM-mirror + VRAM-zero (boot_fsm MIRROR_LOW_RAM)
#
# Same sources and wrapper as tb-sd-boot, elaborated with MIRROR_LOW_RAM=1
# so every word streamed from SD is also written a second time at the
# identical relative offset off address 0 -- the fix that lets cpu040's
# axi_i (which bypasses the crossbar's ROM-overlay redirect entirely, see
# boot_fsm.v's own MIRROR_LOW_RAM parameter doc) find correct ROM content
# in RAM at reset with no runtime redirect logic.  A separate binary
# because the mirror doubles every AW/W transaction the ROM-copy phase
# issues, which tb-sd-boot's plain scenarios count exactly.
#
# SDBOOT_MIRROR_IMAGE_BYTES must cover the tb's whole streamed image
# (NUM_SECTORS*512 = 16*512 = 8192 = 0x2000 bytes); SDBOOT_MIRROR_ZERO_BYTES
# is the low-RAM pre-zero pass's end address, which must sit at or above
# it (the pass starts AT MIRROR_IMAGE_BYTES under MIRROR_LOW_RAM, so it
# never re-zeroes the mirror it just wrote).  Both, minus
# MIRROR_IMAGE_BYTES, and SDBOOT_MIRROR_VRAM_BYTES must be whole multiples
# of (SDBOOT_MIRROR_AWLEN+1)*4 bytes -- boot_fsm has elaboration guards for
# the ZERO_BYTES and VRAM_ZERO_BYTES cases.  Defaults here pick a small
# AWLEN (15 -> 64 B/burst) so the tb stays fast while still exercising a
# multi-beat burst, and a VRAM window at a base address well clear of the
# mirror+zero region so the sparse backing-store map can't alias the two.
SDBOOT_MIRROR_IMAGE_BYTES ?= 8192
SDBOOT_MIRROR_ZERO_BYTES  ?= 12288
SDBOOT_MIRROR_AWLEN       ?= 15
SDBOOT_MIRROR_VRAM_BASE   ?= 1048576
SDBOOT_MIRROR_VRAM_BYTES  ?= 1024
SDBOOT_MIRROR_BUILD := $(BUILD_DIR)/sd_boot_mirror

.PHONY: tb-sd-boot-mirror
tb-sd-boot-mirror: $(SDBOOT_MIRROR_BUILD)/Vtb_sd_boot_top
	@echo "Running sd boot FSM ROM-mirror + VRAM-zero unit tb (MIRROR_IMAGE_BYTES=$(SDBOOT_MIRROR_IMAGE_BYTES), ZERO_BYTES=$(SDBOOT_MIRROR_ZERO_BYTES), VRAM_BYTES=$(SDBOOT_MIRROR_VRAM_BYTES))..."
	$(SDBOOT_MIRROR_BUILD)/Vtb_sd_boot_top +mirror

$(SDBOOT_MIRROR_BUILD)/Vtb_sd_boot_top: $(SDBOOT_RTL) $(TB_DIR)/tb_sd_boot.cpp
	@mkdir -p $(SDBOOT_MIRROR_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(SDBOOT_MIRROR_BUILD) \
		--top-module tb_sd_boot_top \
		-GMIRROR_LOW_RAM=1 \
		-GMIRROR_IMAGE_BYTES=$(SDBOOT_MIRROR_IMAGE_BYTES) \
		-GZERO_BYTES=$(SDBOOT_MIRROR_ZERO_BYTES) \
		-GZERO_AWLEN=$(SDBOOT_MIRROR_AWLEN) \
		-GVRAM_ZERO_BASE=$(SDBOOT_MIRROR_VRAM_BASE) \
		-GVRAM_ZERO_BYTES=$(SDBOOT_MIRROR_VRAM_BYTES) \
		$(SDBOOT_RTL) \
		$(TB_DIR)/tb_sd_boot.cpp \
		-CFLAGS "-std=c++17 -DTB_MIRROR_LOW_RAM=1 \
			-DTB_MIRROR_IMAGE_BYTES=$(SDBOOT_MIRROR_IMAGE_BYTES)u \
			-DTB_ZERO_BYTES=$(SDBOOT_MIRROR_ZERO_BYTES)u \
			-DTB_VRAM_ZERO_BASE=$(SDBOOT_MIRROR_VRAM_BASE)u \
			-DTB_VRAM_ZERO_BYTES=$(SDBOOT_MIRROR_VRAM_BYTES)u"

# ──────────────────────────────────────────────────────────────────────────────
# Cold-boot unit testbench (task #140)
#
# Exercises the full SD → boot_fsm → narrow_to_wide → [C++ DDR aperture] →
# if_to_axi → if_stage path in isolation.  Keeps the recent if_to_axi.v
# 4-word reversal honest without paying a 12M-cycle tb-rom-boot run.
#
# No sd_spi, no xbar, no full m68k_core — just the five modules listed
# above.  A synthetic SD byte source lives in tb/tb_cold_boot.cpp.
# ──────────────────────────────────────────────────────────────────────────────
COLDBOOT_PERIPH_SRC_DIR := $(TB_DIR)/tests/cold_boot_periph
COLDBOOT_PERIPH_BUILD   := $(BUILD_DIR)/cold_boot_periph
COLDBOOT_PERIPH_ASM     := $(wildcard $(COLDBOOT_PERIPH_SRC_DIR)/*.s)
COLDBOOT_PERIPH_BINS    := $(patsubst $(COLDBOOT_PERIPH_SRC_DIR)/%.s,$(COLDBOOT_PERIPH_BUILD)/%.bin,$(COLDBOOT_PERIPH_ASM))

$(COLDBOOT_PERIPH_BUILD)/%.bin: $(COLDBOOT_PERIPH_SRC_DIR)/%.s
	@mkdir -p $(COLDBOOT_PERIPH_BUILD)
	$(M68K_AS) -m68040 -o $(COLDBOOT_PERIPH_BUILD)/$*.o $<
	$(M68K_LD) -Ttext 0x40000000 -o $(COLDBOOT_PERIPH_BUILD)/$*.elf $(COLDBOOT_PERIPH_BUILD)/$*.o
	$(M68K_OBJCOPY) -O binary $(COLDBOOT_PERIPH_BUILD)/$*.elf $@
	@size=$$(stat -c%s $@); \
	if [ $$size -gt 256 ]; then \
		echo "$@ is $$size bytes; directed cold-boot ROMs must stay <= 256 bytes" >&2; \
		exit 1; \
	fi

COLDBOOT_RTL   := \
	$(filter-out $(RTL_DIR)/mac_top.v,$(RTL_SRCS)) \
	$(RTL_DIR)/board/sd_ctrl.v \
	$(RTL_DIR)/soc/boot_fsm.v \
	$(RTL_DIR)/soc/axi_narrow_to_wide.v \
	$(RTL_DIR)/soc/if_to_axi.v \
	$(RTL_DIR)/soc/axi_xbar.v \
	$(RTL_DIR)/soc/axi_wide_to_axilite.v \
	$(RTL_DIR)/board/ddr_ctrl.v \
	$(RTL_DIR)/soc/peripheral_bus.v \
	$(RTL_DIR)/mac/rtc.v \
	$(TB_DIR)/tb_cold_boot.v
COLDBOOT_BUILD := $(BUILD_DIR)/cold_boot
COLDBOOT_EXTRA ?=
COLDBOOT_Q700_DEEP_CYCLES ?= 10000000
COLDBOOT_Q700_DEEP_STATE ?= $(COLDBOOT_BUILD)/q700_10m_state.txt

.PHONY: tb-cold-boot
# tb-cold-boot is currently DISABLED at the user's request (2026-04-26):
# 10/11 scenarios fail with `boot bad (loaded=0 err=0 cause=0)` against
# the boot-FSM/peripheral-island wiring on main HEAD; runtime is ~95s
# even with the recent FULL_BOOT_MAX_CYCLES cut from 7M to 1M.  The
# failures are pre-existing on main and not caused by any individual
# RTL landing — they reflect a long-standing gap between the cold-boot
# scenarios and the current peripheral wiring.  Re-enable once the
# scenarios are updated to match the live ROM/boot path (or the
# runtime budget allows the existing scenarios to converge).
tb-cold-boot: $(COLDBOOT_BUILD)/Vtb_cold_boot $(COLDBOOT_PERIPH_BINS)
	@echo "tb-cold-boot is DISABLED on main HEAD (10/11 scenarios pre-existing FAIL, ~95s runtime)."
	@echo "To run anyway: $(COLDBOOT_BUILD)/Vtb_cold_boot $(COLDBOOT_EXTRA)"

.PHONY: tb-cold-boot-q700-deep
tb-cold-boot-q700-deep: $(COLDBOOT_BUILD)/Vtb_cold_boot $(COLDBOOT_PERIPH_BINS)
	@echo "Running Q700 cold-boot deep path for $(COLDBOOT_Q700_DEEP_CYCLES) cycles..."
	$(COLDBOOT_BUILD)/Vtb_cold_boot +q700_deep \
		+cycles=$(COLDBOOT_Q700_DEEP_CYCLES) \
		+state_dump=$(COLDBOOT_Q700_DEEP_STATE)
	@echo "tb-cold-boot-q700-deep: state at $(COLDBOOT_Q700_DEEP_STATE)"

# COLDBOOT_RTL pulled in the retired flat RTL_SRCS list (rtl/core/* +
# rtl/mac_top.v, minus mac_top.v) for a real CPU — that tree moved to the
# cpu/ submodule.  tb-cold-boot was already policy-disabled (see above);
# fail the underlying build loud instead of letting it silently try (and
# fail differently) with an empty CPU source list.
$(COLDBOOT_BUILD)/Vtb_cold_boot:
	@echo "tb-cold-boot(-q700-deep) was removed: rtl/core/* moved to the cpu/ submodule, so the flat-RTL_SRCS cold-boot harness no longer builds from this repo. Use tb-fpga-top-rom (full CPU+SoC) instead." >&2
	@exit 2

# Cold-boot vec-0 overlay variant — rebuilds tb_cold_boot with
# FETCH_RESET_VECTORS=1 so the CPU performs a real 68k reset-vector fetch
# (SSP from addr 0, PC from addr 4) through the overlay + axi_xbar +
# ddr_ctrl + if_to_axi path instead of starting at the RESET_PC parameter.
# This is the integration-level check for the HW first-light vec-0
# regression (see memory entry project_hw_firstlight_investigation_...).
# The binary runs the same tb_cold_boot.cpp harness; the cpp's main() gates
# vec-0-specific scenarios on argv so they are skipped under the default
# FETCH_RESET_VECTORS=0 build and only exercised here.
COLDBOOT_VEC0_BUILD := $(BUILD_DIR)/cold_boot_vec0

.PHONY: tb-cold-boot-vec0
tb-cold-boot-vec0: $(COLDBOOT_VEC0_BUILD)/Vtb_cold_boot $(COLDBOOT_PERIPH_BINS)
	@echo "Running cold-boot path unit tb (FETCH_RESET_VECTORS=1)..."
	$(COLDBOOT_VEC0_BUILD)/Vtb_cold_boot vec0

$(COLDBOOT_VEC0_BUILD)/Vtb_cold_boot:
	@echo "tb-cold-boot-vec0 was removed: rtl/core/* moved to the cpu/ submodule, so the flat-RTL_SRCS cold-boot harness no longer builds from this repo. Use tb-fpga-top-rom (full CPU+SoC) instead." >&2
	@exit 2

# ──────────────────────────────────────────────────────────────────────────────
# (sd_provision unit testbench REMOVED — sd_provision was deleted; boot is
# via JTAG-AXI now and SD provisioning happens through host-side flashing
# of the SD card before the FPGA powers up.  The boot-side tb-sd-ctrl
# below is the SHARED infrastructure unit test that remains.)
# ──────────────────────────────────────────────────────────────────────────────

# ──────────────────────────────────────────────────────────────────────────────
# sd_ctrl focused unit testbench (unified SD command-transport engine)
#
# Standalone build: sd_ctrl.v + sd_spi.v wrapped by tb/tb_sd_ctrl.v,
# exercised by tb/tb_sd_ctrl.cpp.  Covers CMD17/CMD18/CMD24/CMD25 and
# the error paths (R1 bad, R1 timeout, DR bad, unknown cmd_type).
# ──────────────────────────────────────────────────────────────────────────────
SDCTRL_RTL   := \
	$(RTL_DIR)/board/sd_spi.v \
	$(RTL_DIR)/board/sd_ctrl.v \
	$(TB_DIR)/tb_sd_ctrl.v
SDCTRL_BUILD := $(BUILD_DIR)/sd_ctrl

.PHONY: tb-sd-ctrl
tb-sd-ctrl: $(SDCTRL_BUILD)/Vtb_sd_ctrl
	@echo "Running sd_ctrl unit tb..."
	$(SDCTRL_BUILD)/Vtb_sd_ctrl

$(SDCTRL_BUILD)/Vtb_sd_ctrl: $(SDCTRL_RTL) $(TB_DIR)/tb_sd_ctrl.cpp
	@mkdir -p $(SDCTRL_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(SDCTRL_BUILD) \
		--top-module tb_sd_ctrl \
		$(SDCTRL_RTL) \
		$(TB_DIR)/tb_sd_ctrl.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# sd_jtag_writer focused unit testbench (JTAG AXI-Lite -> CMD24 writer)
# ──────────────────────────────────────────────────────────────────────────────
SDJTAG_WRITER_RTL := \
	$(RTL_DIR)/board/sd_ctrl.v \
	$(RTL_DIR)/board/sd_jtag_writer.v
SDJTAG_WRITER_BUILD := $(BUILD_DIR)/sd_jtag_writer

.PHONY: tb-sd-jtag-writer
tb-sd-jtag-writer: $(SDJTAG_WRITER_BUILD)/Vsd_jtag_writer
	@echo "Running sd_jtag_writer unit tb..."
	$(SDJTAG_WRITER_BUILD)/Vsd_jtag_writer

$(SDJTAG_WRITER_BUILD)/Vsd_jtag_writer: $(SDJTAG_WRITER_RTL) $(TB_DIR)/tb_sd_jtag_writer.cpp
	@mkdir -p $(SDJTAG_WRITER_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(SDJTAG_WRITER_BUILD) \
		--top-module sd_jtag_writer \
		$(SDJTAG_WRITER_RTL) \
		$(TB_DIR)/tb_sd_jtag_writer.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# pram_sd — manual PRAM <-> SD sector persistence (rtl/soc/pram_sd.v)
# ──────────────────────────────────────────────────────────────────────────────
# Wrapper tb/tb_pram_sd_top.v stitches the real pram_sd + pram_cdc + rtc
# together in their two real clock domains; tb/tb_pram_sd.cpp supplies an
# SPI-byte-level SD card model and drives BOTH access paths to the PRAM
# array (pram_sd's snapshot AND rtc.v's own serial protocol), so a content
# check can never be satisfied by a private shadow copy.
#
# NOTE the x-initial/x-assign mode below is `unique`, NOT the `fast` that
# every other target in this Makefile uses.  pram_sd's 512-byte staging
# buffer is deliberately un-reset (a 512-entry reset loop is a big part of
# why sd_jtag_writer costs ~17.5K LUTs); under `fast` an uninitialised read
# returns a tidy zero and a read-before-write bug in that buffer would be
# structurally invisible to this suite.  Do not "harmonise" this back to
# fast — that silently deletes the coverage.
PRAM_SD_RTL := \
	$(RTL_DIR)/board/sd_ctrl.v \
	$(RTL_DIR)/soc/pram_cdc.v \
	$(RTL_DIR)/soc/pram_sd.v \
	$(RTL_DIR)/mac/rtc.v \
	$(TB_DIR)/tb_pram_sd_top.v
PRAM_SD_BUILD := $(BUILD_DIR)/pram_sd

.PHONY: tb-pram-sd-autoload
tb-pram-sd-autoload: $(PRAM_SD_BUILD)_autoload/Vtb_pram_sd_top
	@echo "Running pram_sd boot-autoload tb (AUTOLOAD_ON_BOOT=1)..."
	$(PRAM_SD_BUILD)_autoload/Vtb_pram_sd_top +autoload

# A sibling directory prevents VPATH=.. from reusing the manual-mode object.
$(PRAM_SD_BUILD)_autoload/Vtb_pram_sd_top: $(PRAM_SD_RTL) $(TB_DIR)/tb_pram_sd.cpp
	@mkdir -p $(PRAM_SD_BUILD)_autoload
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign unique --x-initial unique -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-MULTIDRIVEN \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-GAUTOLOAD_ON_BOOT=1 \
		-Mdir $(PRAM_SD_BUILD)_autoload \
		--top-module tb_pram_sd_top \
		$(PRAM_SD_RTL) \
		$(TB_DIR)/tb_pram_sd.cpp \
		-CFLAGS "-std=c++17 -DAUTOLOAD_BUILD"

.PHONY: tb-pram-sd tb-pram-sd-populated
tb-pram-sd: $(PRAM_SD_BUILD)/Vtb_pram_sd_top
	@echo "Running pram_sd unit tb (post-reset default PRAM image)..."
	$(PRAM_SD_BUILD)/Vtb_pram_sd_top

# Same binary, but rtc.v's post-reset image becomes the populated "SCBI"
# variant.  The defaults-fallback scenarios capture that image at runtime,
# so this run FAILS if pram_sd ever hardcodes "fall back to zeros" instead
# of reusing rtc.v's own pram_clear.  That is the whole point of the run.
tb-pram-sd-populated: $(PRAM_SD_BUILD)/Vtb_pram_sd_top
	@echo "Running pram_sd unit tb (+rtc_populated_pram default image)..."
	$(PRAM_SD_BUILD)/Vtb_pram_sd_top +rtc_populated_pram

$(PRAM_SD_BUILD)/Vtb_pram_sd_top: $(PRAM_SD_RTL) $(TB_DIR)/tb_pram_sd.cpp
	@mkdir -p $(PRAM_SD_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign unique --x-initial unique -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-MULTIDRIVEN \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(PRAM_SD_BUILD) \
		--top-module tb_pram_sd_top \
		$(PRAM_SD_RTL) \
		$(TB_DIR)/tb_pram_sd.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# axil_split2 — two-way AXI-Lite address demux behind the SD-JTAG window
# ──────────────────────────────────────────────────────────────────────────────
AXIL_SPLIT2_RTL   := $(RTL_DIR)/soc/axil_split2.v
AXIL_SPLIT2_BUILD := $(BUILD_DIR)/axil_split2

.PHONY: tb-axil-split2
tb-axil-split2: $(AXIL_SPLIT2_BUILD)/Vaxil_split2
	@echo "Running axil_split2 unit tb..."
	$(AXIL_SPLIT2_BUILD)/Vaxil_split2

$(AXIL_SPLIT2_BUILD)/Vaxil_split2: $(AXIL_SPLIT2_RTL) $(TB_DIR)/tb_axil_split2.cpp
	@mkdir -p $(AXIL_SPLIT2_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign unique --x-initial unique -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(AXIL_SPLIT2_BUILD) \
		--top-module axil_split2 \
		$(AXIL_SPLIT2_RTL) \
		$(TB_DIR)/tb_axil_split2.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SD provisioning bitstream — fast bulk SD-write path (burst JTAG-AXI ->
# staging BRAM -> CMD25).  Standalone top (rtl/soc/sd_provision_top.v),
# NOT part of fpga_top.  Unit tb drives sd_provision_core through the
# tb_sd_provision_top.v wrapper: card init (boot_fsm) + burst staging +
# bulk CMD25 write + CMD18 CRC verify against a bit-level SD card model.
# ──────────────────────────────────────────────────────────────────────────────
SDPROV_CORE_RTL := \
	$(RTL_DIR)/board/sd_spi.v \
	$(RTL_DIR)/board/sd_spi_mux.v \
	$(RTL_DIR)/board/sd_ctrl.v \
	$(RTL_DIR)/board/sd_bulk_writer.v \
	$(RTL_DIR)/soc/boot_fsm.v \
	$(RTL_DIR)/soc/sd_provision_core.v
SDPROV_BUILD := $(BUILD_DIR)/sd_provision_tb

.PHONY: tb-sd-provision
tb-sd-provision: $(SDPROV_BUILD)/Vtb_sd_provision_top
	@echo "Running sd_provision bulk-writer unit tb..."
	$(SDPROV_BUILD)/Vtb_sd_provision_top

$(SDPROV_BUILD)/Vtb_sd_provision_top: $(SDPROV_CORE_RTL) \
		$(TB_DIR)/tb_sd_provision_top.v $(TB_DIR)/tb_sd_provision.cpp
	@mkdir -p $(SDPROV_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(SDPROV_BUILD) \
		--top-module tb_sd_provision_top \
		$(SDPROV_CORE_RTL) \
		$(TB_DIR)/tb_sd_provision_top.v \
		$(TB_DIR)/tb_sd_provision.cpp \
		-CFLAGS "-std=c++17"

# Lint the full provisioning top (clocking + IP tie-off path included).
.PHONY: lint-sd-provision
lint-sd-provision:
	$(VERILATOR) --lint-only --cc \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Wall \
		-Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-WIDTHTRUNC -Wno-WIDTHEXPAND -Wno-DECLFILENAME \
		-Wno-CASEINCOMPLETE \
		--top-module sd_provision_top \
		$(SDPROV_CORE_RTL) \
		$(RTL_DIR)/board/clk_rst.v \
		$(RTL_DIR)/board/reset_debounce.v \
		$(RTL_DIR)/soc/sd_provision_top.v \
		$(TB_DIR)/verilator_xilinx_stubs.v

# Dedicated provisioning bitstream (synth-only sanity / full impl).
# Output: $(BUILD_DIR)/sd_provision/sd_provision_top.bit
SDPROV_VIVADO_DIR := $(BUILD_DIR)/sd_provision

.PHONY: sd-provision-synth
sd-provision-synth:
	@mkdir -p $(SDPROV_VIVADO_DIR)
	@touch $(VIVADO_LOCK)
	flock -n -E 75 $(VIVADO_LOCK) -c '$(VIVADO) -mode batch -source $(SYNTH_DIR)/sd_provision.tcl -tclargs synth_only $(SDPROV_VIVADO_DIR)' \
	  || { status=$$?; if [ $$status -eq 75 ]; then echo "ERROR: another Vivado run holds $(VIVADO_LOCK) — fall back to sim work"; fi; exit $$status; }

.PHONY: sd-provision-impl
sd-provision-impl:
	@mkdir -p $(SDPROV_VIVADO_DIR)
	@touch $(VIVADO_LOCK)
	flock -n -E 75 $(VIVADO_LOCK) -c '$(VIVADO) -mode batch -source $(SYNTH_DIR)/sd_provision.tcl -tclargs full_impl $(SDPROV_VIVADO_DIR)' \
	  || { status=$$?; if [ $$status -eq 75 ]; then echo "ERROR: another Vivado run holds $(VIVADO_LOCK) — fall back to sim work"; fi; exit $$status; }

# tb-scaler removed: rtl/board/video_phy/scaler.v was deleted (superseded
# by linebuf_scanout — see video_top.v, which instantiates linebuf_scanout
# and never scaler).

# ──────────────────────────────────────────────────────────────────────────────
# mode_admit unit testbench -- the scan-out ADMISSION AUTHORITY (stage 3)
#
# mode_admit.v owns every admission inequality in the video path.  It is a pure
# combinational function, so this gate is a table: tb_mode_admit.cpp drives
# tuples and compares every output against a model written from the CONTRACT
# (a pixel-domain ceiling division), never transcribed from the RTL.
#
# mode_admit.v MUST be listed here explicitly.  Verilator would find it anyway
# via -I$(RTL_DIR)/board/video_phy, and it would then not be a make
# prerequisite -- so editing it would rebuild nothing and every mutant would
# "pass".  That has bitten this repo before; see docs/video_path_review.md.
# ──────────────────────────────────────────────────────────────────────────────
MODE_ADMIT_RTL := $(RTL_DIR)/board/video_phy/mode_admit.v $(TB_DIR)/tb_mode_admit.v
MODE_ADMIT_BUILD := $(BUILD_DIR)/mode_admit

.PHONY: tb-mode-admit
tb-mode-admit: $(MODE_ADMIT_BUILD)/Vtb_mode_admit
	@echo "Running mode_admit (scan-out admission authority) unit tb..."
	$(MODE_ADMIT_BUILD)/Vtb_mode_admit

$(MODE_ADMIT_BUILD)/Vtb_mode_admit: $(MODE_ADMIT_RTL) $(TB_DIR)/tb_mode_admit.cpp
	@mkdir -p $(MODE_ADMIT_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-j $(VERILATOR_JOBS) --threads $(VERILATOR_THREADS) \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR) \
		-Mdir $(MODE_ADMIT_BUILD) \
		--top-module tb_mode_admit \
		$(MODE_ADMIT_RTL) \
		$(TB_DIR)/tb_mode_admit.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# Scanout placement synchronizer unit testbench
#
# Standalone build for the DAFB base/stride CDC helper used by video_top.
# ──────────────────────────────────────────────────────────────────────────────
SCANOUT_PLACEMENT_SYNC_RTL := $(RTL_DIR)/board/video_phy/mode_admit.v $(RTL_DIR)/board/video_phy/place_plan.v $(RTL_DIR)/board/video_phy/scanout_placement_sync.v $(TB_DIR)/tb_scanout_placement_sync_wrap.v
SCANOUT_PLACEMENT_SYNC_BUILD := $(BUILD_DIR)/scanout_placement_sync

.PHONY: tb-scanout-placement-sync
tb-scanout-placement-sync: $(SCANOUT_PLACEMENT_SYNC_BUILD)/Vtb_scanout_placement_sync_wrap
	@echo "Running scanout placement sync unit tb..."
	$(SCANOUT_PLACEMENT_SYNC_BUILD)/Vtb_scanout_placement_sync_wrap

$(SCANOUT_PLACEMENT_SYNC_BUILD)/Vtb_scanout_placement_sync_wrap: $(SCANOUT_PLACEMENT_SYNC_RTL) $(TB_DIR)/tb_scanout_placement_sync.cpp
	@mkdir -p $(SCANOUT_PLACEMENT_SYNC_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-j $(VERILATOR_JOBS) --threads $(VERILATOR_THREADS) \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR) \
		-Mdir $(SCANOUT_PLACEMENT_SYNC_BUILD) \
		--top-module tb_scanout_placement_sync_wrap \
		$(SCANOUT_PLACEMENT_SYNC_RTL) \
		$(TB_DIR)/tb_scanout_placement_sync.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# tb-scanout-display-latency -- THE FIVE-STAGE LOCKSTEP GATE.
#
# scanout_display.v's latency ladder is five registered stages from a
# combinational x_src to rgb, and six parallel control pipes -- de, hs, vs,
# border, pix_valid, splash -- must match it exactly.  Since the 2026-08-20
# decomposition (stages 9-12 split into pixel_unpack / clut / upscale /
# compositor) that ladder spans three files.
#
# docs/video_path_review.md S6 lists the alignment under "what is NOT wrong"
# with "Hand-verified" as its entire evidence.  This target measures it.
#
# WHY NOT JUST tb-scanout-ddr-frames.  That gate compares every active pixel,
# so it does notice a slipped pipe -- but it reports "first_bad = (line, col)",
# which is the same message a fetch underrun, a bad palette entry or a wrong
# address gives.  This one sweeps the candidate latency 0..9 for EACH pipe
# independently and reports the pipe name and the depth it actually has.  It
# also runs in seconds against a 320x200 synthetic raster, so it belongs in the
# inner loop where the full-chain gates do not.
#
# tb-scanout-display-latency-negctl PROVES the sensitivity rather than assuming
# it: -DSCANOUT_INJECT_SHORT_BORDER_PIPE makes compositor.v tap border_pipe one
# stage early -- literally the reported hardware symptom at border_x -- and the
# gate must FAIL.  Keep both halves.
# ──────────────────────────────────────────────────────────────────────────────
SCANOUT_DISPLAY_LATENCY_RTL := \
	$(RTL_DIR)/board/video_phy/place_plan.v \
	$(RTL_DIR)/board/video_phy/pixel_unpack.v \
	$(RTL_DIR)/board/video_phy/clut.v \
	$(RTL_DIR)/board/video_phy/upscale.v \
	$(RTL_DIR)/board/video_phy/compositor.v \
	$(RTL_DIR)/board/video_phy/scanout_display.v \
	$(TB_DIR)/tb_scanout_display_latency.v
SCANOUT_DISPLAY_LATENCY_BUILD     := $(BUILD_DIR)/scanout_display_latency
SCANOUT_DISPLAY_LATENCY_NEG_BUILD := $(BUILD_DIR)/scanout_display_latency_neg

.PHONY: tb-scanout-display-latency
tb-scanout-display-latency: $(SCANOUT_DISPLAY_LATENCY_BUILD)/Vtb_scanout_display_latency
	@echo "Running scan-out display five-stage lockstep gate..."
	$(SCANOUT_DISPLAY_LATENCY_BUILD)/Vtb_scanout_display_latency

.PHONY: tb-scanout-display-latency-negctl
tb-scanout-display-latency-negctl: $(SCANOUT_DISPLAY_LATENCY_NEG_BUILD)/Vtb_scanout_display_latency
	@echo "Running lockstep NEGATIVE CONTROL (border pipe tapped one stage early)..."
	@if $(SCANOUT_DISPLAY_LATENCY_NEG_BUILD)/Vtb_scanout_display_latency \
	    > /dev/null 2>&1; then \
	  echo "NEGATIVE CONTROL FAILED: the shortened border pipe was NOT detected."; \
	  exit 1; \
	else \
	  echo "NEGATIVE CONTROL OK: the shortened border pipe was detected."; \
	fi

$(SCANOUT_DISPLAY_LATENCY_BUILD)/Vtb_scanout_display_latency: $(SCANOUT_DISPLAY_LATENCY_RTL) $(TB_DIR)/tb_scanout_display_latency.cpp
	@mkdir -p $(SCANOUT_DISPLAY_LATENCY_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED -Wno-SELRANGE \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR) \
		-Mdir $(SCANOUT_DISPLAY_LATENCY_BUILD) \
		--top-module tb_scanout_display_latency \
		$(SCANOUT_DISPLAY_LATENCY_RTL) \
		$(TB_DIR)/tb_scanout_display_latency.cpp \
		-CFLAGS "-std=c++17"

$(SCANOUT_DISPLAY_LATENCY_NEG_BUILD)/Vtb_scanout_display_latency: $(SCANOUT_DISPLAY_LATENCY_RTL) $(TB_DIR)/tb_scanout_display_latency.cpp
	@mkdir -p $(SCANOUT_DISPLAY_LATENCY_NEG_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED -Wno-SELRANGE \
		-DSCANOUT_INJECT_SHORT_BORDER_PIPE \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR) \
		-Mdir $(SCANOUT_DISPLAY_LATENCY_NEG_BUILD) \
		--top-module tb_scanout_display_latency \
		$(SCANOUT_DISPLAY_LATENCY_RTL) \
		$(TB_DIR)/tb_scanout_display_latency.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# VRAM -> fb_reader -> scaler first-light E2E proof.
#
# Programs DAFB-style base/stride state, writes a checkerboard through the
# real vram AXI slave, then scans it through fb_reader/scaler while pclk and
# vram_clk advance with varied phase.  Complements tb-video's video_top wrapper
# and HDMI-control coverage.
# ──────────────────────────────────────────────────────────────────────────────
VRAM_SCALER_FIRSTLIGHT_RTL := \
	$(RTL_DIR)/mac/video.v \
	$(RTL_DIR)/board/vram.v \
	$(RTL_DIR)/board/video_phy/mode_admit.v \
	$(RTL_DIR)/board/video_phy/place_plan.v \
	$(RTL_DIR)/board/video_phy/scanout_placement_sync.v \
	$(RTL_DIR)/board/video_phy/fb_reader.v \
	$(RTL_DIR)/board/video_phy/scanout_fetch.v \
	$(RTL_DIR)/board/video_phy/pixel_unpack.v \
	$(RTL_DIR)/board/video_phy/clut.v \
	$(RTL_DIR)/board/video_phy/upscale.v \
	$(RTL_DIR)/board/video_phy/compositor.v \
	$(RTL_DIR)/board/video_phy/scanout_display.v \
	$(RTL_DIR)/board/video_phy/linebuf_scanout.v \
	$(TB_DIR)/tb_vram_scaler_firstlight.v
VRAM_SCALER_FIRSTLIGHT_BUILD := $(BUILD_DIR)/vram_scaler_firstlight

# -Wno-UNOPTTHREADS is Verilator's thread PARTITIONER hint, not a design
# check: it only reports that the model could not be split across
# $(VERILATOR_THREADS) threads.  Suppressed for the same reason it already is
# on tb-framebuffer-pixel / tb-mame-vram-scaler-dump / tb-vram-ddr-chain --
# a couple of extra gates in linebuf_scanout must not turn a scheduling hint
# into a build failure.  (It did exactly that here when the frame re-arm fix
# added display_frame_started/frame_active.)
.PHONY: tb-vram-scaler-firstlight
tb-vram-scaler-firstlight: $(VRAM_SCALER_FIRSTLIGHT_BUILD)/Vtb_vram_scaler_firstlight
	@echo "Running VRAM->fb_reader->scaler first-light unit tb..."
	$(VRAM_SCALER_FIRSTLIGHT_BUILD)/Vtb_vram_scaler_firstlight

$(VRAM_SCALER_FIRSTLIGHT_BUILD)/Vtb_vram_scaler_firstlight: $(VRAM_SCALER_FIRSTLIGHT_RTL) $(TB_DIR)/tb_vram_scaler_firstlight.cpp
	@mkdir -p $(VRAM_SCALER_FIRSTLIGHT_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-j $(VERILATOR_JOBS) --threads $(VERILATOR_THREADS) \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED -Wno-SELRANGE \
		-Wno-UNOPTTHREADS \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(VRAM_SCALER_FIRSTLIGHT_BUILD) \
		--top-module tb_vram_scaler_firstlight \
		$(VRAM_SCALER_FIRSTLIGHT_RTL) \
		$(TB_DIR)/tb_vram_scaler_firstlight.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# MAME VRAM shared-memory dump -> fb_reader -> scaler image artifact.
#
# Usage:
#   build/mame_vram_scaler_dump/Vtb_mame_vram_scaler_dump \
#     +vram_image=build/mame_runs/run_vram.bin \
#     +ppm=build/mame_runs/run_scaler.ppm +vram_bpp=1 +vram_stride=1024
# ──────────────────────────────────────────────────────────────────────────────
MAME_VRAM_SCALER_DUMP_RTL := \
	$(RTL_DIR)/board/video_phy/fb_reader.v \
	$(RTL_DIR)/board/video_phy/mode_admit.v \
	$(RTL_DIR)/board/video_phy/scanout_fetch.v \
	$(RTL_DIR)/board/video_phy/place_plan.v \
	$(RTL_DIR)/board/video_phy/pixel_unpack.v \
	$(RTL_DIR)/board/video_phy/clut.v \
	$(RTL_DIR)/board/video_phy/upscale.v \
	$(RTL_DIR)/board/video_phy/compositor.v \
	$(RTL_DIR)/board/video_phy/scanout_display.v \
	$(RTL_DIR)/board/video_phy/linebuf_scanout.v \
	$(TB_DIR)/tb_mame_vram_scaler_dump.v
MAME_VRAM_SCALER_DUMP_BUILD := $(BUILD_DIR)/mame_vram_scaler_dump

.PHONY: mame-vram-scaler-dump
mame-vram-scaler-dump: $(MAME_VRAM_SCALER_DUMP_BUILD)/Vtb_mame_vram_scaler_dump
	@echo "Built MAME VRAM scaler dump utility: $(MAME_VRAM_SCALER_DUMP_BUILD)/Vtb_mame_vram_scaler_dump"

$(MAME_VRAM_SCALER_DUMP_BUILD)/Vtb_mame_vram_scaler_dump: $(MAME_VRAM_SCALER_DUMP_RTL) $(TB_DIR)/tb_mame_vram_scaler_dump.cpp
	@mkdir -p $(MAME_VRAM_SCALER_DUMP_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		$(if $(filter 1,$(LINEBUF_DEBUG)),-DLINEBUF_DEBUG,) \
		--x-assign fast --x-initial fast -O3 \
		-j $(VERILATOR_JOBS) --threads $(VERILATOR_THREADS) \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED -Wno-SELRANGE \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(MAME_VRAM_SCALER_DUMP_BUILD) \
		--top-module tb_mame_vram_scaler_dump \
		$(MAME_VRAM_SCALER_DUMP_RTL) \
		$(TB_DIR)/tb_mame_vram_scaler_dump.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# tb-framebuffer-pixel — pixel-exact CPU→VRAM→fb_reader→linebuf_scanout proof.
#
# The bench models the crossbar's S3 byte swap inline. The old standalone
# vram_cpu_byteswap module was removed in September 2026; do not depend on
# it here. Keep the real DAFB mode decoder and scanout dependencies listed.
# ──────────────────────────────────────────────────────────────────────────────
FRAMEBUFFER_PIXEL_RTL := \
	$(RTL_DIR)/mac/video.v \
	$(RTL_DIR)/mac/mode_decode.v \
	$(RTL_DIR)/board/vram.v \
	$(RTL_DIR)/board/video_phy/mode_admit.v \
	$(RTL_DIR)/board/video_phy/place_plan.v \
	$(RTL_DIR)/board/video_phy/scanout_placement_sync.v \
	$(RTL_DIR)/board/video_phy/fb_reader.v \
	$(RTL_DIR)/board/video_phy/scanout_fetch.v \
	$(RTL_DIR)/board/video_phy/pixel_unpack.v \
	$(RTL_DIR)/board/video_phy/clut.v \
	$(RTL_DIR)/board/video_phy/upscale.v \
	$(RTL_DIR)/board/video_phy/compositor.v \
	$(RTL_DIR)/board/video_phy/scanout_display.v \
	$(RTL_DIR)/board/video_phy/linebuf_scanout.v \
	$(TB_DIR)/tb_framebuffer_pixel.v
FRAMEBUFFER_PIXEL_BUILD := $(BUILD_DIR)/framebuffer_pixel

.PHONY: tb-framebuffer-pixel
tb-framebuffer-pixel: $(FRAMEBUFFER_PIXEL_BUILD)/Vtb_framebuffer_pixel
	@echo "Running CPU->VRAM->fb_reader->linebuf_scanout pixel-exact unit tb..."
	$(FRAMEBUFFER_PIXEL_BUILD)/Vtb_framebuffer_pixel

# -Wno-UNOPTTHREADS below is Verilator's thread PARTITIONER, not a design
# check: it only reports that the model could not be split across
# $(VERILATOR_THREADS) threads.  Suppressed here (tb-l2c-chain and
# tb-vram-ddr-chain already do the same) so that a few extra gates in
# linebuf_scanout cannot turn a scheduling hint into a build failure.
$(FRAMEBUFFER_PIXEL_BUILD)/Vtb_framebuffer_pixel: $(FRAMEBUFFER_PIXEL_RTL) $(TB_DIR)/tb_framebuffer_pixel.cpp
	@mkdir -p $(FRAMEBUFFER_PIXEL_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-j $(VERILATOR_JOBS) --threads $(VERILATOR_THREADS) \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED -Wno-SELRANGE \
		-Wno-UNOPTTHREADS \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(FRAMEBUFFER_PIXEL_BUILD) \
		--top-module tb_framebuffer_pixel \
		$(FRAMEBUFFER_PIXEL_RTL) \
		$(TB_DIR)/tb_framebuffer_pixel.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# HDMI video pipeline unit testbench (rtl/mac/video/*)
#
# Standalone build: mmcm_hdmi / vtg / i2c_init / linebuf_scanout / fb_reader /
# video_top exercised through tb/tb_video_top.v against tb/tb_video_top.cpp.
# No core, no mac_top dependency.  The Verilator branch of mmcm_hdmi
# bypasses the Xilinx primitive instantiation so `clk` drives pclk
# directly.
# ──────────────────────────────────────────────────────────────────────────────
VIDEO_RTL := \
	$(RTL_DIR)/board/video_phy/mmcm_hdmi.v \
	$(RTL_DIR)/board/video_phy/vtg.v \
	$(RTL_DIR)/board/video_phy/i2c_init.v \
	$(RTL_DIR)/board/video_phy/mode_admit.v \
	$(RTL_DIR)/board/video_phy/place_plan.v \
	$(RTL_DIR)/board/video_phy/scanout_placement_sync.v \
	$(RTL_DIR)/board/video_phy/scanout_fetch.v \
	$(RTL_DIR)/board/video_phy/pixel_unpack.v \
	$(RTL_DIR)/board/video_phy/clut.v \
	$(RTL_DIR)/board/video_phy/upscale.v \
	$(RTL_DIR)/board/video_phy/compositor.v \
	$(RTL_DIR)/board/video_phy/scanout_display.v \
	$(RTL_DIR)/board/video_phy/linebuf_scanout.v \
	$(RTL_DIR)/board/video_phy/fb_reader.v \
	$(RTL_DIR)/board/video_phy/video_top.v \
	$(TB_DIR)/tb_video_top.v
VIDEO_BUILD := $(BUILD_DIR)/video

.PHONY: tb-video
tb-video: $(VIDEO_BUILD)/Vtb_video_top
	@echo "Running HDMI video pipeline unit tb..."
	$(VIDEO_BUILD)/Vtb_video_top

$(VIDEO_BUILD)/Vtb_video_top: $(VIDEO_RTL) $(TB_DIR)/tb_video_top.cpp
	@mkdir -p $(VIDEO_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-j $(VERILATOR_JOBS) --threads $(VERILATOR_THREADS) \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(VIDEO_BUILD) \
		--top-module tb_video_top \
		$(VIDEO_RTL) \
		$(TB_DIR)/tb_video_top.cpp \
		-CFLAGS "-std=c++17"

VIDEO_PATTERN_BUILD := $(BUILD_DIR)/video_pattern
VIDEO_CHECKERBOARD_BUILD := $(BUILD_DIR)/video_checkerboard

# ── tb-vbl-rate ─────────────────────────────────────────────────────────
# Task #145: gates the DAFB → pulse_cdc → VIA1 CA1 chain at 60 Hz on the
# pclk side and proves no drops across the CDC.  Uses video_top + the
# pulse_cdc + a VIA1 instance directly — no CPU.
VBL_RATE_RTL := \
	$(RTL_DIR)/board/video_phy/mmcm_hdmi.v \
	$(RTL_DIR)/board/video_phy/vtg.v \
	$(RTL_DIR)/board/video_phy/i2c_init.v \
	$(RTL_DIR)/board/video_phy/mode_admit.v \
	$(RTL_DIR)/board/video_phy/place_plan.v \
	$(RTL_DIR)/board/video_phy/scanout_placement_sync.v \
	$(RTL_DIR)/board/video_phy/scanout_fetch.v \
	$(RTL_DIR)/board/video_phy/pixel_unpack.v \
	$(RTL_DIR)/board/video_phy/clut.v \
	$(RTL_DIR)/board/video_phy/upscale.v \
	$(RTL_DIR)/board/video_phy/compositor.v \
	$(RTL_DIR)/board/video_phy/scanout_display.v \
	$(RTL_DIR)/board/video_phy/linebuf_scanout.v \
	$(RTL_DIR)/board/video_phy/fb_reader.v \
	$(RTL_DIR)/board/video_phy/video_top.v \
	$(RTL_DIR)/board/pulse_cdc.v \
	$(RTL_DIR)/mac/via1.v \
	$(TB_DIR)/tb_vbl_rate.v
VBL_RATE_BUILD := $(BUILD_DIR)/vbl_rate

.PHONY: tb-vbl-rate
tb-vbl-rate: $(VBL_RATE_BUILD)/Vtb_vbl_rate
	@echo "Running DAFB vblank rate gate (video_top -> pulse_cdc)..."
	@echo "  NOTE: this is not the Mac 60 Hz tick — see tb-via-tick-rate."
	$(VBL_RATE_BUILD)/Vtb_vbl_rate

$(VBL_RATE_BUILD)/Vtb_vbl_rate: $(VBL_RATE_RTL) $(TB_DIR)/tb_vbl_rate.cpp
	@mkdir -p $(VBL_RATE_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-j $(VERILATOR_JOBS) --threads $(VERILATOR_THREADS) \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(VBL_RATE_BUILD) \
		--top-module tb_vbl_rate \
		$(VBL_RATE_RTL) \
		$(TB_DIR)/tb_vbl_rate.cpp \
		-CFLAGS "-std=c++17"

.PHONY: tb-video-pattern
tb-video-pattern: $(VIDEO_PATTERN_BUILD)/Vtb_video_top
	@echo "Running HDMI direct test-pattern unit tb..."
	$(VIDEO_PATTERN_BUILD)/Vtb_video_top

$(VIDEO_PATTERN_BUILD)/Vtb_video_top: $(VIDEO_RTL) $(TB_DIR)/tb_video_top.cpp
	@mkdir -p $(VIDEO_PATTERN_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-j $(VERILATOR_JOBS) --threads $(VERILATOR_THREADS) \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(VIDEO_PATTERN_BUILD) \
		--top-module tb_video_top -GTEST_PATTERN=1 \
		$(VIDEO_RTL) \
		$(TB_DIR)/tb_video_top.cpp \
		-CFLAGS "-std=c++17"

.PHONY: tb-video-checkerboard
tb-video-checkerboard: $(VIDEO_CHECKERBOARD_BUILD)/Vtb_video_top
	@echo "Running HDMI direct checkerboard unit tb..."
	$(VIDEO_CHECKERBOARD_BUILD)/Vtb_video_top

$(VIDEO_CHECKERBOARD_BUILD)/Vtb_video_top: $(VIDEO_RTL) $(TB_DIR)/tb_video_top.cpp
	@mkdir -p $(VIDEO_CHECKERBOARD_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-j $(VERILATOR_JOBS) --threads $(VERILATOR_THREADS) \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(VIDEO_CHECKERBOARD_BUILD) \
		--top-module tb_video_top -GTEST_PATTERN=2 \
		$(VIDEO_RTL) \
		$(TB_DIR)/tb_video_top.cpp \
		-CFLAGS "-std=c++17"

.PHONY: firstlight-framebuffer-preflight
firstlight-framebuffer-preflight: tb-video-pattern tb-video-checkerboard tb-vram-scaler-firstlight
	@echo "Framebuffer first-light preflight passed."
	@echo "Hardware recipe: 100 MHz default builds with VIDEO_SMOKE=0 (CPU paints VRAM);"
	@echo "set FPGA_100MHZ_VIDEO_SMOKE=1 to compile in the reset-time SMPTE-bar painter."

# ──────────────────────────────────────────────────────────────────────────────
# DAFB register-shim unit testbench (task #143) — rtl/mac/video.v in
# isolation, AXI4-lite host BFM in tb/tb_dafb.cpp.
# ──────────────────────────────────────────────────────────────────────────────
DAFB_RTL   := $(RTL_DIR)/mac/video.v
DAFB_BUILD := $(BUILD_DIR)/dafb
DAFB_VIA_IRQ_RTL := $(RTL_DIR)/mac/video.v $(RTL_DIR)/mac/via1.v $(RTL_DIR)/mac/irq_agg.v $(TB_DIR)/tb_dafb_via_irq.v
DAFB_VIA_IRQ_BUILD := $(BUILD_DIR)/dafb_via_irq

.PHONY: tb-dafb
tb-dafb: $(DAFB_BUILD)/Vvideo
	@echo "Running DAFB shim unit tb..."
	$(DAFB_BUILD)/Vvideo

$(DAFB_BUILD)/Vvideo: $(DAFB_RTL) $(TB_DIR)/tb_dafb.cpp
	@mkdir -p $(DAFB_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(DAFB_BUILD) \
		--top-module video \
		$(DAFB_RTL) \
		$(TB_DIR)/tb_dafb.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# DAFB slot-IRQ unit testbench — rtl/mac/video.v's `irq` OUTPUT PIN in
# isolation.  Distinct from tb-dafb: tb-dafb exercises the +0x108 status
# mirror extensively but never asserts on dut->irq in its cursor scenario,
# so it could not have caught the 0bc53b3 regression (irq driven only
# from vblank_pending, ignoring swatch_cursor_pending entirely -- the
# exact source + ack path real Mac OS uses and the real cause of a boot
# hang on hardware).  This tb asserts on `irq` itself for both sources,
# their independent acks (+0x10C cursor / +0x114 VBL), enable-gating, and
# cross-ack independence.
# ──────────────────────────────────────────────────────────────────────────────
VIDEO_IRQ_RTL   := $(RTL_DIR)/mac/video.v
VIDEO_IRQ_BUILD := $(BUILD_DIR)/video_irq

.PHONY: tb-video-irq
tb-video-irq: $(VIDEO_IRQ_BUILD)/Vvideo
	@echo "Running DAFB slot-IRQ unit tb..."
	$(VIDEO_IRQ_BUILD)/Vvideo

$(VIDEO_IRQ_BUILD)/Vvideo: $(VIDEO_IRQ_RTL) $(TB_DIR)/tb_video_irq.cpp
	@mkdir -p $(VIDEO_IRQ_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(VIDEO_IRQ_BUILD) \
		--top-module video \
		$(VIDEO_IRQ_RTL) \
		$(TB_DIR)/tb_video_irq.cpp \
		-CFLAGS "-std=c++17"

.PHONY: tb-dafb-via-irq
tb-dafb-via-irq: $(DAFB_VIA_IRQ_BUILD)/Vtb_dafb_via_irq
	@echo "Running DAFB VIA IRQ route tb..."
	$(DAFB_VIA_IRQ_BUILD)/Vtb_dafb_via_irq

$(DAFB_VIA_IRQ_BUILD)/Vtb_dafb_via_irq: $(DAFB_VIA_IRQ_RTL) $(TB_DIR)/tb_dafb_via_irq.cpp
	@mkdir -p $(DAFB_VIA_IRQ_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(DAFB_VIA_IRQ_BUILD) \
		--top-module tb_dafb_via_irq \
		$(DAFB_VIA_IRQ_RTL) \
		$(TB_DIR)/tb_dafb_via_irq.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# Extended-monitor sense unit tb — instantiates video.v with
# MONITOR_TYPE = ext(2,3,1) = 0x5D (16" RGB) so the m_monitor_id ×
# MONITOR_TYPE convolution in the +0x1C read path is exercised.
# ──────────────────────────────────────────────────────────────────────────────
DAFB_EXTMON_RTL := $(RTL_DIR)/mac/video.v $(TB_DIR)/tb_dafb_extmon.v
DAFB_EXTMON_BUILD := $(BUILD_DIR)/dafb_extmon

.PHONY: tb-dafb-extmon
tb-dafb-extmon: $(DAFB_EXTMON_BUILD)/Vtb_dafb_extmon
	@echo "Running DAFB extended-monitor sense unit tb..."
	$(DAFB_EXTMON_BUILD)/Vtb_dafb_extmon

$(DAFB_EXTMON_BUILD)/Vtb_dafb_extmon: $(DAFB_EXTMON_RTL) $(TB_DIR)/tb_dafb_extmon.cpp
	@mkdir -p $(DAFB_EXTMON_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(DAFB_EXTMON_BUILD) \
		--top-module tb_dafb_extmon \
		$(DAFB_EXTMON_RTL) \
		$(TB_DIR)/tb_dafb_extmon.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# DAFB mode-decode contract: the Quadra 700 monitor-sense table, in lockstep
# against MAME 0.285 dafb_base.  Every DAFB register write a real macqd700 ROM
# boot performs, captured once per monitor-sense code (tb_mode_decode_traces.h)
# and replayed into the real video.v + mode_decode over AXI.
#
# Runs its own negative control: MD_SELFTEST perturbs the MODEL, and a green
# run under any perturbation means the table is measuring nothing.
# ──────────────────────────────────────────────────────────────────────────────
MODE_DECODE_RTL := $(RTL_DIR)/mac/video.v $(RTL_DIR)/mac/mode_decode.v \
                   $(TB_DIR)/tb_mode_decode.v
MODE_DECODE_BUILD := $(BUILD_DIR)/mode_decode

.PHONY: tb-mode-decode
tb-mode-decode: $(MODE_DECODE_BUILD)/Vtb_mode_decode
	@echo "Running DAFB mode-decode monitor-sense table (vs MAME)..."
	$(MODE_DECODE_BUILD)/Vtb_mode_decode
	@echo "-- negative controls: each MUST report failures --"
	@for n in 1 2 3 4 5 6 7; do \
		MD_SELFTEST=$$n $(MODE_DECODE_BUILD)/Vtb_mode_decode >/dev/null 2>&1 \
			|| { echo "MD_SELFTEST=$$n did not survive its own control"; exit 1; }; \
	done
	@echo "-- all 7 negative controls fired --"

$(MODE_DECODE_BUILD)/Vtb_mode_decode: $(MODE_DECODE_RTL) \
                                      $(TB_DIR)/tb_mode_decode.cpp \
                                      $(TB_DIR)/tb_mode_decode_traces.h
	@mkdir -p $(MODE_DECODE_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(MODE_DECODE_BUILD) \
		--top-module tb_mode_decode \
		$(MODE_DECODE_RTL) \
		$(TB_DIR)/tb_mode_decode.cpp \
		-CFLAGS "-std=c++17 -I$(TB_DIR)"

# ──────────────────────────────────────────────────────────────────────────────
# Live DAFB state -> scaler proof.
# ──────────────────────────────────────────────────────────────────────────────
DAFB_SCANOUT_RTL := \
	$(RTL_DIR)/mac/video.v \
	$(RTL_DIR)/board/video_phy/fb_reader.v \
	$(RTL_DIR)/board/video_phy/mode_admit.v \
	$(RTL_DIR)/board/video_phy/scanout_fetch.v \
	$(RTL_DIR)/board/video_phy/place_plan.v \
	$(RTL_DIR)/board/video_phy/pixel_unpack.v \
	$(RTL_DIR)/board/video_phy/clut.v \
	$(RTL_DIR)/board/video_phy/upscale.v \
	$(RTL_DIR)/board/video_phy/compositor.v \
	$(RTL_DIR)/board/video_phy/scanout_display.v \
	$(RTL_DIR)/board/video_phy/linebuf_scanout.v \
	$(TB_DIR)/tb_dafb_scanout.v
DAFB_SCANOUT_BUILD := $(BUILD_DIR)/dafb_scanout

.PHONY: tb-dafb-scanout
tb-dafb-scanout: $(DAFB_SCANOUT_BUILD)/Vtb_dafb_scanout
	@echo "Running DAFB scanout wiring unit tb..."
	$(DAFB_SCANOUT_BUILD)/Vtb_dafb_scanout

$(DAFB_SCANOUT_BUILD)/Vtb_dafb_scanout: $(DAFB_SCANOUT_RTL) $(TB_DIR)/tb_dafb_scanout.cpp
	@mkdir -p $(DAFB_SCANOUT_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED -Wno-SELRANGE \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(DAFB_SCANOUT_BUILD) \
		--top-module tb_dafb_scanout \
		$(DAFB_SCANOUT_RTL) \
		$(TB_DIR)/tb_dafb_scanout.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# tb-scanout-frames — MULTI-FRAME scanout gate at shipping geometry.
#
# The 2026-07-31 hardware bug (exactly LINE_COUNT=64 source rows at the top of
# the screen, black below, oscillating) escaped every other video tb for four
# structural reasons, each of which this target exists to remove:
#   1. it only shows up ACROSS frames (a failure to re-arm the fetch walk),
#      and every other scanout tb asserts inside one frame walk;
#   2. it happens exactly at the 64-entry line-buffer ring wrap, so a source
#      height <= LINE_COUNT never reaches it (tb-framebuffer-pixel is 64x64);
#   3. with identical source rows a parked fetcher and a working one render
#      the SAME image, so content has to vary per row;
#   4. it needs the real 1920x1080 VTG, whose ~45-line vertical blanking is
#      what lets the fetch walk run a full ring ahead of the display.
# See tb/tb_scanout_frames.v's header for the full rationale.
# ──────────────────────────────────────────────────────────────────────────────
SCANOUT_FRAMES_RTL := \
	$(RTL_DIR)/board/video_phy/vtg.v \
	$(RTL_DIR)/board/video_phy/mode_admit.v \
	$(RTL_DIR)/board/video_phy/scanout_fetch.v \
	$(RTL_DIR)/board/video_phy/place_plan.v \
	$(RTL_DIR)/board/video_phy/pixel_unpack.v \
	$(RTL_DIR)/board/video_phy/clut.v \
	$(RTL_DIR)/board/video_phy/upscale.v \
	$(RTL_DIR)/board/video_phy/compositor.v \
	$(RTL_DIR)/board/video_phy/scanout_display.v \
	$(RTL_DIR)/board/video_phy/linebuf_scanout.v \
	$(TB_DIR)/tb_scanout_frames.v
SCANOUT_FRAMES_BUILD := $(BUILD_DIR)/scanout_frames

.PHONY: tb-scanout-frames
tb-scanout-frames: $(SCANOUT_FRAMES_BUILD)/Vtb_scanout_frames
	@echo "Running multi-frame shipping-geometry scanout unit tb..."
	$(SCANOUT_FRAMES_BUILD)/Vtb_scanout_frames

# NEGATIVE CONTROL for the frame-to-frame identity assertion.  Same binary,
# with SCANOUT_FRAMES_MUTATE=1, which flips one bit of one captured display
# line.  The identity check MUST notice; the recipe passes only when the run
# fails.  Without this, "every frame matched every other frame" would be
# equally consistent with a checker that compares nothing.
.PHONY: tb-scanout-frames-negctl
tb-scanout-frames-negctl: $(SCANOUT_FRAMES_BUILD)/Vtb_scanout_frames
	@echo "Running multi-frame scanout NEGATIVE CONTROL (mutated frame)..."
	@if SCANOUT_FRAMES_MUTATE=1 $(SCANOUT_FRAMES_BUILD)/Vtb_scanout_frames \
	    > /dev/null 2>&1; then \
	  echo "NEGATIVE CONTROL FAILED: a mutated frame was reported identical"; \
	  exit 1; \
	else \
	  echo "NEGATIVE CONTROL OK: the mutated frame was detected."; \
	fi

$(SCANOUT_FRAMES_BUILD)/Vtb_scanout_frames: $(SCANOUT_FRAMES_RTL) $(TB_DIR)/tb_scanout_frames.cpp
	@mkdir -p $(SCANOUT_FRAMES_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED -Wno-SELRANGE \
		-Wno-UNOPTTHREADS \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR) \
		-Mdir $(SCANOUT_FRAMES_BUILD) \
		--top-module tb_scanout_frames \
		$(SCANOUT_FRAMES_RTL) \
		$(TB_DIR)/tb_scanout_frames.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# tb-scanout-1bpp — PIXEL-EXACT 1bpp gate, run at EVERY destination mode.
#
# tb-scanout-frames has a 1bpp scenario, but it paints whole source rows as
# 0x00 or 0xFF, so it cannot see the horizontal half of the 1bpp path at all
# (the sub-byte slice order, the `x_src >> bpp_shift` byte index, the x_lo
# pipeline), and it is pinned to DST 1920x1080 where 640x480 scales at N=2.
# The shipping raster moved to 1280x720 in 788cd2b0, where that source scales
# at N=1 for the FIRST time -- a case no video tb had ever run.
#
# This target runs the same scanner with a source that varies per byte AND per
# bit, and compares every destination pixel of whole frames.  The geometry is a
# -G override read back out of the DUT, so the SHIPPING mode is the default and
# the mode it replaced is kept as a second target: a future mode change adds a
# target here instead of silently going uncovered.
# ──────────────────────────────────────────────────────────────────────────────
SCANOUT_1BPP_RTL := \
	$(RTL_DIR)/board/video_phy/vtg.v \
	$(RTL_DIR)/board/video_phy/mode_admit.v \
	$(RTL_DIR)/board/video_phy/scanout_fetch.v \
	$(RTL_DIR)/board/video_phy/place_plan.v \
	$(RTL_DIR)/board/video_phy/pixel_unpack.v \
	$(RTL_DIR)/board/video_phy/clut.v \
	$(RTL_DIR)/board/video_phy/upscale.v \
	$(RTL_DIR)/board/video_phy/compositor.v \
	$(RTL_DIR)/board/video_phy/scanout_display.v \
	$(RTL_DIR)/board/video_phy/linebuf_scanout.v \
	$(TB_DIR)/tb_scanout_1bpp.v
SCANOUT_1BPP_BUILD      := $(BUILD_DIR)/scanout_1bpp_720p
SCANOUT_1BPP_1080_BUILD := $(BUILD_DIR)/scanout_1bpp_1080p
SCANOUT_1BPP_NEG_BUILD  := $(BUILD_DIR)/scanout_1bpp_negctl

# tb_scanout_1bpp.v's defaults ARE the shipping mode (720p60), so this arm
# needs no -G at all.
.PHONY: tb-scanout-1bpp
tb-scanout-1bpp: $(SCANOUT_1BPP_BUILD)/Vtb_scanout_1bpp
	@echo "Running pixel-exact 1bpp scanout gate at the SHIPPING mode (720p60)..."
	$(SCANOUT_1BPP_BUILD)/Vtb_scanout_1bpp

# The mode 788cd2b0 replaced, kept so a regression that is specific to the NEW
# raster is distinguishable from one that was always there.
.PHONY: tb-scanout-1bpp-1080p
tb-scanout-1bpp-1080p: $(SCANOUT_1BPP_1080_BUILD)/Vtb_scanout_1bpp
	@echo "Running pixel-exact 1bpp scanout gate at 1080p (the pre-788cd2b0 mode)..."
	$(SCANOUT_1BPP_1080_BUILD)/Vtb_scanout_1bpp

define SCANOUT_1BPP_BUILD_RULE
	@mkdir -p $(1)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED -Wno-SELRANGE \
		-Wno-UNOPTTHREADS \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR) \
		-Mdir $(1) \
		--top-module tb_scanout_1bpp $(2) \
		$(SCANOUT_1BPP_RTL) \
		$(TB_DIR)/tb_scanout_1bpp.cpp \
		-CFLAGS "-std=c++17"
endef

$(SCANOUT_1BPP_BUILD)/Vtb_scanout_1bpp: $(SCANOUT_1BPP_RTL) $(TB_DIR)/tb_scanout_1bpp.cpp
	$(call SCANOUT_1BPP_BUILD_RULE,$(SCANOUT_1BPP_BUILD),)

$(SCANOUT_1BPP_1080_BUILD)/Vtb_scanout_1bpp: $(SCANOUT_1BPP_RTL) $(TB_DIR)/tb_scanout_1bpp.cpp
	$(call SCANOUT_1BPP_BUILD_RULE,$(SCANOUT_1BPP_1080_BUILD),\
		-GDST_W=1920 -GDST_H=1080 -GH_FP=88 -GH_SYNC=44 -GH_BP=148 \
		-GV_FP=4 -GV_SYNC=5 -GV_BP=36)

# NEGATIVE CONTROL.  Mirrors the 1bpp sub-byte slice order inside
# pixel_unpack.v (see its PIXEL_UNPACK_INJECT_1BPP_MIRROR block) and REQUIRES
# the gate above to go red.  This is what distinguishes "the 1bpp horizontal
# path is correct" from "the source pattern is constant along a row" -- which
# is precisely why tb-scanout-frames' 1bpp scenarios could not have caught a
# horizontal defect.
.PHONY: tb-scanout-1bpp-negctl
tb-scanout-1bpp-negctl: $(SCANOUT_1BPP_NEG_BUILD)/Vtb_scanout_1bpp
	@echo "Running pixel-exact 1bpp NEGATIVE CONTROL (sub-byte slice mirrored)..."
	@if $(SCANOUT_1BPP_NEG_BUILD)/Vtb_scanout_1bpp > /dev/null 2>&1; then \
	  echo "NEGATIVE CONTROL FAILED: a mirrored 1bpp slice order was reported pixel-exact"; \
	  exit 1; \
	else \
	  echo "NEGATIVE CONTROL OK: the mirrored slice order was detected."; \
	fi

$(SCANOUT_1BPP_NEG_BUILD)/Vtb_scanout_1bpp: $(SCANOUT_1BPP_RTL) $(TB_DIR)/tb_scanout_1bpp.cpp
	$(call SCANOUT_1BPP_BUILD_RULE,$(SCANOUT_1BPP_NEG_BUILD),\
		-DPIXEL_UNPACK_INJECT_1BPP_MIRROR)

# ──────────────────────────────────────────────────────────────────────────────
# DAFB depth-switch scanout at PRODUCTION geometry (24bpp black-screen gate).
#
# tb-framebuffer-pixel elaborates the same modules at SRC_W=SRC_H=64 against a
# 2 MiB FB_MAX_PIXELS, so its placement-range terms are ~1000x below the
# aperture and the "does the frame fit in VRAM" gate is structurally
# unreachable there at any depth.  This target instantiates the SHIPPING
# geometry (1024x768 / 2 MiB / 1920x1080) and drives the real DAFB shim with
# the register values read off the live board at 8bpp and 24bpp.
# ──────────────────────────────────────────────────────────────────────────────
DAFB_24BPP_CAP_RTL := \
	$(RTL_DIR)/mac/video.v \
	$(RTL_DIR)/board/video_phy/mode_admit.v \
	$(RTL_DIR)/board/video_phy/place_plan.v \
	$(RTL_DIR)/board/video_phy/scanout_placement_sync.v \
	$(RTL_DIR)/board/video_phy/scanout_fetch.v \
	$(RTL_DIR)/board/video_phy/pixel_unpack.v \
	$(RTL_DIR)/board/video_phy/clut.v \
	$(RTL_DIR)/board/video_phy/upscale.v \
	$(RTL_DIR)/board/video_phy/compositor.v \
	$(RTL_DIR)/board/video_phy/scanout_display.v \
	$(RTL_DIR)/board/video_phy/linebuf_scanout.v \
	$(RTL_DIR)/board/video_phy/fb_reader.v \
	$(TB_DIR)/tb_dafb_24bpp_capacity.v
DAFB_24BPP_CAP_BUILD := $(BUILD_DIR)/dafb_24bpp_capacity

.PHONY: tb-dafb-24bpp-capacity
tb-dafb-24bpp-capacity: $(DAFB_24BPP_CAP_BUILD)/Vtb_dafb_24bpp_capacity
	@echo "Running production-geometry DAFB 8/24bpp scanout unit tb..."
	$(DAFB_24BPP_CAP_BUILD)/Vtb_dafb_24bpp_capacity

$(DAFB_24BPP_CAP_BUILD)/Vtb_dafb_24bpp_capacity: $(DAFB_24BPP_CAP_RTL) $(TB_DIR)/tb_dafb_24bpp_capacity.cpp
	@mkdir -p $(DAFB_24BPP_CAP_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED -Wno-SELRANGE \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(DAFB_24BPP_CAP_BUILD) \
		--top-module tb_dafb_24bpp_capacity \
		$(DAFB_24BPP_CAP_RTL) \
		$(TB_DIR)/tb_dafb_24bpp_capacity.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# WHOLE-MATRIX DAFB mode x depth scanout gate.
#
# Every other video tb pins ONE geometry (usually 640x480 -- the mode the board
# happened to be in) and one or two depths.  That is exactly the coverage shape
# that lets "640x480 works" stand in for "the runtime geometry plumbing works":
# a mode that works because it equals a default is indistinguishable from one
# that works because the plumbing works.  This target walks the whole set of
# monitor modes MAME's macqd700 offers (register values captured from a real
# ROM boot per monitor code) crossed with every AC842 depth, and checks each
# combo pixel-exact.  It also carries a synthetic 800x600 geometry that is NOT
# a real Q700 mode, as a positive control that the path is runtime-driven.
# ──────────────────────────────────────────────────────────────────────────────
DAFB_MODE_MATRIX_RTL := \
	$(RTL_DIR)/mac/video.v \
	$(RTL_DIR)/board/video_phy/mode_admit.v \
	$(RTL_DIR)/board/video_phy/place_plan.v \
	$(RTL_DIR)/board/video_phy/scanout_placement_sync.v \
	$(RTL_DIR)/board/video_phy/scanout_fetch.v \
	$(RTL_DIR)/board/video_phy/pixel_unpack.v \
	$(RTL_DIR)/board/video_phy/clut.v \
	$(RTL_DIR)/board/video_phy/upscale.v \
	$(RTL_DIR)/board/video_phy/compositor.v \
	$(RTL_DIR)/board/video_phy/scanout_display.v \
	$(RTL_DIR)/board/video_phy/linebuf_scanout.v \
	$(TB_DIR)/tb_dafb_mode_matrix.v
DAFB_MODE_MATRIX_BUILD := $(BUILD_DIR)/dafb_mode_matrix

# MM_SRC_W / MM_SRC_H elaborate the scanner at a chosen source bound so the
# same matrix can be run against the shipping geometry and against any
# candidate widening of it.  Defaults track rtl/soc/fpga_top_video.vh.
MM_SRC_W ?= 1152
MM_SRC_H ?= 1024
# Aperture + read-port address width.  Defaults are the 2 MiB / 21-bit URAM
# build; the SHIPPING bitstream is VRAM_IN_DDR = 4 MiB / 22-bit (see
# rtl/soc/fpga_top_video.vh).  tb-dafb-mode-matrix-ddr-aperture below walks
# the identical matrix at the shipping numbers -- both are real
# configurations and the admission gate compares against them directly.
# DECIMAL on purpose: this value is substituted into BOTH the Verilog
# parameter and (via verilator's -D forwarding) the C++ model, and 0x...
# is not Verilog.  2097152 = 2 MiB, 4194304 = 4 MiB.
MM_FB_MAX_PIXELS ?= 2097152
MM_ADDR_W        ?= 21

.PHONY: tb-dafb-mode-matrix
tb-dafb-mode-matrix: $(DAFB_MODE_MATRIX_BUILD)/Vtb_dafb_mode_matrix
	@echo "Running whole-matrix DAFB mode x depth scanout tb (SRC $(MM_SRC_W)x$(MM_SRC_H))..."
	$(DAFB_MODE_MATRIX_BUILD)/Vtb_dafb_mode_matrix

# The SHIPPING VRAM_IN_DDR configuration: 4 MiB aperture, 22-bit read-port
# address.  Distinct from the default target, which uses the 2 MiB / 21-bit
# URAM numbers -- the aperture is one side of the placement admission
# comparison, so a mode can be admitted under one and rejected under the
# other.
.PHONY: tb-dafb-mode-matrix-ddr-aperture
tb-dafb-mode-matrix-ddr-aperture:
	$(MAKE) tb-dafb-mode-matrix MM_FB_MAX_PIXELS=4194304 MM_ADDR_W=22 \
		DAFB_MODE_MATRIX_BUILD=$(BUILD_DIR)/dafb_mode_matrix_ddr

# The PRE-widening bound, kept as a live target: it is the configuration in
# which 1152x870 renders a 1024x768 crop, and having both means the widening
# is demonstrated rather than asserted.
.PHONY: tb-dafb-mode-matrix-legacy-src
tb-dafb-mode-matrix-legacy-src:
	$(MAKE) tb-dafb-mode-matrix MM_SRC_W=1024 MM_SRC_H=768 \
		DAFB_MODE_MATRIX_BUILD=$(BUILD_DIR)/dafb_mode_matrix_1024

# NEGATIVE CONTROLS.  Both must FAIL for the recipe to pass.
#   -break : one mode is programmed with a deliberately wrong HFP, so the DUT
#            renders a different width than the model expects.  Proves the
#            geometry is genuinely measured, not assumed.
#   -mutate: one captured pixel is flipped.  Proves the pixel comparison is
#            actually reading the capture.
.PHONY: tb-dafb-mode-matrix-negctl
tb-dafb-mode-matrix-negctl: $(DAFB_MODE_MATRIX_BUILD)/Vtb_dafb_mode_matrix
	@echo "Negative control 1/2: deliberately broken geometry for 832x624-16in..."
	@if MATRIX_BREAK=832x624-16in MATRIX_MODE=832x624-16in MATRIX_DEPTH=8bpp \
	    $(DAFB_MODE_MATRIX_BUILD)/Vtb_dafb_mode_matrix > /dev/null 2>&1; then \
	  echo "NEGATIVE CONTROL FAILED: a broken geometry was reported green"; exit 1; \
	else echo "NEGATIVE CONTROL OK: the broken geometry was detected."; fi
	@echo "Negative control 2/2: mutated captured pixel..."
	@if MATRIX_MUTATE=1 MATRIX_MODE=640x480-hires MATRIX_DEPTH=8bpp \
	    $(DAFB_MODE_MATRIX_BUILD)/Vtb_dafb_mode_matrix > /dev/null 2>&1; then \
	  echo "NEGATIVE CONTROL FAILED: a mutated pixel was reported identical"; exit 1; \
	else echo "NEGATIVE CONTROL OK: the mutated pixel was detected."; fi

$(DAFB_MODE_MATRIX_BUILD)/Vtb_dafb_mode_matrix: $(DAFB_MODE_MATRIX_RTL) $(TB_DIR)/tb_dafb_mode_matrix.cpp
	@mkdir -p $(DAFB_MODE_MATRIX_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED -Wno-SELRANGE \
		+define+MM_SRC_W=$(MM_SRC_W) +define+MM_SRC_H=$(MM_SRC_H) \
		+define+MM_FB_MAX_PIXELS=$(MM_FB_MAX_PIXELS) \
		+define+MM_ADDR_W=$(MM_ADDR_W) \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(DAFB_MODE_MATRIX_BUILD) \
		--top-module tb_dafb_mode_matrix \
		$(DAFB_MODE_MATRIX_RTL) \
		$(TB_DIR)/tb_dafb_mode_matrix.cpp \
		-CFLAGS "-std=c++17 -DMM_SRC_W=$(MM_SRC_W) -DMM_SRC_H=$(MM_SRC_H) -DMM_FB_MAX_PIXELS=$(MM_FB_MAX_PIXELS)u -DMM_ADDR_W=$(MM_ADDR_W)"

# ──────────────────────────────────────────────────────────────────────────────
# Decode cross-check against Musashi ISS — REMOVED
# ──────────────────────────────────────────────────────────────────────────────
# The historical `decode-check` target invoked tools/decode_check.py, which
# was never landed in this repo (CLAUDE.md mentioned it as a future utility,
# but Musashi cross-checking is now done via `make fuzz` which co-sims the
# RTL pipeline against Musashi end-to-end).  Removed 2026-05-07 to fix the
# dead target.  See `make fuzz` for the live golden-ref check.

# ──────────────────────────────────────────────────────────────────────────────
# Lint
# ──────────────────────────────────────────────────────────────────────────────
# Lint — Verilator --lint-only.
#
# NB: --cc is REQUIRED alongside --lint-only; pp-only mode (without --cc)
# doesn't fully elaborate hierarchies and bypasses most -Wno- filters,
# surfacing ~140 spurious warnings that a full build correctly suppresses.
#
# Whole-design lint: `make lint` — delegates to `lint-fpga-top`, the real
# SoC top (this repo has no monorepo rtl/core/* tree to lint a flat
# mac_top from any more — see the CPU RTL note near the top of this file).
# Per-module lint:   `make lint MODULE=alu` — finds any single rtl/**/*.v
# by basename and lints it standalone (works for any module still local
# to this repo; core-track modules live in cpu/ — `cd cpu && make lint
# MODULE=alu`).
.PHONY: lint
LINT_FLAGS := --lint-only --cc \
	--unroll-count 128 \
	-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR)/mac -I$(RTL_DIR) \
	-Wall \
	-Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
	-Wno-WIDTHTRUNC -Wno-WIDTHEXPAND -Wno-DECLFILENAME \
	-Wno-CASEINCOMPLETE -Wno-CMPCONST -Wno-UNOPTFLAT \
	-Wno-SELRANGE -Wno-LITENDIAN
lint:
ifdef MODULE
ifeq ($(MODULE),sd_jtag_writer)
	$(VERILATOR) $(LINT_FLAGS) \
		--top-module sd_jtag_writer \
		$(RTL_DIR)/board/sd_ctrl.v \
		$(RTL_DIR)/board/sd_jtag_writer.v
else
	$(VERILATOR) $(LINT_FLAGS) \
		$$(find $(RTL_DIR) \( -name "$(MODULE).v" -o -name "$(MODULE).sv" \))
endif
else
	@echo "── whole-design lint: every define combination a bitstream can ship ──"
	$(MAKE) lint-configs
	$(MAKE) lint-realmig
	$(MAKE) lint-eth-link
endif

# 2026-08-19: bare `make lint` used to run ONLY `lint-fpga-top`, which
# passes just -DSIM_MODEL.  Every shipping bitstream sets L2C_ENABLE=1,
# VRAM_IN_DDR=1 and ENABLE_VIO=1, so the default lint covered a
# configuration NO board build has ever used — the entire
# `ifdef VIO_ENABLE` block in fpga_top_debug_vio.vh was skipped, and
# `vio_hard_reset` (fpga_top_clocks.vh) looked undriven on the platform
# reset path because its only driver lives inside that block.
# `lint-configs` already existed to cover all eight real combinations
# and NOTHING invoked it.  It now IS the default; `make lint-fpga-top`
# still runs the single SIM_MODEL-only pass on its own if you want it.

# ──────────────────────────────────────────────────────────────────────────────
# CPU build select — CPU=stub (default) | CPU=m68k (submodule) | CPU=m68k040 (submodule)
# ──────────────────────────────────────────────────────────────────────────────
# CPU=stub    : fpga_top's CPU socket binds rtl/soc/cpu_stub.v (idle).  This is
#             the standalone SoC build; nothing under cpu/ or cpu040/ is
#             referenced.
# CPU=m68k  : the socket instead binds cpu/rtl/core/m68k_axi_wrapper.v (the real
#             m68k_core + if_to_axi + axi_narrow_to_wide + debug_ctrl glue),
#             selected at elaboration via +define+CPU_M68K (see
#             rtl/soc/fpga_top_debug_ctrl.vh).  The wrapper's RTL tree
#             (cpu/rtl/core/**) is appended to the fpga_top source list and the
#             cpu/ include dirs are added.  Shared modules that ALSO live in the
#             SoC tree (if_to_axi, axi_narrow_to_wide, irq_agg via cpu/rtl/sys +
#             cpu/rtl/mac) are supplied by the SoC glob;
#             the colliding cpu/ copies are deliberately NOT added to avoid
#             double-definition.  CPU=stub is unaffected.
# CPU=m68k040 : task #270 / SOC-3.  The socket instead binds cpu040/generated/
#             M68kSocketTop.v -- the m68k-core-040-ooo (SpinalHDL, "v2") CPU,
#             selected at elaboration via +define+CPU_M68K040 (see
#             rtl/soc/fpga_top_debug_ctrl.vh).  Unlike CPU=m68k, M68kSocketTop
#             IS the wrapper (no separate if_to_axi/axi_narrow_to_wide glue
#             needed -- its generated port list is an exact match for
#             cpu_socket.vh already): CPU_M68K040_SRCS is just that ONE
#             generated Verilog file, not a source tree glob, and it is
#             regenerated on demand from cpu040's SpinalHDL sources (see the
#             $(CPU_M68K040_V) rule below) rather than checked in as the
#             thing the build reads. axi_i is native 256b for this CPU with
#             no CPU-side downconverter, so CPU=m68k040 unconditionally folds
#             `+define+L2C_ENABLE +define+VRAM_IN_DDR` into CPU_DEFINE below
#             (task #269's dedicated 256b l2c fetch port is the ONLY axi_i
#             binding this CPU can use safely -- see the `error` backstop
#             at rtl/soc/fpga_top_cpu.vh's CPU_AXI_I_DW guard). Synth/impl
#             (synth/vivado.tcl) wiring for CPU=m68k040 is NOT done as part
#             of SOC-3 -- this knob is Verilator lint/sim only for now.
#
# DEFAULT = m68k040.  It used to be `stub`, and that default cost real time more
# than once: `make impl` with no arguments produced a bitstream with NO CPU, and
# the failure is silent -- you get a board that never executes anything.
# tools/build_bitstream.sh already REFUSES CPU=stub for exactly this reason.
# Stub remains available explicitly (CPU=stub) for standalone SoC lint builds.
CPU ?= m68k040

# No exclusions.  There USED to be a "module-name collision guard" here
# dropping debug_stop_manager.v because rtl/soc/fpga_top.v defined its own
# inline copy.  That inline copy was deleted 2026-08-19 (see the tombstone
# comment at the top of rtl/soc/fpga_top.v): cpu/ now holds the single
# definition, and skipping it here would leave m68k_axi_wrapper's
# `u_debug_stop` instance undefined.  synth/vivado.tcl mirrors this glob
# and had the same guard removed.

# ── CPU=m68k040 (task #270 / SOC-3): cpu040/, SpinalHDL-generated ─────
# Unlike CPU_M68K_SRCS (a checked-in RTL tree read as-is), M68kSocketTop.v
# is a GENERATED artifact -- one flat, self-contained Verilog file with no
# `include`s and no module parameters, produced by cpu040's own SpinalHDL
# elaboration.  $(CPU_M68K040_V) below regenerates it on demand (only when
# cpu040's Scala sources are newer than the last generated .v, same
# staleness contract as any other Make file target) via the exact command
# task #270 confirmed works: `cd cpu040 && sbt "runMain
# m68k040.top.GenSocketTopVerilog"` (~20s cold, faster warm).  No
# CPU_EXTRA_IDIRS needed -- nothing else `include`s into it.
CPU040_SRC_DIR   ?= $(RTL_DIR)/../cpu040
CPU_M68K040_V    := $(CPU040_SRC_DIR)/generated/M68kSocketTop.v
# Spinal emits $readmemb paths relative to its generated-Verilog directory.
# Run m68k040 simulations there so those memories are not silently zeroed.
CPU_MODEL_RUN_PREFIX := $(if $(filter m68k040,$(CPU)),cd $(CPU040_SRC_DIR)/generated && ,)
CPU040_SCALA_SRCS := $(shell find $(CPU040_SRC_DIR)/src $(CPU040_SRC_DIR)/build.sbt \
                        -type f 2>/dev/null | sort)
CPU_M68K040_SRCS := $(CPU_M68K040_V)
CPU_M68K040_IDIRS :=

.PHONY: cpu040-gen
cpu040-gen:
	@echo "cpu040-gen: regenerating M68kSocketTop.v via sbt (cd $(CPU040_SRC_DIR) && sbt \"runMain m68k040.top.GenSocketTopVerilog\") ..."
	cd $(CPU040_SRC_DIR) && sbt "runMain m68k040.top.GenSocketTopVerilog"

# Only re-run sbt when a cpu040 Scala source (or build.sbt) is newer than
# the last generated .v -- keeps `make lint-fpga-top CPU=m68k040` cheap on
# repeat invocations instead of eating ~20s of sbt startup every time.
$(CPU_M68K040_V): $(CPU040_SCALA_SRCS)
	$(MAKE) cpu040-gen

# ── CPU physical-register-file size knob ─────────────────────────────
# `make lint-configs CPU=m68k PRF_INT=64` (and the fpga_top sim builds)
# pick the same defines the Vivado flow forwards via VIVADO_RUN_ENV /
# synth/vivado.tcl, so a size is lint-checked here before it is synthed.
# The tag width is derived (Verilog-2005's preprocessor cannot compute
# ceil(log2 N)); cpu/rtl/core/rename/{rat,fp_rat}.v carry a hard
# elaboration check that fails the build on a mismatched pair.
#
# REQUIRES a cpu submodule at or after the CPU commit that introduced the
# knob ("core: make the physical register file size a build-time knob").
# Against an older cpu/, uop_pkg.v `define's PHYS_INT_REGS unconditionally
# so the +define+ here loses and the build silently stays at 96 -- still
# functionally correct, but you get none of the area back.  Confirm with
# `grep -n 'ifndef PHYS_INT_REGS' cpu/rtl/core/decode/uop_pkg.v`.
CPU_PRF_DEFINES :=
ifneq ($(PRF_INT),)
CPU_PRF_DEFINES += +define+PHYS_INT_REGS=$(PRF_INT) \
                   +define+PREG_INT_W=$(shell python3 -c "print(max(1,(int('$(PRF_INT)')-1).bit_length()))")
endif
ifneq ($(PRF_FP),)
CPU_PRF_DEFINES += +define+PHYS_FP_REGS=$(PRF_FP) \
                   +define+PREG_FP_W=$(shell python3 -c "print(max(1,(int('$(PRF_FP)')-1).bit_length()))")
endif

ifeq ($(CPU),m68k040)
# +define+L2C_ENABLE +define+VRAM_IN_DDR is NOT optional here -- axi_i is
# native 256b for this CPU with no CPU-side downconverter, and the ONLY
# 256b-safe binding is task #269's dedicated l2c fetch port (L2C_ENABLE).
# The L2C_ENABLE-undefined xbar M2 fallback is 128b-only and would
# silently truncate every instruction fetch; rtl/soc/fpga_top_cpu.vh has
# a hard `error backstop for anyone who strips this define back out.
# CPU_PRF_DEFINES is a no-op today (cpu040 has no PHYS_INT_REGS/
# PHYS_FP_REGS build-time knob the way cpu/'s uop_pkg.v does), forwarded
# anyway for symmetry in case that knob lands here later.
CPU_DEFINE      := +define+CPU_M68K040 +define+L2C_ENABLE +define+VRAM_IN_DDR $(CPU_PRF_DEFINES)
CPU_EXTRA_SRCS  := $(CPU_M68K040_SRCS)
CPU_EXTRA_IDIRS := $(CPU_M68K040_IDIRS)
# M68kSocketTop.v is SpinalHDL-generated and (a) carries its own
# `timescale directive, which every OTHER file in this repo lacks -- once
# ANY file sets a timescale Verilator (IEEE 1800-2023 3.14.2.3) warns on
# every module that doesn't, which is ~96 pre-existing SoC modules, none
# of which is actually a cpu040-side problem; (b) has one internal
# SpinalHDL name shadow (VARHIDDEN, FpRoundPack's `rp` result wire vs. its
# own instance name) that is cosmetic; and (c) genuinely uses `rst`
# ASYNCHRONOUSLY in at least one register bank (`always @(posedge clk or
# posedge rst)` in the ALU EU pipeline valids) where the rest of the SoC
# (and cpu_socket.vh group 1's port doc, "input rst ... synchronous")
# treats the same net as a synchronous, registered reset -- SYNCASYNCNET.
# (a) and (b) are pure lint noise; (c) is a REAL socket-contract question
# for cpu040 to confirm intentional (see the task #270/SOC-3 report) --
# suppressed here only so lint can finish and report OTHER real problems,
# not because it's been resolved.
CPU_EXTRA_LINT_FLAGS := -Wno-TIMESCALEMOD -Wno-VARHIDDEN -Wno-SYNCASYNCNET
else
CPU_DEFINE           :=
CPU_EXTRA_SRCS       :=
CPU_EXTRA_IDIRS      :=
CPU_EXTRA_LINT_FLAGS :=
endif

FPGA_TOP_RTL_SRCS := $(shell find $(RTL_DIR) \( -name '*.v' -o -name '*.sv' \) ! -name '._*' | sort)
FPGA_TOP_ROM_BUILD := $(BUILD_DIR)/fpga_top_rom
FPGA_TOP_ROM_PPM ?= $(BUILD_DIR)/fpga_top_rom/scaler_scanout.ppm
FPGA_TOP_ROM_MAX_INSTS ?= 100000000
FPGA_TOP_ROM_TIMEOUT ?= 2000000000
FPGA_TOP_ROM_PATCH ?=
FPGA_TOP_ROM_EXTRA ?=
# SD_IMAGE=<raw Mac HDD image>  — attach a functional SPI SD card model
# (tb/models/sd_card_spi.h) to tb-fpga-top-rom, mounted at SD LBA 8192 so
# the SCSI virtual HDD can actually serve blocks.  Unset = no card, which
# is the historical behaviour (sd_miso tied high).
SD_IMAGE ?=

.PHONY: lint-fpga-top
lint-fpga-top: $(if $(filter m68k040,$(CPU)),$(CPU_M68K040_V))
	$(VERILATOR) --lint-only --cc -DSIM_MODEL $(CPU_DEFINE) $(VERILATOR_EXTRA_DEFINES) \
		-I$(RTL_DIR)/core/decode -I$(RTL_DIR)/core -I$(RTL_DIR)/soc \
		-I$(RTL_DIR)/board -I$(RTL_DIR)/board/vendor \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		$(CPU_EXTRA_IDIRS) \
		-Wall \
		-Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-WIDTHTRUNC -Wno-WIDTHEXPAND -Wno-DECLFILENAME \
		-Wno-CASEINCOMPLETE -Wno-CMPCONST -Wno-UNOPTFLAT \
		-Wno-SELRANGE -Wno-LITENDIAN -Wno-UNSIGNED -Wno-BLKSEQ \
		$(CPU_EXTRA_LINT_FLAGS) \
		--top-module fpga_top \
		$(FPGA_TOP_RTL_SRCS) \
		$(CPU_EXTRA_SRCS) \
		$(TB_DIR)/verilator_xilinx_stubs.v

# lint-configs — lint fpga_top in every define-combination a real
# bitstream build can actually use.
#
# Why this exists (2026-07-25): plain `lint-fpga-top` above passes only
# -DSIM_MODEL, so the ENTIRE `ifdef VIO_ENABLE block in
# fpga_top_debug_vio.vh -- every VIO probe -- is skipped and never
# linted.  L2C_ENABLE / VRAM_IN_DDR are likewise opt-in env vars in
# synth/vivado.tcl.  A VIO probe that referenced L2C-only wires
# therefore linted clean and then failed synth ~5 minutes into a
# 40-minute impl run.  Run this before any impl.
#
# DISABLE_SD_JTAG_WRITER (2026-08-03) is the same trap one layer down, and
# a worse one: synth/vivado.tcl adds that define to EVERY production
# bitstream (it saves ~17.5K LUTs), so the `ifdef arm this repo was
# actually linting was the one that never ships.  Adding pram_sd behind
# axil_split2 turned the disabled-writer arm's old combinational
# AXI-Lite tie-off into a write-channel deadlock that no target here
# would have caught.  Both rows below elaborate the SHIPPING arm.
#
# ILA_ENABLE (2026-09-05) is the same trap again, and it went unnoticed for a
# WEEK. No row here passed -DILA_ENABLE, so the entire `ifdef ILA_ENABLE arm of
# fpga_top_debug_ctrl.vh was never elaborated by anything -- and it had been
# broken since 2026-08-28: it connected ~22 `.dbg040_*` ports that exist on no
# cpu040 ref. Nothing caught it because every board bitstream since was built
# with ENABLE_ILA=0, so the defect only surfaced when someone actually needed an
# ILA to debug a boot wedge, which is precisely when they can least afford to
# discover it. The `ila` row below elaborates the SHIPPING ILA arm, with the same
# define set a real `ENABLE_ILA=1` board build uses (see synth/vivado.tcl:997-999
# and 1030-1031: VIO_ENABLE, ILA_ENABLE, JTAG_AXI_ENABLE, plus the L2C/VRAM pair
# that CPU=m68k040 forces unconditionally).
#
# "ship" (2026-09-12) is the SAME trap once more, and it was the arm that
# mattered most: it is EXACTLY what tools/build_bitstream.sh produces by
# default (ENABLE_VIO=1, ENABLE_JTAG_AXI=1, L2C_ENABLE=1, VRAM_IN_DDR=1, plus
# the unconditional DISABLE_SD_JTAG_WRITER from synth/vivado.tcl:1230), and no
# row covered it.  The "ila" row is the only other one that defines
# JTAG_AXI_ENABLE, and it does NOT define DISABLE_SD_JTAG_WRITER; the
# "nosdjtag*" rows define DISABLE_SD_JTAG_WRITER but no host master.  So the
# combination every board bitstream actually ships -- a JTAG-AXI host with the
# SD writer gated out -- was elaborated by nothing.  The debug-bus work
# (docs/bus_debug_split_plan.md) lives entirely inside `ifdef JTAG_AXI_ENABLE,
# which is what made the hole worth closing before touching it.
#
# VIDEO_SMOKE is a top-level PARAMETER, not a define (synth/vivado.tcl
# passes it as a generic), so it needs `-GVIDEO_SMOKE=1` rather than a
# -D.  The `smoke` / `smoke+vio` rows cover the CPU-less video-test-rig
# build: VIDEO_SMOKE=1 + VRAM_IN_DDR + L2C_ENABLE, which was NOT reachable
# from any row here before -- the four original rows all elaborate the
# VIDEO_SMOKE=0 arm.  CPU= is whatever the invoking environment sets
# (CPU=stub is the default, and is the rig's configuration).
.PHONY: lint-configs
lint-configs: $(if $(filter m68k040,$(CPU)),$(CPU_M68K040_V))
	@rc=0; \
	for cfg in "default:" \
	           "vio:-DVIO_ENABLE" \
	           "l2c:-DL2C_ENABLE -DVRAM_IN_DDR" \
	           "vio+l2c:-DVIO_ENABLE -DL2C_ENABLE -DVRAM_IN_DDR" \
	           "smoke:-DL2C_ENABLE -DVRAM_IN_DDR -GVIDEO_SMOKE=1" \
	           "smoke+vio:-DVIO_ENABLE -DL2C_ENABLE -DVRAM_IN_DDR -GVIDEO_SMOKE=1" \
	           "nosdjtag:-DDISABLE_SD_JTAG_WRITER" \
	           "nosdjtag+l2c:-DDISABLE_SD_JTAG_WRITER -DL2C_ENABLE -DVRAM_IN_DDR" \
	           "ila:-DILA_ENABLE -DVIO_ENABLE -DJTAG_AXI_ENABLE -DL2C_ENABLE -DVRAM_IN_DDR" \
	           "ship:-DVIO_ENABLE -DJTAG_AXI_ENABLE -DL2C_ENABLE -DVRAM_IN_DDR -DDISABLE_SD_JTAG_WRITER" \
	           "scsitrace:-DSCSI_TRACE_ENABLE -DVIO_ENABLE -DL2C_ENABLE -DVRAM_IN_DDR" \
	           "noscsitrace:-DVIO_ENABLE -DL2C_ENABLE -DVRAM_IN_DDR"; do \
	  name=$${cfg%%:*}; defs=$${cfg#*:}; \
	  printf '%-10s ' "$$name"; \
	  if $(VERILATOR) --lint-only --cc -DSIM_MODEL $(CPU_DEFINE) $$defs \
	      -I$(RTL_DIR)/core/decode -I$(RTL_DIR)/core -I$(RTL_DIR)/soc \
	      -I$(RTL_DIR)/board -I$(RTL_DIR)/board/vendor \
	      -I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
	      $(CPU_EXTRA_IDIRS) -Wall \
	      -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
	      -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND -Wno-DECLFILENAME \
	      -Wno-CASEINCOMPLETE -Wno-CMPCONST -Wno-UNOPTFLAT \
	      -Wno-SELRANGE -Wno-LITENDIAN -Wno-UNSIGNED -Wno-BLKSEQ \
	      $(CPU_EXTRA_LINT_FLAGS) \
	      --top-module fpga_top $(FPGA_TOP_RTL_SRCS) $(CPU_EXTRA_SRCS) \
	      $(TB_DIR)/verilator_xilinx_stubs.v >/tmp/lintcfg_$$name.log 2>&1; \
	  then echo "PASS"; \
	  else echo "FAIL"; grep -E '%Error' /tmp/lintcfg_$$name.log | head -5; rc=1; fi; \
	done; \
	exit $$rc

# lint-realmig — elaborate the REAL-HARDWARE arm of the DDR path.
#
# Every row in lint-configs passes -DSIM_MODEL, so the whole
# `ifndef SIM_MODEL arm of rtl/soc/fpga_top_ddr.vh -- the DDR4 MIG black-box
# instance, the MIG<->fabric bridge wiring, the STARTUPE3/EOS clear path in
# fpga_top_clocks.vh -- is elaborated by NOTHING in this repo.  Only Vivado
# ever sees it, ~5 minutes into a 40-minute run.
#
# That is the same trap the lint-configs header already documents three times
# (VIO_ENABLE, DISABLE_SD_JTAG_WRITER, ILA_ENABLE), and it bit again on
# 2026-09-12: the DDR4 MIG's `sys_rst` was tied to `~btn[0]` and nothing else,
# so the memory controller was the one block in the design that a VIO reset
# could not reach -- and no lint here could have shown it, because no lint here
# elaborates that instance.
#
# The DDR4 stub already exists (rtl/board/vendor/design_1_ddr4_0_1_stub.v);
# the only thing missing was STARTUPE3, now in tb/verilator_xilinx_stubs.v.
# Defines match the `ship` row minus -DSIM_MODEL.
.PHONY: lint-realmig
lint-realmig: $(if $(filter m68k040,$(CPU)),$(CPU_M68K040_V))
	@printf '%-10s ' "realmig"; \
	if $(VERILATOR) --lint-only --cc $(CPU_DEFINE) $(VERILATOR_EXTRA_DEFINES) \
	    -DVIO_ENABLE -DJTAG_AXI_ENABLE -DL2C_ENABLE -DVRAM_IN_DDR \
	    -DDISABLE_SD_JTAG_WRITER \
	    -I$(RTL_DIR)/core/decode -I$(RTL_DIR)/core -I$(RTL_DIR)/soc \
	    -I$(RTL_DIR)/board -I$(RTL_DIR)/board/vendor \
	    -I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
	    $(CPU_EXTRA_IDIRS) -Wall \
	    -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
	    -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND -Wno-DECLFILENAME \
	    -Wno-CASEINCOMPLETE -Wno-CMPCONST -Wno-UNOPTFLAT \
	    -Wno-SELRANGE -Wno-LITENDIAN -Wno-UNSIGNED -Wno-BLKSEQ \
	    $(CPU_EXTRA_LINT_FLAGS) \
	    --top-module fpga_top $(FPGA_TOP_RTL_SRCS) $(CPU_EXTRA_SRCS) \
	    $(TB_DIR)/verilator_xilinx_stubs.v >/tmp/lintcfg_realmig.log 2>&1; \
	then echo "PASS"; \
	else echo "FAIL"; grep -E '%Error' /tmp/lintcfg_realmig.log | head -5; exit 1; fi

# ETH_ENABLE whole-link elaboration.
#
# Neither `make lint` (lint-configs) nor `make lint-fpga-top` defines
# ETH_ENABLE, so rtl/board/q700_eth_link.sv and its instance in
# fpga_top_ethernet.vh are never elaborated by either — a missing, misnamed or
# mis-sized port on that instance survives every lint and only surfaces ~50
# minutes into a Vivado run.  This target elaborates the REAL hierarchy against
# the pinned rk5-eth Taxi sources in a couple of seconds, for BOTH endpoint
# selections (ICMP responder and SONIC packet adapter), since each lives in a
# different arm of the same generate.  The top is fpga_top, not the link module
# alone: the port list is only checked where it is INSTANTIATED.
# The taxi/rk5-eth sources are VENDORED IN-TREE (vendor/rk5-eth), so an ETH build
# no longer depends on a sibling checkout existing.  vendor/rk5-eth mirrors the
# original rk5-eth layout exactly (third_party/taxi/... and rtl/...), so every
# $(ETH_RK5_DIR)-relative path still resolves unchanged.
# Override to build against an external rk5-eth checkout instead.
ETH_RK5_DIR ?= $(PROJ_ROOT)/vendor/rk5-eth
.PHONY: lint-eth-link
lint-eth-link:
	@if [ ! -d "$(ETH_RK5_DIR)" ]; then \
	  echo "eth-link   SKIP (no rk5-eth tree at $(ETH_RK5_DIR); set ETH_RK5_DIR)"; \
	  exit 0; \
	fi; \
	srcs=`python3 $(TOOLS_DIR)/taxi_filelist.py $(ETH_RK5_DIR)` || exit 1; \
	rc=0; \
	for ep in "icmp:1:" "sonic:0:" "sonic+dbg:0:-DETH_DEBUG_ENABLE" \
	          "sonic+netvhdd:0:-DENABLE_NET_VHDD"; do \
	  name=$${ep%%:*}; rest=$${ep#*:}; sel=$${rest%%:*}; defs=$${rest#*:}; \
	  printf '%-10s ' "eth-$$name"; \
	  if $(VERILATOR) --lint-only --cc -DETH_ENABLE -DSIM_MODEL $(CPU_DEFINE) $$defs \
	      -GETH_ICMP_RESPONDER=$$sel \
	      -I$(RTL_DIR)/core/decode -I$(RTL_DIR)/core -I$(RTL_DIR)/soc \
	      -I$(RTL_DIR)/board -I$(RTL_DIR)/board/vendor \
	      -I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
	      $(CPU_EXTRA_IDIRS) \
	      -Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
	      -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND -Wno-DECLFILENAME \
	      -Wno-CASEINCOMPLETE -Wno-CMPCONST -Wno-UNOPTFLAT \
	      -Wno-SELRANGE -Wno-LITENDIAN -Wno-UNSIGNED -Wno-BLKSEQ -Wno-TIMESCALEMOD \
	      -Wno-GENUNNAMED -Wno-SYNCASYNCNET \
	      $(CPU_EXTRA_LINT_FLAGS) \
	      --top-module fpga_top $(FPGA_TOP_RTL_SRCS) $(CPU_EXTRA_SRCS) $$srcs \
	      $(TB_DIR)/verilator_xilinx_stubs.v >/tmp/linteth_$$(echo $$name | tr + _).log 2>&1; \
	  then echo "PASS"; \
	  else echo "FAIL"; grep -E '%Error' /tmp/linteth_$$(echo $$name | tr + _).log | head -5; rc=1; fi; \
	done; \
	exit $$rc

# SIM_L2C_ENABLE — put the L2 cache in the CPU's memory path in SIMULATION.
#
# `l2c` is instantiated behind `ifdef L2C_ENABLE (rtl/soc/fpga_top_ddr.vh:101).
# That define was only ever set by synth/vivado.tcl for the BITSTREAM flow, so
# until now L2C had NEVER been elaborated in any CPU-bearing simulation --
# not tb-fpga-top-rom, not anything.  tb-l2c* exercise it standalone against
# BFMs with no CPU attached.
#
# Since L2C became default-on for bitstreams, every board build has shipped a
# memory path that no CPU simulation has executed a single instruction
# through.  That is the largest known sim/HW difference and is a prime
# suspect for HW-only memory bugs (see task #240).
#
# Usage: make tb-fpga-top-rom SIM_L2C_ENABLE=1
SIM_L2C_ENABLE ?= 0

.PHONY: tb-fpga-top-rom
tb-fpga-top-rom: $(FPGA_TOP_ROM_BUILD)/Vfpga_top
	$(CPU_MODEL_RUN_PREFIX)$(FPGA_TOP_ROM_BUILD)/Vfpga_top +rom=$(ROM) \
		+ppm=$(FPGA_TOP_ROM_PPM) \
		+max_insts=$(FPGA_TOP_ROM_MAX_INSTS) \
		+timeout=$(FPGA_TOP_ROM_TIMEOUT) \
		$(if $(FPGA_TOP_ROM_PATCH),+rom_patch=$(FPGA_TOP_ROM_PATCH)) \
		$(if $(SD_IMAGE),+sd_image=$(SD_IMAGE)) \
		$(FPGA_TOP_ROM_EXTRA)

.PHONY: tb-fpga-top-rom-monitor-guard
tb-fpga-top-rom-monitor-guard:
	$(MAKE) tb-fpga-top-rom \
		FPGA_TOP_ROM_MAX_INSTS=85000 \
		FPGA_TOP_ROM_TIMEOUT=3000000 \
		FPGA_TOP_ROM_PATCH=mame-fastdiag,chime-skip \
		FPGA_TOP_ROM_EXTRA="+fail_monitor"

# ── Synthetic boot-stub ROM (task #272 / SOC-3 behavioral smoke) ──────
# tb/tests/synth_boot/*.s are NOT firmware -- minimal 68k reset-vector
# boot stubs (a valid initial SSP @ address 0 / initial PC @ address 4,
# i.e. the real 68k cold-reset vector table, followed by a tight
# branch-to-self "idle" loop) used to confirm a CPU= backend's SOCKET
# integration -- reset deassert, first instruction fetch, sustained
# fetch/decode/execute without wedging -- with no dependency on any
# real OS/firmware ROM.  Built with the same M68K_AS/M68K_LD/
# M68K_OBJCOPY toolchain as tb/tests/cold_boot_periph (see
# idle_loop.s's header comment for the exact standalone invocation).
SYNTH_BOOT_SRC_DIR := $(TB_DIR)/tests/synth_boot
SYNTH_BOOT_BUILD   := $(BUILD_DIR)/synth_boot
SYNTH_BOOT_ASM     := $(wildcard $(SYNTH_BOOT_SRC_DIR)/*.s)
SYNTH_BOOT_BINS    := $(patsubst $(SYNTH_BOOT_SRC_DIR)/%.s,$(SYNTH_BOOT_BUILD)/%.bin,$(SYNTH_BOOT_ASM))

$(SYNTH_BOOT_BUILD)/%.bin: $(SYNTH_BOOT_SRC_DIR)/%.s
	@mkdir -p $(SYNTH_BOOT_BUILD)
	$(M68K_AS) -m68040 -o $(SYNTH_BOOT_BUILD)/$*.o $<
	$(M68K_LD) -Ttext 0x40000000 -o $(SYNTH_BOOT_BUILD)/$*.elf $(SYNTH_BOOT_BUILD)/$*.o
	$(M68K_OBJCOPY) -O binary $(SYNTH_BOOT_BUILD)/$*.elf $@

# `make tb-fpga-top-rom-synth-boot CPU=m68k040` -- intended use is
# CPU=m68k040 (SOC-3's new SpinalHDL core); CPU=stub/m68k both idle
# their socket and don't need a real boot stub to prove reset+fetch.
# Forwards straight to tb-fpga-top-rom with ROM=<built idle_loop.bin>;
# any other tb-fpga-top-rom knob (WAVES=1, FPGA_TOP_ROM_MAX_INSTS=...,
# FPGA_TOP_ROM_TIMEOUT=..., etc) still applies on top.
.PHONY: tb-fpga-top-rom-synth-boot
tb-fpga-top-rom-synth-boot: $(SYNTH_BOOT_BUILD)/idle_loop.bin
	$(MAKE) tb-fpga-top-rom ROM=$(SYNTH_BOOT_BUILD)/idle_loop.bin

# End-to-end SCC RX → CPU → SCC TX validation through fpga_top RTL.
# Boots the Q700 ROM with the fast-diag patch set, waits for MacsBug
# entry (rom_scc_rx_* poll), injects "G\r" on the SCC chan-A external
# RX byte interface, and captures up to SCC_TX_MAX bytes from BOTH the
# CPU-side TX byte interface (scc_uart_tx_valid/data) and the post-bridge
# UART pin (uart_rtl_0_txd).  Exits 0 once both viewpoints have collected
# their byte budgets.
SCC_RX_INJECT ?= G\\r
SCC_TX_MAX ?= 64
SCC_RX_INJECT_FAST ?= 1
SCC_LOOPBACK_PATCH ?= mame-fastdiag,chime-skip
SCC_LOOPBACK_MAX_INSTS ?= 5000000
SCC_LOOPBACK_TIMEOUT ?= 200000000
SCC_TX_LOG ?= $(BUILD_DIR)/fpga_top_rom/scc_tx.log
SCC_TX_UART_LOG ?= $(BUILD_DIR)/fpga_top_rom/scc_tx_uart.log

# ──────────────────────────────────────────────────────────────────────────────
# AXI lockstep — MAME vs RTL CPU-side bus parity
# ──────────────────────────────────────────────────────────────────────────────
# Captures every CPU-side AXI transaction (outside DDR + ROM) on both sides
# and byte-diffs the streams.  The first divergent transaction localises the
# peripheral / VRAM / DAFB register where the boot paths split.  See
# docs/axi_lockstep.md.
#
# Usage:
#   make tb-axi-lockstep
#   make tb-axi-lockstep AXI_LOCKSTEP_PATCH=<set> AXI_LOCKSTEP_MAX=10000
#
# Variables:
#   AXI_LOCKSTEP_PATCH   ROM patch set applied on BOTH sides (default
#                        "mame-fastdiag,chime-skip", matches the RTL boot
#                        path PM identified the divergence on).
#   AXI_LOCKSTEP_MAX     transaction cap; stop after N events on each side.
#                        5000 covers the ROM I/O-discovery / SIMM-probe
#                        window where divergence is expected.
#   AXI_LOCKSTEP_TIMEOUT sim_time cap for the RTL run (≥200M cycles
#                        recommended to reach the divergence).
#   AXI_LOCKSTEP_MAX_INSTS retired-uop cap for the RTL run.
#   MAME_ROM_PATH        rompath for `mame -rompath` (must contain
#                        macqd700/ + adbmodem/ ROM dirs; default
#                        /tmp/mame_iwm/roms — same as iwm lockstep).
#
# Outputs:
#   $(BUILD_DIR)/axi_lockstep/axi_mame.csv  — golden-side capture
#   $(BUILD_DIR)/axi_lockstep/axi_rtl.csv   — RTL-side capture
#   $(BUILD_DIR)/axi_lockstep/patches.txt   — applied byte-patch list
AXI_LOCKSTEP_DIR    := $(BUILD_DIR)/axi_lockstep
AXI_LOCKSTEP_PATCH  ?= mame-fastdiag,chime-skip
AXI_LOCKSTEP_MAX    ?= 5000
AXI_LOCKSTEP_TIMEOUT ?= 400000000
AXI_LOCKSTEP_MAX_INSTS ?= 100000000
AXI_LOCKSTEP_SECONDS ?= 4
MAME_ROM_PATH ?= /tmp/mame_iwm/roms

$(AXI_LOCKSTEP_DIR)/dump_rom_patches: $(TOOLS_DIR)/dump_rom_patches.cpp \
		$(TB_DIR)/models/rom_patch_sets.h
	@mkdir -p $(AXI_LOCKSTEP_DIR)
	g++ -std=c++17 -O2 -I$(TB_DIR)/models -o $@ $<

# ──────────────────────────────────────────────────────────────────────────────
# tb-via1-lockstep  —  MAME-vs-RTL VIA1 register-file byte parity
# ──────────────────────────────────────────────────────────────────────────────
# Captures every VIA1 register access during ROM boot on both sides and
# byte-diffs the streams.  Same shape as tb-axi-lockstep, but scoped to the
# VIA1 register file (16 registers x byte data) at the peripheral_bus
# master face.  See docs/via1_lockstep.md.
#
# Usage:
#   make tb-via1-lockstep
#   make tb-via1-lockstep VIA1_LOCKSTEP_PATCH=mame-fastdiag,chime-skip
#
# Variables:
#   VIA1_LOCKSTEP_PATCH    ROM patch set applied on BOTH sides.
#   VIA1_LOCKSTEP_MAX      event cap (default 4096 — covers ROM init).
#   VIA1_LOCKSTEP_TIMEOUT  sim_time cap for the RTL run.
#   VIA1_LOCKSTEP_MAX_INSTS retired-uop cap for the RTL run.
#   VIA1_LOCKSTEP_SECONDS  MAME-side capture wall-time (default 4).
#   MAME_ROM_PATH          rompath for `mame -rompath`.
#
# Outputs:
#   $(BUILD_DIR)/via1_lockstep/mame_via1.csv  — golden-side capture
#   $(BUILD_DIR)/via1_lockstep/rtl_via1.csv   — RTL-side capture
#   $(BUILD_DIR)/via1_lockstep/patches.txt    — applied byte-patch list
VIA1_LOCKSTEP_DIR    := $(BUILD_DIR)/via1_lockstep
VIA1_LOCKSTEP_NVRAM  := $(VIA1_LOCKSTEP_DIR)/mame_nvram
VIA1_LOCKSTEP_PATCH  ?= mame-fastdiag,chime-skip
VIA1_LOCKSTEP_MAX    ?= 4096
VIA1_LOCKSTEP_TIMEOUT ?= 400000000
VIA1_LOCKSTEP_MAX_INSTS ?= 100000000
VIA1_LOCKSTEP_SECONDS ?= 4

.PHONY: tb-via1-lockstep
tb-via1-lockstep: $(FPGA_TOP_ROM_BUILD)/Vfpga_top \
		$(AXI_LOCKSTEP_DIR)/dump_rom_patches
	@mkdir -p $(VIA1_LOCKSTEP_DIR)
	@echo "[via1-lockstep] generating patch file for set='$(VIA1_LOCKSTEP_PATCH)'"
	$(AXI_LOCKSTEP_DIR)/dump_rom_patches "$(VIA1_LOCKSTEP_PATCH)" \
		$(VIA1_LOCKSTEP_DIR)/patches.txt
	@echo "[via1-lockstep] capturing MAME-side trace -> $(VIA1_LOCKSTEP_DIR)/mame_via1.csv"
	@# Force MAME to boot from power-on-default (zero) PRAM by pointing
	@# -nvram_directory at a freshly-wiped dir.  MAME persists the RTC
	@# PRAM image to $HOME/.mame/nvram by default; a stale image makes
	@# MAME take the "PRAM valid" boot fast-path while the RTL (which
	@# resets PRAM to zero) takes the "PRAM invalid -> reinit" path, so
	@# the streams diverge for environmental reasons, not RTL bugs.
	@rm -rf $(VIA1_LOCKSTEP_NVRAM)
	@mkdir -p $(VIA1_LOCKSTEP_NVRAM)
	MAME_VIA1_TRACE_OUT=$(VIA1_LOCKSTEP_DIR)/mame_via1.csv \
	MAME_VIA1_TRACE_LIMIT=$(VIA1_LOCKSTEP_MAX) \
	MAME_VIA1_TRACE_SECONDS=$(VIA1_LOCKSTEP_SECONDS) \
	MAME_VIA1_PATCH_FILE=$(VIA1_LOCKSTEP_DIR)/patches.txt \
	mame -rompath $(MAME_ROM_PATH) -nvram_directory $(VIA1_LOCKSTEP_NVRAM) macqd700 \
		-window -resolution0 320x240 -nothrottle \
		-seconds_to_run $(VIA1_LOCKSTEP_SECONDS) -sound none -skip_gameinfo \
		-autoboot_delay 0 \
		-autoboot_script $(TOOLS_DIR)/mame_via1_capture.lua \
		>$(VIA1_LOCKSTEP_DIR)/mame.log 2>&1 || true
	@echo "[via1-lockstep] running RTL sim -> $(VIA1_LOCKSTEP_DIR)/rtl_via1.csv"
	$(CPU_MODEL_RUN_PREFIX)$(FPGA_TOP_ROM_BUILD)/Vfpga_top +rom=$(ROM) \
		+ppm=$(VIA1_LOCKSTEP_DIR)/scaler_scanout.ppm \
		+max_insts=$(VIA1_LOCKSTEP_MAX_INSTS) \
		+timeout=$(VIA1_LOCKSTEP_TIMEOUT) \
		+rom_patch=$(VIA1_LOCKSTEP_PATCH) \
		+via1_lockstep_log=$(VIA1_LOCKSTEP_DIR)/rtl_via1.csv \
		+via1_lockstep_max=$(VIA1_LOCKSTEP_MAX) \
		>$(VIA1_LOCKSTEP_DIR)/rtl.log 2>&1 || true
	@echo "[via1-lockstep] diffing"
	python3 $(TOOLS_DIR)/via1_lockstep_diff.py \
		$(VIA1_LOCKSTEP_DIR)/mame_via1.csv \
		$(VIA1_LOCKSTEP_DIR)/rtl_via1.csv

.PHONY: tb-axi-lockstep
tb-axi-lockstep: $(FPGA_TOP_ROM_BUILD)/Vfpga_top \
		$(AXI_LOCKSTEP_DIR)/dump_rom_patches
	@mkdir -p $(AXI_LOCKSTEP_DIR)
	@echo "[axi-lockstep] generating patch file for set='$(AXI_LOCKSTEP_PATCH)'"
	$(AXI_LOCKSTEP_DIR)/dump_rom_patches "$(AXI_LOCKSTEP_PATCH)" \
		$(AXI_LOCKSTEP_DIR)/patches.txt
	@echo "[axi-lockstep] capturing MAME-side trace -> $(AXI_LOCKSTEP_DIR)/axi_mame.csv"
	MAME_AXI_TRACE_OUT=$(AXI_LOCKSTEP_DIR)/axi_mame.csv \
	MAME_AXI_TRACE_LIMIT=$(AXI_LOCKSTEP_MAX) \
	MAME_AXI_TRACE_SECONDS=$(AXI_LOCKSTEP_SECONDS) \
	MAME_AXI_PATCH_FILE=$(AXI_LOCKSTEP_DIR)/patches.txt \
	mame -rompath $(MAME_ROM_PATH) macqd700 \
		-window -resolution0 320x240 -nothrottle \
		-seconds_to_run $(AXI_LOCKSTEP_SECONDS) -sound none -skip_gameinfo \
		-autoboot_delay 0 \
		-autoboot_script $(TOOLS_DIR)/mame_axi_capture.lua \
		>$(AXI_LOCKSTEP_DIR)/mame.log 2>&1 || true
	@echo "[axi-lockstep] running RTL sim -> $(AXI_LOCKSTEP_DIR)/axi_rtl.csv"
	$(CPU_MODEL_RUN_PREFIX)$(FPGA_TOP_ROM_BUILD)/Vfpga_top +rom=$(ROM) \
		+ppm=$(AXI_LOCKSTEP_DIR)/scaler_scanout.ppm \
		+max_insts=$(AXI_LOCKSTEP_MAX_INSTS) \
		+timeout=$(AXI_LOCKSTEP_TIMEOUT) \
		+rom_patch=$(AXI_LOCKSTEP_PATCH) \
		+axi_lockstep_log=$(AXI_LOCKSTEP_DIR)/axi_rtl.csv \
		+axi_lockstep_max=$(AXI_LOCKSTEP_MAX) \
		>$(AXI_LOCKSTEP_DIR)/rtl.log 2>&1 || true
	@echo "[axi-lockstep] diffing"
	python3 $(TOOLS_DIR)/axi_lockstep_diff.py \
		$(AXI_LOCKSTEP_DIR)/axi_mame.csv \
		$(AXI_LOCKSTEP_DIR)/axi_rtl.csv

# ──────────────────────────────────────────────────────────────────────────────
# tb-dafb-lockstep: DAFB-side MAME-vs-RTL byte-level lockstep
# ──────────────────────────────────────────────────────────────────────────────
# Captures every CPU-side AXI transaction targeting the DAFB register
# aperture (0xF980_0000..0xF980_03FF), the TurboSCSI register window
# (0x5000_F000..0x5000_F0FF) and the TurboSCSI DMA handshake
# (0x5000_F100..0x5000_F101) on both sides, and byte-diffs the streams.
# The first divergent transaction localises the DAFB register where the
# shim falls out of parity with MAME's reference dafb_device.  See
# docs/dafb_lockstep.md.
#
# Variables:
#   DAFB_LOCKSTEP_PATCH   ROM patch set applied on BOTH sides (default
#                         empty — the unpatched ROM is needed because
#                         mame-fastdiag jumps past the DAFB init).
#   DAFB_LOCKSTEP_MAX     transaction cap; stop after N events on each
#                         side.  8000 covers the boot DAFB programming
#                         + first-frame status polls.
#   DAFB_LOCKSTEP_TIMEOUT sim_time cap for the RTL run.
#   DAFB_LOCKSTEP_MAX_INSTS retired-uop cap for the RTL run.
#   DAFB_LOCKSTEP_SECONDS MAME wall-clock seconds to capture for
#                         (default 4 s — the Q700 ROM finishes its
#                         DAFB init within ~1 s, but extra time
#                         catches the first VBL/sense polling round).
#   DAFB_LOCKSTEP_INCLUDE_VRAM (0/1) — include VRAM aperture writes too.
#                         Default 0 (off) since VRAM traffic dominates.
#   MAME_ROM_PATH         rompath for `mame -rompath`.
DAFB_LOCKSTEP_DIR        := $(BUILD_DIR)/dafb_lockstep
DAFB_LOCKSTEP_PATCH      ?=
DAFB_LOCKSTEP_MAX        ?= 8000
DAFB_LOCKSTEP_TIMEOUT    ?= 600000000
DAFB_LOCKSTEP_MAX_INSTS  ?= 200000000
DAFB_LOCKSTEP_SECONDS    ?= 4
DAFB_LOCKSTEP_INCLUDE_VRAM ?= 0

$(DAFB_LOCKSTEP_DIR)/dump_rom_patches: $(TOOLS_DIR)/dump_rom_patches.cpp \
		$(TB_DIR)/models/rom_patch_sets.h
	@mkdir -p $(DAFB_LOCKSTEP_DIR)
	g++ -std=c++17 -O2 -I$(TB_DIR)/models -o $@ $<

.PHONY: tb-dafb-lockstep
tb-dafb-lockstep: $(FPGA_TOP_ROM_BUILD)/Vfpga_top \
		$(DAFB_LOCKSTEP_DIR)/dump_rom_patches
	@mkdir -p $(DAFB_LOCKSTEP_DIR)
	@echo "[dafb-lockstep] generating patch file for set='$(DAFB_LOCKSTEP_PATCH)'"
	$(DAFB_LOCKSTEP_DIR)/dump_rom_patches "$(DAFB_LOCKSTEP_PATCH)" \
		$(DAFB_LOCKSTEP_DIR)/patches.txt
	@echo "[dafb-lockstep] capturing MAME-side trace -> $(DAFB_LOCKSTEP_DIR)/dafb_mame.csv"
	MAME_DAFB_TRACE_OUT=$(DAFB_LOCKSTEP_DIR)/dafb_mame.csv \
	MAME_DAFB_TRACE_LIMIT=$(DAFB_LOCKSTEP_MAX) \
	MAME_DAFB_TRACE_SECONDS=$(DAFB_LOCKSTEP_SECONDS) \
	MAME_DAFB_INCLUDE_VRAM=$(DAFB_LOCKSTEP_INCLUDE_VRAM) \
	MAME_DAFB_PATCH_FILE=$(DAFB_LOCKSTEP_DIR)/patches.txt \
	mame -rompath $(MAME_ROM_PATH) macqd700 \
		-window -resolution0 320x240 -nothrottle \
		-seconds_to_run $(DAFB_LOCKSTEP_SECONDS) -sound none -skip_gameinfo \
		-autoboot_delay 0 \
		-autoboot_script $(TOOLS_DIR)/mame_dafb_capture.lua \
		>$(DAFB_LOCKSTEP_DIR)/mame.log 2>&1 || true
	@echo "[dafb-lockstep] running RTL sim -> $(DAFB_LOCKSTEP_DIR)/dafb_rtl.csv"
	$(CPU_MODEL_RUN_PREFIX)$(FPGA_TOP_ROM_BUILD)/Vfpga_top +rom=$(ROM) \
		+ppm=$(DAFB_LOCKSTEP_DIR)/scaler_scanout.ppm \
		+max_insts=$(DAFB_LOCKSTEP_MAX_INSTS) \
		+timeout=$(DAFB_LOCKSTEP_TIMEOUT) \
		$(if $(DAFB_LOCKSTEP_PATCH),+rom_patch=$(DAFB_LOCKSTEP_PATCH)) \
		+dafb_lockstep_log=$(DAFB_LOCKSTEP_DIR)/dafb_rtl.csv \
		+dafb_lockstep_max=$(DAFB_LOCKSTEP_MAX) \
		$(if $(filter 1,$(DAFB_LOCKSTEP_INCLUDE_VRAM)),+dafb_lockstep_include_vram) \
		>$(DAFB_LOCKSTEP_DIR)/rtl.log 2>&1 || true
	@echo "[dafb-lockstep] diffing"
	python3 $(TOOLS_DIR)/dafb_lockstep_diff.py \
		$(DAFB_LOCKSTEP_DIR)/dafb_mame.csv \
		$(DAFB_LOCKSTEP_DIR)/dafb_rtl.csv

.PHONY: tb-scc-uart-loopback
tb-scc-uart-loopback: $(FPGA_TOP_ROM_BUILD)/Vfpga_top
	@mkdir -p $(BUILD_DIR)/fpga_top_rom
	$(CPU_MODEL_RUN_PREFIX)$(FPGA_TOP_ROM_BUILD)/Vfpga_top +rom=$(ROM) \
		+ppm=$(FPGA_TOP_ROM_PPM) \
		+max_insts=$(SCC_LOOPBACK_MAX_INSTS) \
		+timeout=$(SCC_LOOPBACK_TIMEOUT) \
		+rom_patch=$(SCC_LOOPBACK_PATCH) \
		+scc_rx_inject="$(SCC_RX_INJECT)" \
		$(if $(filter 1,$(SCC_RX_INJECT_FAST)),+scc_rx_inject_fast) \
		+scc_tx_log=$(SCC_TX_LOG) \
		+scc_tx_uart_log=$(SCC_TX_UART_LOG) \
		+scc_tx_max=$(SCC_TX_MAX)

$(FPGA_TOP_ROM_BUILD)/Vfpga_top: $(FPGA_TOP_RTL_SRCS) $(CPU_EXTRA_SRCS) \
		$(TB_DIR)/verilator_xilinx_stubs.v \
		$(TB_DIR)/tb_fpga_top_rom.cpp \
		$(TB_DIR)/models/rom_patch_sets.h \
		$(TB_DIR)/models/sd_card_spi.h
	@mkdir -p $(FPGA_TOP_ROM_BUILD)
	$(VERILATOR) --cc --exe --build --assert --public-flat-rw --savable \
		$(if $(filter 1,$(WAVES)),--trace-fst,) \
		-DSIM_MODEL -DFPGA_ROM_SIM $(CPU_DEFINE) \
		$(if $(filter 1,$(SIM_L2C_ENABLE)),-DL2C_ENABLE,) \
		$(if $(SIM_DDR_READ_DELAY),+define+SIM_DDR_READ_DELAY=$(SIM_DDR_READ_DELAY),) \
		$(if $(filter 1,$(DEBUG)),+define+CORE_DEBUG +define+LSU_DEBUG,) \
		--x-assign fast --x-initial fast -O3 \
		-j $(VERILATOR_JOBS) --threads $(VERILATOR_THREADS) \
		--unroll-count 128 \
		-I$(RTL_DIR)/core/decode -I$(RTL_DIR)/core -I$(RTL_DIR)/soc -I$(RTL_DIR)/board \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		$(CPU_EXTRA_IDIRS) \
		-Wall \
		-Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-WIDTHTRUNC -Wno-WIDTHEXPAND -Wno-DECLFILENAME \
		-Wno-CASEINCOMPLETE -Wno-CMPCONST -Wno-UNOPTFLAT \
		-Wno-UNOPTTHREADS -Wno-SELRANGE -Wno-LITENDIAN \
		-Wno-UNSIGNED -Wno-BLKSEQ \
		$(CPU_EXTRA_LINT_FLAGS) \
		-Mdir $(FPGA_TOP_ROM_BUILD) \
		--top-module fpga_top \
		$(FPGA_TOP_RTL_SRCS) \
		$(CPU_EXTRA_SRCS) \
		$(TB_DIR)/verilator_xilinx_stubs.v \
		$(TB_DIR)/tb_fpga_top_rom.cpp \
		-CFLAGS "-std=c++17 $(if $(filter m68k,$(CPU)),-DCPU_M68K,) $(if $(filter m68k040,$(CPU)),-DCPU_M68K040,) $(if $(filter 1,$(SIM_L2C_ENABLE)),-DSIM_L2C_ENABLE,) -I$(TB_DIR) -I$(TB_DIR)/models"

# fpga_top_rom_realboot: same fpga_top_rom harness, but WITHOUT the
# FPGA_ROM_SIM boot bypass -- boot_fsm runs for real (cpu_rst/
# boot_rom_ready genuinely wait for its own SD-streaming-and-mirror
# completion, no vio_boot_ctrl force-release) while still getting
# FPGA_ROM_SIM's DDR-model sizing (ddr_ctrl.v's addr_to_idx/BEATS,
# sized to hold the real 1 MiB ROM at its real production addresses:
# 0x0 RAM window + 0x4000_0000 native ROM window -- see rtl/board/
# ddr_ctrl.v and rtl/soc/axi_defs.vh AXI_ROM_BASE/AXI_ROM_SIZE), not
# the tiny 4096-beat/64 KB SIM_MODEL default that can't hold it.
#
# Drive with +no_preload (skip the DRAM-backdoor ROM write entirely)
# and +sd_card_image=<img> (a real SD-card image with the ROM at LBA 0,
# tb/models/sd_card_spi.h's attach_raw) so the CPU can ONLY see ROM
# content that boot_fsm itself streamed in over the modelled SD/SPI
# bus -- see docs background in the task that added this target.
#
# Usage: make tb-fpga-top-rom-realboot CPU=m68k040 \
#          SD_CARD_IMAGE=<path> FPGA_TOP_ROM_EXTRA="+no_preload ..."
FPGA_TOP_ROM_REALBOOT_BUILD := $(BUILD_DIR)/fpga_top_rom_realboot
SD_CARD_IMAGE ?=

.PHONY: tb-fpga-top-rom-realboot
tb-fpga-top-rom-realboot: $(FPGA_TOP_ROM_REALBOOT_BUILD)/Vfpga_top
	$(CPU_MODEL_RUN_PREFIX)$(FPGA_TOP_ROM_REALBOOT_BUILD)/Vfpga_top +rom=$(ROM) \
		+ppm=$(FPGA_TOP_ROM_PPM) \
		+max_insts=$(FPGA_TOP_ROM_MAX_INSTS) \
		+timeout=$(FPGA_TOP_ROM_TIMEOUT) \
		+no_preload \
		$(if $(FPGA_TOP_ROM_PATCH),+rom_patch=$(FPGA_TOP_ROM_PATCH)) \
		$(if $(SD_CARD_IMAGE),+sd_card_image=$(SD_CARD_IMAGE)) \
		$(FPGA_TOP_ROM_EXTRA)

$(FPGA_TOP_ROM_REALBOOT_BUILD)/Vfpga_top: $(FPGA_TOP_RTL_SRCS) $(CPU_EXTRA_SRCS) \
		$(TB_DIR)/verilator_xilinx_stubs.v \
		$(TB_DIR)/tb_fpga_top_rom.cpp \
		$(TB_DIR)/models/rom_patch_sets.h \
		$(TB_DIR)/models/sd_card_spi.h
	@mkdir -p $(FPGA_TOP_ROM_REALBOOT_BUILD)
	$(VERILATOR) --cc --exe --build --assert --public-flat-rw \
		$(if $(filter 1,$(WAVES)),--trace-fst,) \
		-DSIM_MODEL -DFPGA_ROM_SIM -DFPGA_ROM_SIM_REALBOOT $(CPU_DEFINE) \
		$(if $(filter 1,$(SIM_L2C_ENABLE)),-DL2C_ENABLE,) \
		$(if $(SIM_DDR_READ_DELAY),+define+SIM_DDR_READ_DELAY=$(SIM_DDR_READ_DELAY),) \
		$(if $(filter 1,$(DEBUG)),+define+CORE_DEBUG +define+LSU_DEBUG,) \
		--x-assign fast --x-initial fast -O3 \
		-j $(VERILATOR_JOBS) --threads $(VERILATOR_THREADS) \
		--unroll-count 128 \
		-I$(RTL_DIR)/core/decode -I$(RTL_DIR)/core -I$(RTL_DIR)/soc -I$(RTL_DIR)/board \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		$(CPU_EXTRA_IDIRS) \
		-Wall \
		-Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-WIDTHTRUNC -Wno-WIDTHEXPAND -Wno-DECLFILENAME \
		-Wno-CASEINCOMPLETE -Wno-CMPCONST -Wno-UNOPTFLAT \
		-Wno-UNOPTTHREADS -Wno-SELRANGE -Wno-LITENDIAN \
		-Wno-UNSIGNED -Wno-BLKSEQ \
		$(CPU_EXTRA_LINT_FLAGS) \
		-Mdir $(FPGA_TOP_ROM_REALBOOT_BUILD) \
		--top-module fpga_top \
		$(FPGA_TOP_RTL_SRCS) \
		$(CPU_EXTRA_SRCS) \
		$(TB_DIR)/verilator_xilinx_stubs.v \
		$(TB_DIR)/tb_fpga_top_rom.cpp \
		-CFLAGS "-std=c++17 $(if $(filter m68k,$(CPU)),-DCPU_M68K,) $(if $(filter m68k040,$(CPU)),-DCPU_M68K040,) $(if $(filter 1,$(SIM_L2C_ENABLE)),-DSIM_L2C_ENABLE,) -I$(TB_DIR) -I$(TB_DIR)/models"

# fpga_top_rom_mig: same as fpga_top_rom but routes through the production
# axi_ddr4_mig_bridge + a behavioural 256-bit sim_mig_backend.  Exercises
# the MIG-side byte-lane packing/wstrb that regular SIM_MODEL elides.
# Add SIM_MIG_IGNORE_WSTRB=1 to additionally simulate the hypothesised
# "real DDR4 MIG drops wstrb on partial writes" failure mode.
FPGA_TOP_ROM_MIG_BUILD := $(BUILD_DIR)/fpga_top_rom_mig
.PHONY: tb-fpga-top-rom-mig
tb-fpga-top-rom-mig: $(FPGA_TOP_ROM_MIG_BUILD)/Vfpga_top

$(FPGA_TOP_ROM_MIG_BUILD)/Vfpga_top: $(FPGA_TOP_RTL_SRCS) $(CPU_EXTRA_SRCS) \
		$(TB_DIR)/verilator_xilinx_stubs.v \
		$(TB_DIR)/tb_fpga_top_rom.cpp \
		$(TB_DIR)/models/rom_patch_sets.h \
		$(TB_DIR)/models/sd_card_spi.h
	@mkdir -p $(FPGA_TOP_ROM_MIG_BUILD)
	$(VERILATOR) --cc --exe --build --assert --public-flat-rw \
		$(if $(filter 1,$(WAVES)),--trace-fst,) \
		-DSIM_MODEL -DFPGA_ROM_SIM -DSIM_MIG_BRIDGE \
		$(CPU_DEFINE) \
		$(if $(filter 1,$(SIM_MIG_IGNORE_WSTRB)),-DSIM_MIG_IGNORE_WSTRB,) \
		$(if $(filter 1,$(SIM_MIG_DROP_HI_PER_NIBBLE)),-DSIM_MIG_DROP_HI_PER_NIBBLE,) \
		$(if $(filter 1,$(SIM_MIG_DROP_UPPER_HALF)),-DSIM_MIG_DROP_UPPER_HALF,) \
		$(if $(filter 1,$(SIM_MIG_DROP_BIT3_PER_NIBBLE)),-DSIM_MIG_DROP_BIT3_PER_NIBBLE,) \
		--x-assign fast --x-initial fast -O3 \
		-j $(VERILATOR_JOBS) --threads $(VERILATOR_THREADS) \
		--unroll-count 128 \
		-I$(RTL_DIR)/core/decode -I$(RTL_DIR)/core -I$(RTL_DIR)/soc -I$(RTL_DIR)/board \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		$(CPU_EXTRA_IDIRS) \
		-Wall \
		-Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-WIDTHTRUNC -Wno-WIDTHEXPAND -Wno-DECLFILENAME \
		-Wno-CASEINCOMPLETE -Wno-CMPCONST -Wno-UNOPTFLAT \
		-Wno-UNOPTTHREADS -Wno-SELRANGE -Wno-LITENDIAN \
		-Wno-UNSIGNED -Wno-BLKSEQ \
		$(CPU_EXTRA_LINT_FLAGS) \
		-Mdir $(FPGA_TOP_ROM_MIG_BUILD) \
		--top-module fpga_top \
		$(FPGA_TOP_RTL_SRCS) \
		$(CPU_EXTRA_SRCS) \
		$(TB_DIR)/verilator_xilinx_stubs.v \
		$(TB_DIR)/tb_fpga_top_rom.cpp \
		-CFLAGS "-std=c++17 -DSIM_MIG_BRIDGE $(if $(filter m68k,$(CPU)),-DCPU_M68K,) $(if $(filter m68k040,$(CPU)),-DCPU_M68K040,) -I$(TB_DIR) -I$(TB_DIR)/models"

.PHONY: lint-fpga-top-real-mig
lint-fpga-top-real-mig:
	$(VERILATOR) --lint-only --cc \
		-I$(RTL_DIR)/core/decode -I$(RTL_DIR)/core -I$(RTL_DIR)/soc -I$(RTL_DIR)/board \
		-I$(RTL_DIR)/board/video_phy -I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Wall \
		-Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-WIDTHTRUNC -Wno-WIDTHEXPAND -Wno-DECLFILENAME \
		-Wno-CASEINCOMPLETE -Wno-CMPCONST -Wno-UNOPTFLAT \
		-Wno-SELRANGE -Wno-LITENDIAN -Wno-UNSIGNED -Wno-BLKSEQ \
		--top-module fpga_top \
		$(FPGA_TOP_RTL_SRCS) \
		$(TB_DIR)/verilator_xilinx_stubs.v

# ──────────────────────────────────────────────────────────────────────────────
# ISA coverage report
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: coverage
coverage:
	python3 $(TOOLS_DIR)/isa_coverage.py --logdir $(BUILD_DIR)/logs

# ──────────────────────────────────────────────────────────────────────────────
# Vivado synthesis / implementation (non-project mode)
# ──────────────────────────────────────────────────────────────────────────────
VIVADO_IMPL_DIR ?= $(BUILD_DIR)/vivado
FPGA_BUILDINFO = $(VIVADO_IMPL_DIR)/fpga_top.buildinfo

# Incremental-compile reference checkpoint.  When this DCP exists AND
# NO_INCREMENTAL is 0, vivado.tcl calls `read_checkpoint -incremental` before
# opt_design so place+route reuse the prior result for unchanged hierarchy
# (typical 40–60 % impl-time saving on peripheral-only edits).  After a
# successful route the new route.dcp is copied here automatically.
#
# INCREMENTAL IS OFF BY DEFAULT (2026-08-07).  A stale reference produced two
# routing-congestion failures in one day — `[Route 35-2] Design is not legally
# routed`, 23054 then 2096 node overlaps — each costing a full impl run, and
# both cleared by `make clean-incremental` + a full rebuild.  The failure is
# also self-perpetuating: the reference is REFRESHED after every successful
# route, so a build that merely placed badly seeds the next one.  Worse, the
# guard is "does the DCP file exist", so deleting it looks like it disabled
# incremental while the next successful build silently re-armed it.
#
# Re-enable deliberately with NO_INCREMENTAL=0 when iterating on a small,
# known-local edit against a reference you trust — not as the default.
NO_INCREMENTAL      ?= 1
INCREMENTAL_REF_DCP ?= $(BUILD_DIR)/vivado_incremental_ref/route.dcp

# ── impl defaults: debug-enabled bitstream ────────────────────────────────────
# Bare `make impl` should produce a flashable, JTAG-debuggable bitstream.
# These ?= defaults are picked up by the env-var plumbing below; explicit
# preset targets (e.g. fpga-100mhz-jtag-bitstream) and operator overrides
# still win because they assign before reaching this point.
ENABLE_VIO            ?= 1
ENABLE_JTAG_AXI       ?= 1
REAL_FPGA_BUILD       ?= 1
USE_REAL_MIG          ?= 1
VIDEO_SMOKE           ?= 0
TARGET_FREQ_MHZ       ?= 100
# CORE_CLK_HZ DERIVES FROM TARGET_FREQ_MHZ, and CORE_MMCM follows it.
#
# These used to be three independent knobs defaulting to 100 MHz, so
# `make impl TARGET_FREQ_MHZ=200` constrained the design to 200 MHz while still
# CLOCKING THE CORE AT 100 -- a bitstream that meets timing trivially, boots fine,
# and is not running at the frequency its name and buildinfo imply. That is not
# hypothetical: build/vivado150_1080p30 was loaded and reported as "150 MHz" while
# its buildinfo recorded core_clk_hz=100000000.
#
# Above the 100 MHz board oscillator the core clock must come from the MMCM
# (synth/vivado.tcl: VCO 1200 MHz, CLKOUT0 = core, CLKOUT1 = pb at exactly
# 1200/24 = 50.000 MHz), and only frequencies dividing 1200 cleanly are legal --
# every Mac peripheral timebase in this SoC is derived from PB_CLK_HZ.
# ⚠️ 120 MHz is in vivado.tcl's legal set but is NOT usable in practice: the 12:5
# ratio against pb leaves a 1.667 ns window and fails by ~19k endpoints. Use
# 100 / 150 / 200.
CORE_CLK_HZ           ?= $(strip $(TARGET_FREQ_MHZ))_000_000
ifneq ($(filter-out 100,$(strip $(TARGET_FREQ_MHZ))),)
CORE_MMCM             ?= 1
endif
PB_CLK_HZ             ?= 50_000_000
BOOT_ROM_SECTORS      ?= 2048
# Ethernet ON by default, in its SONIC DMA form -- that is the configuration the
# board actually runs and the one every recent bitstream was built with.
# ETH_ICMP_RESPONDER selects the OTHER generate branch in q700_eth_link (a
# standalone ICMP responder, no SONIC DMA); defaulting it to 1 meant a plain
# `ETH_ENABLE=1` build silently produced the wrong ethernet.
ETH_ENABLE            ?= 1
ETH_ICMP_RESPONDER    ?= 0
ETH_DEBUG_ENABLE      ?= 0
# Resolve against whichever location actually exists: this repo is usually built
# from a WORKTREE, where $(PROJ_ROOT)/.. is the worktrees directory and does NOT
# contain rk5-eth.  The bare default then fails the build at read_all_rtl with
# "ETH_ENABLE=1 requires rk5-eth at ...".  Last entry keeps that error message
# pointing somewhere sensible when neither path exists.
# ETH_RK5_DIR is defined ABOVE (search "this is the FIRST `?=`").  `?=` takes the
# first assignment, so repeating it here would have no effect.

VIVADO_RUN_ENV := PCIE_TEST_DIR=$(PCIE_TEST_DIR) INCREMENTAL_REF_DCP=$(INCREMENTAL_REF_DCP)
# CPU socket select for synth/impl — same CPU=stub|m68k knob as the
# Verilator flow (see "CPU build select" above); consumed by vivado.tcl.
VIVADO_RUN_ENV += CPU=$(CPU)
ifneq ($(NO_INCREMENTAL),)
VIVADO_RUN_ENV += NO_INCREMENTAL=$(NO_INCREMENTAL)
endif
ifneq ($(origin DDR4_MIG_DCP),file)
VIVADO_RUN_ENV += DDR4_MIG_DCP=$(DDR4_MIG_DCP)
endif
ifneq ($(origin PCIE_XDMA_XCI),file)
VIVADO_RUN_ENV += PCIE_XDMA_XCI=$(PCIE_XDMA_XCI)
endif
# L2C / VRAM-in-DDR cutover knobs.  synth/vivado.tcl gates these on
# `info exists ::env(...)`, so they must reach the vivado process as real
# environment variables.  Forwarding them here makes `make impl
# L2C_ENABLE=1 VRAM_IN_DDR=1` work explicitly instead of depending on the
# caller having exported them into make's own environment.
#
# These two belong together: l2c's 8-way 2 MB data array needs ~64
# URAM288, which only fits the KU5P budget once VRAM-in-DDR frees the
# VRAM array's ~57 (see the L2C_ENABLE comment block in vivado.tcl).
#
# DEFAULT ON since 2026-08-03 (user directive: "next build should have L2C
# on and maybe it should always be on from now on").  Before this, both
# were opt-in, so every routine bitstream shipped WITHOUT the L2C -- and
# nothing in fpga_top.buildinfo recorded that fact, so the only way to
# tell after the fact was to grep the routed design for l2c cells.  See
# the `buildinfo` note below: the config is now recorded.
#
# To build WITHOUT the L2C, pass `L2C_ENABLE=0` (or `no`/`off`).  Note
# that synth/vivado.tcl gates on `info exists ::env(L2C_ENABLE)`, NOT on
# its value -- so forwarding `L2C_ENABLE=0` would still switch it ON.
# The filter below therefore suppresses the export entirely rather than
# exporting a falsey value.
L2C_ENABLE  ?= 1
VRAM_IN_DDR ?= 1
# CRITICAL: GNU make AUTO-EXPORTS variables set on the command line into every
# recipe's environment.  synth/vivado.tcl gates on `info exists ::env(...)`,
# NOT on the value -- so without these unexports, `make impl L2C_ENABLE=0`
# puts L2C_ENABLE=0 in vivado's environment and turns the cache ON.  Measured
# 2026-08-03: a build invoked with L2C_ENABLE=0 VRAM_IN_DDR=0 produced a
# bitstream with 146 l2c cells and buildinfo l2c_enable=1.  The explicit
# VIVADO_RUN_ENV suppression below is necessary but NOT sufficient on its own.
unexport L2C_ENABLE
unexport VRAM_IN_DDR
ifeq ($(filter 0 no off false,$(L2C_ENABLE)),)
ifneq ($(L2C_ENABLE),)
VIVADO_RUN_ENV += L2C_ENABLE=$(L2C_ENABLE)
endif
endif
ifeq ($(filter 0 no off false,$(VRAM_IN_DDR)),)
ifneq ($(VRAM_IN_DDR),)
VIVADO_RUN_ENV += VRAM_IN_DDR=$(VRAM_IN_DDR)
endif
endif
# CPU physical-register-file size knob.  Same env-var pattern as
# L2C_ENABLE above: synth/vivado.tcl gates on `info exists ::env(...)`,
# so `make impl CPU=m68k PRF_INT=64` has to reach the vivado process as
# a real environment variable.  vivado.tcl derives the matching tag width
# (PREG_INT_W / PREG_FP_W) and emits both -verilog_define pairs; the CPU's
# rat.v / fp_rat.v carry a hard elaboration check that fails synthesis if
# the pair is ever inconsistent.  Only meaningful with CPU=m68k.
ifneq ($(PRF_INT),)
VIVADO_RUN_ENV += PRF_INT=$(PRF_INT)
endif
ifneq ($(PRF_FP),)
VIVADO_RUN_ENV += PRF_FP=$(PRF_FP)
endif
ifneq ($(origin PCIE_XDMA_DCP),file)
VIVADO_RUN_ENV += PCIE_XDMA_DCP=$(PCIE_XDMA_DCP)
endif
ifneq ($(REAL_FPGA_BUILD),)
VIVADO_RUN_ENV += REAL_FPGA_BUILD=$(REAL_FPGA_BUILD)
endif
ifneq ($(USE_REAL_MIG),)
VIVADO_RUN_ENV += USE_REAL_MIG=$(USE_REAL_MIG)
endif
ifneq ($(USE_PCIE_TEST_MIG_DCP),)
VIVADO_RUN_ENV += USE_PCIE_TEST_MIG_DCP=$(USE_PCIE_TEST_MIG_DCP)
endif
ifneq ($(USE_PCIE_TEST_XDMA_XCI),)
VIVADO_RUN_ENV += USE_PCIE_TEST_XDMA_XCI=$(USE_PCIE_TEST_XDMA_XCI)
endif
ifneq ($(ALLOW_UNDEBUGGABLE_FPGA_BUILD),)
VIVADO_RUN_ENV += ALLOW_UNDEBUGGABLE_FPGA_BUILD=$(ALLOW_UNDEBUGGABLE_FPGA_BUILD)
endif
ifneq ($(ALLOW_SIM_MODEL_ROM_WRAP),)
VIVADO_RUN_ENV += ALLOW_SIM_MODEL_ROM_WRAP=$(ALLOW_SIM_MODEL_ROM_WRAP)
endif
ifneq ($(TARGET_FREQ_MHZ),)
VIVADO_RUN_ENV += TARGET_FREQ_MHZ=$(TARGET_FREQ_MHZ)
endif
ifneq ($(CORE_CLK_DIVIDE),)
VIVADO_RUN_ENV += CORE_CLK_DIVIDE=$(CORE_CLK_DIVIDE)
endif
ifneq ($(CORE_CLK_HZ),)
VIVADO_RUN_ENV += CORE_CLK_HZ=$(CORE_CLK_HZ)
endif
# CORE_MMCM=1 swaps the BUFG_GT core/pb divider tree for an MMCM so the core
# can run ABOVE the 100 MHz board oscillator (pb stays exactly 50 MHz).
# Unset = the shipping topology, bit-identical to before.  See the CORE_MMCM
# block in synth/vivado.tcl and rtl/soc/fpga_top_clocks.vh.
ifneq ($(CORE_MMCM),)
VIVADO_RUN_ENV += CORE_MMCM=$(CORE_MMCM)
endif
ifneq ($(PB_CLK_HZ),)
VIVADO_RUN_ENV += PB_CLK_HZ=$(PB_CLK_HZ)
endif
ifneq ($(VIDEO_SMOKE),)
VIVADO_RUN_ENV += VIDEO_SMOKE=$(VIDEO_SMOKE)
endif
ifneq ($(BOOT_ROM_SECTORS),)
VIVADO_RUN_ENV += BOOT_ROM_SECTORS=$(BOOT_ROM_SECTORS)
endif
ifneq ($(ENABLE_VIO),)
VIVADO_RUN_ENV += ENABLE_VIO=$(ENABLE_VIO)
endif
ifneq ($(ENABLE_JTAG_AXI),)
VIVADO_RUN_ENV += ENABLE_JTAG_AXI=$(ENABLE_JTAG_AXI)
endif
ifneq ($(ENABLE_PCIE_XDMA),)
VIVADO_RUN_ENV += ENABLE_PCIE_XDMA=$(ENABLE_PCIE_XDMA)
endif
ifneq ($(ENABLE_ILA),)
VIVADO_RUN_ENV += ENABLE_ILA=$(ENABLE_ILA)
endif
ifneq ($(ETH_ENABLE),)
VIVADO_RUN_ENV += ETH_ENABLE=$(ETH_ENABLE)
endif
ifneq ($(ETH_ICMP_RESPONDER),)
VIVADO_RUN_ENV += ETH_ICMP_RESPONDER=$(ETH_ICMP_RESPONDER)
endif
ifneq ($(ETH_DEBUG_ENABLE),)
VIVADO_RUN_ENV += ETH_DEBUG_ENABLE=$(ETH_DEBUG_ENABLE)
endif
ifneq ($(ETH_RK5_DIR),)
VIVADO_RUN_ENV += ETH_RK5_DIR=$(ETH_RK5_DIR)
endif

# First-board 50 MHz preflight contract.  These are make variables so
# operators can print/override them, but the default target below stays
# pinned to the AN9134 200 MHz oscillator divided by four.
FPGA_50MHZ_TARGET_FREQ_MHZ ?= 50
ifeq ($(USE_REAL_MIG),)
FPGA_50MHZ_CORE_CLK_DIVIDE ?= 4
else
FPGA_50MHZ_CORE_CLK_DIVIDE ?= 2
endif
FPGA_50MHZ_CORE_CLK_HZ ?= 50_000_000
FPGA_50MHZ_PB_CLK_HZ ?= 50_000_000
# Video smoke was used for first-light HDMI bring-up and is dead weight
# (~10K LUTs) once CPU→VRAM scanout works.  Default OFF to match the
# 100 MHz preset and avoid the routing congestion that drove the
# WNS=-4.8ns cliff in the first 50 MHz impl attempt (2026-04-30).
# Override to 1 only if you specifically need the SMPTE-bar painter back.
FPGA_50MHZ_VIDEO_SMOKE ?= 0
FPGA_50MHZ_BOOT_ROM_SECTORS ?= 2048
FPGA_50MHZ_ENV := \
	TARGET_FREQ_MHZ=$(FPGA_50MHZ_TARGET_FREQ_MHZ) \
	CORE_CLK_DIVIDE=$(FPGA_50MHZ_CORE_CLK_DIVIDE) \
	CORE_CLK_HZ=$(FPGA_50MHZ_CORE_CLK_HZ) \
	PB_CLK_HZ=$(FPGA_50MHZ_PB_CLK_HZ) \
	VIDEO_SMOKE=$(FPGA_50MHZ_VIDEO_SMOKE) \
	BOOT_ROM_SECTORS=$(FPGA_50MHZ_BOOT_ROM_SECTORS)
FPGA_50MHZ_VIO_ENV := ENABLE_VIO=1 ENABLE_JTAG_AXI=1 $(FPGA_50MHZ_ENV)
FPGA_50MHZ_REAL_MIG_CORE_CLK_DIVIDE ?= 2
FPGA_50MHZ_REAL_MIG_ENV := \
	REAL_FPGA_BUILD=1 \
	TARGET_FREQ_MHZ=$(FPGA_50MHZ_TARGET_FREQ_MHZ) \
	CORE_CLK_DIVIDE=$(FPGA_50MHZ_REAL_MIG_CORE_CLK_DIVIDE) \
	CORE_CLK_HZ=$(FPGA_50MHZ_CORE_CLK_HZ) \
	PB_CLK_HZ=$(FPGA_50MHZ_PB_CLK_HZ) \
	VIDEO_SMOKE=$(FPGA_50MHZ_VIDEO_SMOKE) \
	BOOT_ROM_SECTORS=$(FPGA_50MHZ_BOOT_ROM_SECTORS)
FPGA_50MHZ_REAL_MIG_VIO_ENV := ENABLE_VIO=1 ENABLE_JTAG_AXI=1 $(FPGA_50MHZ_REAL_MIG_ENV)

# First-board 100 MHz contract (real MIG shell clock domain: 100 MHz,
# CORE_CLK_DIVIDE=1).  This is now the canonical hardware target.
# Default to the real video path, not the built-in HDMI color bars.
FPGA_100MHZ_TARGET_FREQ_MHZ ?= 100
FPGA_100MHZ_CORE_CLK_DIVIDE ?= 1
FPGA_100MHZ_CORE_CLK_HZ ?= 100_000_000
FPGA_100MHZ_PB_CLK_HZ ?= 50_000_000
# VIDEO_SMOKE writer is dead weight once the CPU→VRAM AXI path works.
# It compiled the SMPTE-bar reset-time framebuffer painter into the
# bitstream — useful for first-light HDMI bring-up, but now costs ~10k
# LUTs of FSM + word_data combinational mux.  Default off; flip to 1
# explicitly if a future bring-up wants the static rainbow back.
FPGA_100MHZ_VIDEO_SMOKE ?= 0
FPGA_100MHZ_BOOT_ROM_SECTORS ?= 2048
FPGA_100MHZ_ENV := \
	TARGET_FREQ_MHZ=$(FPGA_100MHZ_TARGET_FREQ_MHZ) \
	CORE_CLK_DIVIDE=$(FPGA_100MHZ_CORE_CLK_DIVIDE) \
	CORE_CLK_HZ=$(FPGA_100MHZ_CORE_CLK_HZ) \
	PB_CLK_HZ=$(FPGA_100MHZ_PB_CLK_HZ) \
	VIDEO_SMOKE=$(FPGA_100MHZ_VIDEO_SMOKE) \
	BOOT_ROM_SECTORS=$(FPGA_100MHZ_BOOT_ROM_SECTORS)
FPGA_100MHZ_REAL_MIG_ENV := REAL_FPGA_BUILD=1 $(FPGA_100MHZ_ENV)
FPGA_100MHZ_REAL_MIG_VIO_ENV := ENABLE_VIO=1 ENABLE_JTAG_AXI=1 $(FPGA_100MHZ_REAL_MIG_ENV)

# Keep the plain Vivado entry points aligned with the first-light preset:
# a bare TARGET_FREQ_MHZ=50 invocation should also drive the 50 MHz core
# generics unless the caller overrides them explicitly.
ifeq ($(TARGET_FREQ_MHZ),$(FPGA_50MHZ_TARGET_FREQ_MHZ))
ifeq ($(CORE_CLK_DIVIDE),)
VIVADO_RUN_ENV += CORE_CLK_DIVIDE=$(FPGA_50MHZ_CORE_CLK_DIVIDE)
endif
ifeq ($(CORE_CLK_HZ),)
VIVADO_RUN_ENV += CORE_CLK_HZ=$(FPGA_50MHZ_CORE_CLK_HZ)
endif
ifeq ($(PB_CLK_HZ),)
VIVADO_RUN_ENV += PB_CLK_HZ=$(FPGA_50MHZ_PB_CLK_HZ)
endif
ifeq ($(VIDEO_SMOKE),)
VIVADO_RUN_ENV += VIDEO_SMOKE=$(FPGA_50MHZ_VIDEO_SMOKE)
endif
ifeq ($(BOOT_ROM_SECTORS),)
VIVADO_RUN_ENV += BOOT_ROM_SECTORS=$(FPGA_50MHZ_BOOT_ROM_SECTORS)
endif
endif

# ──────────────────────────────────────────────────────────────────────────────
# Vivado mutex — only ONE synth/impl at a time machine-wide.
#
# KU5P synth uses ~12-15 GiB RSS + spawns 4 parallel_synth workers at
# ~3-5 GiB each — two concurrent runs OOM a 62 GiB machine and swap-
# thrashing makes each run 10× slower.
#
# flock(1) fails fast with -n — if the lock is held, the agent sees
# a non-zero exit immediately and can pivot to sim work instead of
# busy-waiting on a long-running Vivado.  DO NOT switch to a blocking
# flock; agents should NEVER wait for Vivado.
VIVADO_LOCK ?= /var/tmp/m68k-ooo-vivado.lock

.PHONY: ddr4-mig-validate
ddr4-mig-validate:
	@mkdir -p $(DDR4_MIG_DIR)
	@if python3 $(DDR4_MIG_CACHE_TOOL) check --mode validate --dir "$(DDR4_MIG_DIR)" --generator "$(DDR4_MIG_GEN_TCL)"; then \
	  :; \
	else \
	  status=$$?; \
	  if [ $$status -ne 1 ]; then exit $$status; fi; \
	  touch $(VIVADO_LOCK); \
	  flock -n -E 75 $(VIVADO_LOCK) -c '$(VIVADO) -mode batch -source $(DDR4_MIG_GEN_TCL) -tclargs validate $(DDR4_MIG_DIR)' \
	    || { status=$$?; if [ $$status -eq 75 ]; then echo "ERROR: another Vivado run holds $(VIVADO_LOCK) — DDR4 MIG validation pending"; fi; exit $$status; }; \
	  python3 $(DDR4_MIG_CACHE_TOOL) stamp --mode validate --dir "$(DDR4_MIG_DIR)" --generator "$(DDR4_MIG_GEN_TCL)"; \
	fi

.PHONY: ddr4-mig-cache-status
ddr4-mig-cache-status:
	@mkdir -p $(DDR4_MIG_DIR)
	@if python3 $(DDR4_MIG_CACHE_TOOL) check --mode synth --dir "$(DDR4_MIG_DIR)" --generator "$(DDR4_MIG_GEN_TCL)" --explain-miss; then \
	  :; \
	else \
	  status=$$?; \
	  if [ $$status -eq 1 ]; then \
	    echo "DDR4 MIG cache miss (synth): $(DDR4_MIG_DIR)"; \
	  fi; \
	  exit $$status; \
	fi

.PHONY: ddr4-mig
ddr4-mig:
	@mkdir -p $(DDR4_MIG_DIR)
	@if python3 $(DDR4_MIG_CACHE_TOOL) check --mode synth --dir "$(DDR4_MIG_DIR)" --generator "$(DDR4_MIG_GEN_TCL)"; then \
	  :; \
	else \
	  status=$$?; \
	  if [ $$status -ne 1 ]; then exit $$status; fi; \
	  touch $(VIVADO_LOCK); \
	  flock -n -E 75 $(VIVADO_LOCK) -c '$(VIVADO) -mode batch -source $(DDR4_MIG_GEN_TCL) -tclargs synth $(DDR4_MIG_DIR)' \
	    || { status=$$?; if [ $$status -eq 75 ]; then echo "ERROR: another Vivado run holds $(VIVADO_LOCK) — DDR4 MIG generation pending"; fi; exit $$status; }; \
	  python3 $(DDR4_MIG_CACHE_TOOL) stamp --mode synth --dir "$(DDR4_MIG_DIR)" --generator "$(DDR4_MIG_GEN_TCL)"; \
	fi

.PHONY: pcie-xdma-validate
pcie-xdma-validate:
	@mkdir -p $(PCIE_XDMA_DIR)
	@touch $(VIVADO_LOCK)
	flock -n -E 75 $(VIVADO_LOCK) -c 'PCIE_TEST_DIR=$(PCIE_TEST_DIR) $(VIVADO) -mode batch -source $(SYNTH_DIR)/gen_pcie_xdma.tcl -tclargs validate $(PCIE_XDMA_DIR)' \
	  || { status=$$?; if [ $$status -eq 75 ]; then echo "ERROR: another Vivado run holds $(VIVADO_LOCK) — PCIe/XDMA validation pending"; fi; exit $$status; }
	python3 $(TOOLS_DIR)/check_pcie_xdma_ip.py --dir $(PCIE_XDMA_DIR)

.PHONY: pcie-xdma
pcie-xdma:
	@mkdir -p $(PCIE_XDMA_DIR)
	@touch $(VIVADO_LOCK)
	flock -n -E 75 $(VIVADO_LOCK) -c 'PCIE_TEST_DIR=$(PCIE_TEST_DIR) $(VIVADO) -mode batch -source $(SYNTH_DIR)/gen_pcie_xdma.tcl -tclargs synth $(PCIE_XDMA_DIR)' \
	  || { status=$$?; if [ $$status -eq 75 ]; then echo "ERROR: another Vivado run holds $(VIVADO_LOCK) — PCIe/XDMA generation pending"; fi; exit $$status; }
	python3 $(TOOLS_DIR)/check_pcie_xdma_ip.py --dir $(PCIE_XDMA_DIR)

.PHONY: synth
.PHONY: check-synth-sources
# Fail if an RTL file is invisible to synth/vivado.tcl's EXPLICIT
# read_verilog list.  The Makefile finds sources with `find`, so a module
# can pass lint AND every unit tb and still be absent from the bitstream.
# That happened with vhdd_ctrl/vhdd_mux: synthesis died at
# "module 'vhdd_ctrl' not found" after they were fully verified.
check-synth-sources:
	@python3 $(CURDIR)/tools/check_synth_sources.py

.PHONY: check-storage-reset-pairing
# Fail if sd_ctrl can be reset without also resetting sd_scsi_bridge.  The
# bridge's only escape from a transfer killed mid-flight is its peer-reset
# rescue, armed by a RISING EDGE on core_rst; reset sd_ctrl without that edge
# and pb_busy_internal latches FOREVER, wedging the CPU in the ROM's blind
# pseudo-DMA burst.  Fixed here by 56283b91 -- but the same wiring was still
# WRONG on other branches months later, because this is a structural bug that
# no unit tb can see.  Gate the build on it so a branch cannot regress it.
check-storage-reset-pairing:
	@python3 $(CURDIR)/tools/check_storage_reset_pairing.py

synth: check-synth-sources check-storage-reset-pairing
	@mkdir -p $(VIVADO_IMPL_DIR)
	@touch $(VIVADO_LOCK)
	flock -n -E 75 $(VIVADO_LOCK) -c '$(VIVADO_RUN_ENV) $(VIVADO) -mode batch -source $(SYNTH_DIR)/vivado.tcl -tclargs synth_only $(VIVADO_IMPL_DIR)' \
	  || { status=$$?; if [ $$status -eq 75 ]; then echo "ERROR: another Vivado run holds $(VIVADO_LOCK) — fall back to sim work"; fi; exit $$status; }

.PHONY: impl
impl: check-synth-sources
	@mkdir -p $(VIVADO_IMPL_DIR)
	@touch $(VIVADO_LOCK)
	flock -n -E 75 $(VIVADO_LOCK) -c '$(VIVADO_RUN_ENV) $(VIVADO) -mode batch -source $(SYNTH_DIR)/vivado.tcl -tclargs full_impl $(VIVADO_IMPL_DIR)' \
	  || { status=$$?; if [ $$status -eq 75 ]; then echo "ERROR: another Vivado run holds $(VIVADO_LOCK) — fall back to sim work"; fi; exit $$status; }

.PHONY: incremental-status
incremental-status:
	@ref="$(INCREMENTAL_REF_DCP)"; \
	  echo "INCREMENTAL_REF_DCP = $$ref"; \
	  if [ -f "$$ref" ]; then \
	    sz=$$(stat -c %s "$$ref" 2>/dev/null || stat -f %z "$$ref"); \
	    mt=$$(stat -c %y "$$ref" 2>/dev/null || stat -f "%Sm" "$$ref"); \
	    echo "  exists: yes (bytes=$$sz, mtime=$$mt)"; \
	    rep="$(VIVADO_IMPL_DIR)/reports/incremental_reuse.rpt"; \
	    if [ -f "$$rep" ]; then \
	      echo "  last reuse report: $$rep"; \
	      grep -E '^(\| (Reuse|Cell|Net) Type|Overall Reuse)' "$$rep" 2>/dev/null | head -20; \
	    fi; \
	  else \
	    echo "  exists: no — first impl will run full and stash on success"; \
	  fi

.PHONY: clean-incremental
clean-incremental:
	@rm -f "$(INCREMENTAL_REF_DCP)"
	@echo "removed incremental reference: $(INCREMENTAL_REF_DCP)"

.PHONY: verify-fpga-debug-artifacts
verify-fpga-debug-artifacts:
	@bit="$(VIVADO_IMPL_DIR)/fpga_top.blank.bit"; \
	ltx="$(VIVADO_IMPL_DIR)/fpga_top.ltx"; \
	info="$(FPGA_BUILDINFO)"; \
	[ -f "$$bit" ] || { echo "ERROR: missing bitstream: $$bit"; exit 1; }; \
	[ -f "$$ltx" ] || { echo "ERROR: missing debug probes: $$ltx"; exit 1; }; \
	[ -f "$$info" ] || { echo "ERROR: missing build manifest: $$info"; exit 1; }; \
	grep -qx 'enable_vio=1' "$$info" || { echo "ERROR: build manifest says ENABLE_VIO was off: $$info"; exit 1; }; \
	grep -Eq '^host_debug=(jtag_axi|pcie_xdma)$$' "$$info" || { echo "ERROR: build manifest says no supported host debug path was enabled: $$info"; exit 1; }; \
	echo "Verified debug-capable FPGA artifacts:"; \
	echo "  bitstream: $$bit"; \
	echo "  probes:    $$ltx"; \
	echo "  manifest:  $$info"

.PHONY: fpga-50mhz-jtag-bitstream
fpga-50mhz-jtag-bitstream:
	$(MAKE) fpga-50mhz-jtag-bitstream-dram

.PHONY: fpga-50mhz-jtag-bitstream-dram
fpga-50mhz-jtag-bitstream-dram:
	$(MAKE) ddr4-mig
	$(MAKE) USE_REAL_MIG=1 $(FPGA_50MHZ_REAL_MIG_VIO_ENV) impl
	$(MAKE) USE_REAL_MIG=1 $(FPGA_50MHZ_REAL_MIG_VIO_ENV) verify-fpga-debug-artifacts

.PHONY: fpga-sdmin-bitstream
# Standalone SD-card + CRC16 test harness (rtl/soc/fpga_top_sdmin.v) — no
# DDR4 MIG, no CPU, no HDMI. Built for fast iteration on the 2026-07-23
# CMD18/CRC16 boot-blocker investigation: the full fpga_top build takes
# 35-45 min/cycle; this has nothing to place/route beyond boot_fsm +
# sd_ctrl + sd_spi + a tiny VIO core, so it should complete in a few
# minutes. VIO probe_out0 bits (cfg_skip_cmd59, cfg_force_hs, soft-reset)
# let ONE flashed bitstream test multiple init-sequence hypotheses
# without a resynth between experiments — see boot_fsm.v's port comment.
SDMIN_BUILD_DIR ?= $(BUILD_DIR)/vivado_sdmin
fpga-sdmin-bitstream:
	@mkdir -p $(SDMIN_BUILD_DIR)
	@touch $(VIVADO_LOCK)
	flock -n -E 75 $(VIVADO_LOCK) -c '$(VIVADO) -mode batch -source $(SYNTH_DIR)/vivado_sdmin.tcl -tclargs $(SDMIN_BUILD_DIR)' \
	  || { status=$$?; if [ $$status -eq 75 ]; then echo "ERROR: another Vivado run holds $(VIVADO_LOCK) — fall back to sim work"; fi; exit $$status; }

.PHONY: fpga-100mhz-jtag-bitstream
fpga-100mhz-jtag-bitstream:
	$(MAKE) fpga-100mhz-jtag-bitstream-dram

.PHONY: fpga-100mhz-jtag-bitstream-dram
fpga-100mhz-jtag-bitstream-dram:
	$(MAKE) ddr4-mig
	$(MAKE) USE_REAL_MIG=1 $(FPGA_100MHZ_REAL_MIG_VIO_ENV) impl
	$(MAKE) USE_REAL_MIG=1 $(FPGA_100MHZ_REAL_MIG_VIO_ENV) verify-fpga-debug-artifacts

.PHONY: pcie-xdma-dry-run
pcie-xdma-dry-run: pcie-xdma-validate
	$(MAKE) USE_REAL_MIG=1 ENABLE_PCIE_XDMA=1 vivado-dry-run

.PHONY: pcie-xdma-pincheck
pcie-xdma-pincheck: pcie-xdma-validate
	$(MAKE) USE_REAL_MIG=1 ENABLE_PCIE_XDMA=1 ddr-pincheck

.PHONY: pcie-xdma-bitstream
pcie-xdma-bitstream:
	$(MAKE) ddr4-mig
	$(MAKE) pcie-xdma
	$(MAKE) USE_REAL_MIG=1 ENABLE_VIO=1 ENABLE_PCIE_XDMA=1 $(FPGA_100MHZ_REAL_MIG_ENV) impl
	$(MAKE) USE_REAL_MIG=1 ENABLE_VIO=1 ENABLE_PCIE_XDMA=1 $(FPGA_100MHZ_REAL_MIG_ENV) verify-fpga-debug-artifacts

.PHONY: jtag-discover
jtag-discover:
	$(VIVADO) -nojournal -nolog -mode batch -source $(SYNTH_DIR)/hw_discover.tcl

.PHONY: jtag-dashboard
jtag-dashboard:
	$(VIVADO) -nojournal -nolog -mode batch -source $(SYNTH_DIR)/jtag_bringup.tcl -tclargs dashboard

.PHONY: jtag-status
jtag-status:
	python3 tools/jtag_bringup_tui.py status

.PHONY: jtag-tui
jtag-tui:
	python3 tools/jtag_bringup_tui.py dashboard

.PHONY: ddr-pincheck
ddr-pincheck:
	@mkdir -p $(VIVADO_IMPL_DIR)
	@touch $(VIVADO_LOCK)
	flock -n -E 75 $(VIVADO_LOCK) -c '$(VIVADO_RUN_ENV) USE_REAL_MIG=1 $(VIVADO) -mode batch -source $(SYNTH_DIR)/vivado.tcl -tclargs ddr_pincheck $(VIVADO_IMPL_DIR)' \
	  || { status=$$?; if [ $$status -eq 75 ]; then echo "ERROR: another Vivado run holds $(VIVADO_LOCK) — fall back to sim work"; fi; exit $$status; }

.PHONY: ddr-pcie-test-check
ddr-pcie-test-check:
	python3 $(TOOLS_DIR)/check_ddr4_pcie_test.py --repo $(PROJ_ROOT) --pcie-test $(PCIE_TEST_DIR)

.PHONY: pcie-checkpoint-dump-check
pcie-checkpoint-dump-check:
	python3 $(TOOLS_DIR)/check_pcie_xdma_dump.py --chunk-bytes 4194304

FACTORY_IMAGE_KU5P_DIR ?= $(HOME)/FPGA/9.RK-XCKU5P-F/5_FactoryData/image_ku5p

.PHONY: ddr-reference-check
ddr-reference-check:
	python3 $(TOOLS_DIR)/check_ddr4_pcie_test.py --repo $(PROJ_ROOT) --pcie-test $(PCIE_TEST_DIR) --factory-image $(FACTORY_IMAGE_KU5P_DIR)

.PHONY: vivado-dry-run
vivado-dry-run:
	@mkdir -p $(VIVADO_IMPL_DIR)
	@touch $(VIVADO_LOCK)
	flock -n -E 75 $(VIVADO_LOCK) -c '$(VIVADO_RUN_ENV) $(VIVADO) -mode batch -source $(SYNTH_DIR)/vivado.tcl -tclargs dry_run $(VIVADO_IMPL_DIR)' \
	  || { status=$$?; if [ $$status -eq 75 ]; then echo "ERROR: another Vivado run holds $(VIVADO_LOCK) — fall back to sim work"; fi; exit $$status; }

# Phase-1 Ethernet integration preflight: imports the pinned rk5-eth Taxi
# source lists and RGMII constraints, with the known-good ICMP responder as
# the endpoint.  A successful dry run is the prerequisite to a full hardware
# image (`make ETH_ENABLE=1 impl`).
.PHONY: eth-icmp-dry-run
eth-icmp-dry-run:
	$(MAKE) ETH_ENABLE=1 ETH_ICMP_RESPONDER=1 vivado-dry-run

.PHONY: clock-report
clock-report:
	@mkdir -p $(VIVADO_IMPL_DIR)
	@touch $(VIVADO_LOCK)
	flock -n -E 75 $(VIVADO_LOCK) -c '$(VIVADO_RUN_ENV) $(VIVADO) -mode batch -source $(SYNTH_DIR)/vivado.tcl -tclargs clock_report $(VIVADO_IMPL_DIR)' \
	  || { status=$$?; if [ $$status -eq 75 ]; then echo "ERROR: another Vivado run holds $(VIVADO_LOCK) — fall back to sim work"; fi; exit $$status; }

.PHONY: fpga-first-hw-offline-preflight
fpga-first-hw-offline-preflight:
	@echo "=== FPGA first-hardware offline preflight: no Vivado/netlist/impl/bitstream ==="
	$(MAKE) ddr-pcie-test-check
	$(MAKE) lint-fpga-top
	$(MAKE) tb-axi-ddr-contract
	$(MAKE) tb-axi-ddr4-mig-bridge
	$(MAKE) CLK_DIVIDE=4 tb-clk-rst
	$(MAKE) CLK_DIVIDE=2 tb-clk-rst
	$(MAKE) firstlight-framebuffer-preflight

.PHONY: fpga-50mhz-preflight
fpga-50mhz-preflight: fpga-first-hw-offline-preflight
	@echo "=== FPGA 50 MHz Vivado preflight: RTL elaboration/constraints only ==="
	$(MAKE) ddr4-mig-validate
	$(MAKE) $(FPGA_50MHZ_ENV) clock-report
	$(MAKE) $(FPGA_50MHZ_REAL_MIG_ENV) ddr-pincheck

.PHONY: fpga-100mhz-preflight
fpga-100mhz-preflight: fpga-first-hw-offline-preflight
	@echo "=== FPGA 100 MHz Vivado preflight: RTL elaboration/constraints only ==="
	$(MAKE) ddr4-mig-validate
	$(MAKE) $(FPGA_100MHZ_ENV) clock-report
	$(MAKE) $(FPGA_100MHZ_REAL_MIG_ENV) ddr-pincheck

.PHONY: fpga-first-hw-preflight
fpga-first-hw-preflight: fpga-100mhz-preflight
	@echo "=== FPGA first-hardware preflight complete ==="

.PHONY: timing
timing:
	@if [ -f $(VIVADO_IMPL_DIR)/timing_summary.rpt ]; then \
		grep -A 5 "Design Timing Summary" $(VIVADO_IMPL_DIR)/timing_summary.rpt; \
		grep "WNS" $(VIVADO_IMPL_DIR)/timing_summary.rpt | head -5; \
	else \
		echo "No timing report found. Run 'make impl' first."; \
	fi

TIMING_AUDIT_REPORT ?= $(VIVADO_IMPL_DIR)/reports/timing_synth.rpt

.PHONY: timing-audit
timing-audit:
	python3 $(TOOLS_DIR)/vivado_timing_audit.py $(TIMING_AUDIT_REPORT)

.PHONY: gui
gui:
ifdef STEP
	$(VIVADO) $(VIVADO_IMPL_DIR)/checkpoints/$(STEP).dcp &
else
	$(VIVADO) $(VIVADO_IMPL_DIR)/checkpoints/route.dcp &
endif

# ras_sim / ras_test (rtl/core/fetch/ras.v) removed: ras.v moved to the
# cpu/ submodule in the SoC split — `cd cpu && make ras_test` instead.

# ──────────────────────────────────────────────────────────────────────────────
# DDR controller SIM_MODEL unit testbench
#
# Exercises rtl/board/ddr_ctrl.v against tb/tb_ddr_model.v (thin wrapper that
# pins the SIM_MODEL parameters for fast simulation).  Standalone from the
# main `sim` target — ddr_ctrl has no core dependencies.
# ──────────────────────────────────────────────────────────────────────────────
DDR_RTL   := \
	$(RTL_DIR)/board/ddr_ctrl.v \
	$(TB_DIR)/tb_ddr_model.v
DDR_BUILD := $(BUILD_DIR)/ddr_model

.PHONY: tb-ddr-model
tb-ddr-model: $(DDR_BUILD)/Vtb_ddr_model
	@echo "Running ddr_ctrl SIM_MODEL unit tb..."
	$(DDR_BUILD)/Vtb_ddr_model

$(DDR_BUILD)/Vtb_ddr_model: $(DDR_RTL) $(TB_DIR)/tb_ddr_model.cpp
	@mkdir -p $(DDR_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		+define+SIM_MODEL \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(DDR_BUILD) \
		--top-module tb_ddr_model \
		$(DDR_RTL) \
		$(TB_DIR)/tb_ddr_model.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# 32-bit AXI to DDR contract unit testbench
#
# Exercises the first-light memory path used by boot_fsm/core word accesses:
# axi_narrow_to_wide feeding ddr_ctrl SIM_MODEL.  This is intentionally
# narrower than fpga_top and avoids core/SD/HDMI/peripheral dependencies.
# ──────────────────────────────────────────────────────────────────────────────
AXI_DDR_CONTRACT_RTL := \
	$(RTL_DIR)/soc/axi_narrow_to_wide.v \
	$(RTL_DIR)/board/ddr_ctrl.v \
	$(TB_DIR)/tb_axi_ddr_contract.v
AXI_DDR_CONTRACT_BUILD := $(BUILD_DIR)/axi_ddr_contract

.PHONY: tb-axi-ddr-contract
tb-axi-ddr-contract: $(AXI_DDR_CONTRACT_BUILD)/Vtb_axi_ddr_contract
	@echo "Running 32-bit AXI to DDR contract tb..."
	$(AXI_DDR_CONTRACT_BUILD)/Vtb_axi_ddr_contract

$(AXI_DDR_CONTRACT_BUILD)/Vtb_axi_ddr_contract: $(AXI_DDR_CONTRACT_RTL) $(TB_DIR)/tb_axi_ddr_contract.cpp
	@mkdir -p $(AXI_DDR_CONTRACT_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		+define+SIM_MODEL \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(AXI_DDR_CONTRACT_BUILD) \
		--top-module tb_axi_ddr_contract \
		$(AXI_DDR_CONTRACT_RTL) \
		$(TB_DIR)/tb_axi_ddr_contract.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# DDR4 pcie_test MIG bridge contract unit testbench
#
# Exercises the standalone 128-bit/6-ID/32-address repo DDR AXI to
# 256-bit/1-ID/31-address known-good pcie_test MIG AXI contract shim.
# This is wired into ddr_ctrl's real-MIG path and prevents silent contract
# drift before the Vivado IP integration step.
# ──────────────────────────────────────────────────────────────────────────────
AXI_DDR4_MIG_BRIDGE_RTL := \
	$(RTL_DIR)/board/axi_ddr4_mig_bridge.v
AXI_DDR4_MIG_BRIDGE_BUILD := $(BUILD_DIR)/axi_ddr4_mig_bridge

.PHONY: tb-axi-ddr4-mig-bridge
tb-axi-ddr4-mig-bridge: $(AXI_DDR4_MIG_BRIDGE_BUILD)/Vaxi_ddr4_mig_bridge
	@echo "Running DDR4 pcie_test MIG bridge contract tb..."
	@# axi_ddr4_mig_bridge.v carries sim-only self-checks (the cwd_* shadow
	@# coherence controls, the dual-writer agreement check, and the AW
	@# protocol monitor) that report via $$display.  The C++ scenarios do
	@# NOT watch for those strings, so before this gate an assertion could
	@# fire on every cycle and the tb would still print "All N scenarios
	@# PASSED".  Fail the target if any of them fires.
	@set -o pipefail; \
	  $(AXI_DDR4_MIG_BRIDGE_BUILD)/Vaxi_ddr4_mig_bridge 2>&1 | tee $(AXI_DDR4_MIG_BRIDGE_BUILD)/run.log; \
	  rc=$$?; \
	  if grep -q 'ASSERTION FAIL' $(AXI_DDR4_MIG_BRIDGE_BUILD)/run.log; then \
	    echo "FAIL: RTL sim-only self-check fired (see 'ASSERTION FAIL' above)"; \
	    exit 1; \
	  fi; \
	  exit $$rc

$(AXI_DDR4_MIG_BRIDGE_BUILD)/Vaxi_ddr4_mig_bridge: $(AXI_DDR4_MIG_BRIDGE_RTL) $(TB_DIR)/tb_axi_ddr4_mig_bridge.cpp
	@mkdir -p $(AXI_DDR4_MIG_BRIDGE_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(AXI_DDR4_MIG_BRIDGE_BUILD) \
		--top-module axi_ddr4_mig_bridge \
		$(AXI_DDR4_MIG_BRIDGE_RTL) \
		$(TB_DIR)/tb_axi_ddr4_mig_bridge.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# clk_rst unit testbench (board clock divider + reset-release helper)
#
# Standalone Verilator build — clk_rst.v plus Xilinx primitive stubs and
# a tiny wrapper.  Default CLK_DIVIDE=4 exercises the first-board 50 MHz
# path on a 200 MHz input; override with `make tb-clk-rst CLK_DIVIDE=2`
# to sanity-check the 100 MHz variant.
# ──────────────────────────────────────────────────────────────────────────────
ifdef CORE_CLK_DIVIDE
CLK_DIVIDE ?= $(CORE_CLK_DIVIDE)
else
CLK_DIVIDE ?= 4
endif
CLK_RST_RTL   := \
	$(RTL_DIR)/board/clk_rst.v \
	$(TB_DIR)/tb_clk_rst.v \
	$(TB_DIR)/verilator_xilinx_stubs.v
CLK_RST_BUILD := $(BUILD_DIR)/clk_rst_$(CLK_DIVIDE)

.PHONY: tb-clk-rst
tb-clk-rst: $(CLK_RST_BUILD)/Vtb_clk_rst
	@echo "Running clk_rst unit tb (CLK_DIVIDE=$(CLK_DIVIDE))..."
	$(CLK_RST_BUILD)/Vtb_clk_rst

$(CLK_RST_BUILD)/Vtb_clk_rst: $(CLK_RST_RTL) $(TB_DIR)/tb_clk_rst.cpp
	@mkdir -p $(CLK_RST_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) -I$(TB_DIR) \
		-GCLK_DIVIDE=$(CLK_DIVIDE) \
		-Mdir $(CLK_RST_BUILD) \
		--top-module tb_clk_rst \
		$(CLK_RST_RTL) \
		$(TB_DIR)/tb_clk_rst.cpp \
		-CFLAGS "-std=c++17 -DTB_CLK_DIVIDE=$(CLK_DIVIDE)"

# ──────────────────────────────────────────────────────────────────────────────
# VIA2 unit testbench (6522 Quadra 700 variant)
#
# Standalone Verilator build — via2.v has no dependencies on the core.
# Exercises reset reads, ORA/ORB default = 0xFF, IFR always 0, and
# register round-trip for a normal register.
# ──────────────────────────────────────────────────────────────────────────────
VIA2_RTL   := $(RTL_DIR)/mac/via2.v
VIA2_BUILD := $(BUILD_DIR)/via2

.PHONY: tb-via
tb-via: tb-via1 tb-via2

.PHONY: tb-via2
tb-via2: $(VIA2_BUILD)/Vvia2
	@echo "Running via2 unit tb..."
	$(VIA2_BUILD)/Vvia2

$(VIA2_BUILD)/Vvia2: $(VIA2_RTL) $(TB_DIR)/tb_via2.cpp
	@mkdir -p $(VIA2_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(VIA2_BUILD) \
		--top-module via2 \
		$(VIA2_RTL) \
		$(TB_DIR)/tb_via2.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# VIA tick-rate gate — the Mac 60 Hz tick, measured end to end
#
# Rebuilds the chain fpga_top_peripherals.vh actually wires today:
#   pb_clk -> phi2 NCO (verbatim from fpga_top_clocks.vh) -> VIA2 T1
#   free-run/PB7 -> via1_ca1_in -> VIA1 IFR.CA1 -> via1_irq.
# Programs VIA2 with the values the Q700 ROM writes and asserts the
# resulting interrupt RATE in Hz, two-sided.  Neither tb-via2 (toggle
# only) nor tb-vbl-rate (models the superseded dafb_vbl_level chain) can
# see a wrong tick rate.
# ──────────────────────────────────────────────────────────────────────────────
VIA_TICK_RATE_RTL   := $(RTL_DIR)/mac/via1.v $(RTL_DIR)/mac/via2.v \
                       $(TB_DIR)/tb_via_tick_rate.v
VIA_TICK_RATE_BUILD := $(BUILD_DIR)/via_tick_rate

.PHONY: tb-via-tick-rate
tb-via-tick-rate: $(VIA_TICK_RATE_BUILD)/Vtb_via_tick_rate
	@echo "Running VIA 60 Hz tick-rate gate (VIA2 T1 -> PB7 -> VIA1 CA1)..."
	$(VIA_TICK_RATE_BUILD)/Vtb_via_tick_rate

$(VIA_TICK_RATE_BUILD)/Vtb_via_tick_rate: $(VIA_TICK_RATE_RTL) $(TB_DIR)/tb_via_tick_rate.cpp
	@mkdir -p $(VIA_TICK_RATE_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(VIA_TICK_RATE_BUILD) \
		--top-module tb_via_tick_rate \
		$(VIA_TICK_RATE_RTL) \
		$(TB_DIR)/tb_via_tick_rate.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SCC unit testbench (Z85C30 — dual-channel async serial)
#
# Standalone Verilator build — scc.v has no dependencies on the core.
# The module's BRG pre-scaler default is 54 (tuned for 200 MHz clk →
# 3.672 MHz PCLK); the tb overrides to PCLK_DIV=2 so byte-times elapse
# in a handful of simulation cycles.
# ──────────────────────────────────────────────────────────────────────────────
SCC_RTL   := $(RTL_DIR)/mac/scc.v
SCC_BUILD := $(BUILD_DIR)/scc

.PHONY: tb-scc
tb-scc: $(SCC_BUILD)/Vscc
	@echo "Running scc unit tb..."
	$(SCC_BUILD)/Vscc

$(SCC_BUILD)/Vscc: $(SCC_RTL) $(TB_DIR)/tb_scc.cpp
	@mkdir -p $(SCC_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-GPCLK_DIV=2 \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(SCC_BUILD) \
		--top-module scc \
		$(SCC_RTL) \
		$(TB_DIR)/tb_scc.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# Peripheral-bus unit testbench (rtl/soc/peripheral_bus.v)
#
# Standalone Verilator build — peripheral_bus.v only.  Drives the single
# AXI4 slave port from a tb master and asserts on the 9 downstream faces
# (VIA1, VIA2, Ethernet ID, SONIC, SCC, SCSI, ASC pb_*; debug_ctrl
# AXI-Lite).
#
# PB_WATCHDOG_LOG2 is overridden down to 10 (2^10 = 1024-cycle ack
# timeout, production default is 24 = ~335 ms @ 50 MHz) so the watchdog
# scenarios in tb_peripheral_bus.cpp run in ~1k cycles instead of 16M.
# Keep the tb's WD_TIMEOUT constant in sync with this value.
# ──────────────────────────────────────────────────────────────────────────────
PB_RTL   := $(RTL_DIR)/soc/peripheral_bus.v
PB_BUILD := $(BUILD_DIR)/peripheral_bus
MAME_AXI_PERIPH_RTL := $(RTL_DIR)/soc/axi_xbar.v \
	$(RTL_DIR)/soc/axi_wide_to_axilite.v \
	$(RTL_DIR)/soc/peripheral_bus.v \
	$(RTL_DIR)/mac/via1.v \
	$(RTL_DIR)/mac/via2.v \
	$(RTL_DIR)/mac/rtc.v \
	$(RTL_DIR)/mac/scc.v \
	$(RTL_DIR)/mac/scsi.v \
	$(RTL_DIR)/mac/asc.v \
	$(RTL_DIR)/mac/video.v \
	$(RTL_DIR)/mac/q700_eth_sonic.v \
	$(RTL_DIR)/mac/orwell_stub.v \
	$(RTL_DIR)/mac/iwm_stub.v \
	$(RTL_DIR)/soc/sd_scsi_lba_mapper.v \
	$(RTL_DIR)/soc/vhdd_sd.v \
	$(TB_DIR)/mame_axi_periph_top.v
MAME_AXI_PERIPH_BUILD := $(BUILD_DIR)/mame_axi_periph_bridge
MAME_PATCH_ROM := $(BUILD_DIR)/mame_patch_rom
MAME_RTL_BIN ?= /tmp/mame/macrtl
MAME_ROMPATH ?= $(BUILD_DIR)/mame_roms
MAME_ADB_ROM ?= $(PROJ_ROOT)/files/342s0440-b.bin
MAME_PLATFORM_LOCKSTEP_DIR ?= $(BUILD_DIR)/mame_runs/platform_lockstep
MAME_PLATFORM_LOCKSTEP_SOCKET ?= $(MAME_PLATFORM_LOCKSTEP_DIR)/bridge.sock
MAME_PLATFORM_LOCKSTEP_SECONDS ?= 30
MAME_PLATFORM_LOCKSTEP_LABELS ?= VIA1,VIA2,ENET,SONIC,SCC,ORWELL,SCSI,SCSI DMA,ASC,SWIM,DAFB,UNMODELED_IO,UNMODELED_VIDEO
MAME_PLATFORM_LOCKSTEP_PATCH ?=
MAME_PLATFORM_LOCKSTEP_ACCEPT_VALIDATED_TIMING ?= 1
MAME_PLATFORM_LOCKSTEP_RTC_DATE ?= 2026-04-26T00:00:00
MAME_PLATFORM_LOCKSTEP_RTC_SECONDS ?= 3860006400
MAME_PLATFORM_LOCKSTEP_MAX_CPU_ADVANCE ?= 1000000
MAME_PLATFORM_LOCKSTEP_BRIDGE_PLUSARGS ?= +rtc_mame_state +rtc_init_seconds=$(MAME_PLATFORM_LOCKSTEP_RTC_SECONDS)
MAME_PLATFORM_LOCKSTEP_ROM := $(MAME_PLATFORM_LOCKSTEP_DIR)/roms/macqd700/420dbff3.rom
MAME_PLATFORM_LOCKSTEP_ACTIVE_ROMPATH := $(if $(strip $(MAME_PLATFORM_LOCKSTEP_PATCH)),$(MAME_PLATFORM_LOCKSTEP_DIR)/roms,$(MAME_ROMPATH))

.PHONY: tb-peripheral-bus
tb-peripheral-bus: $(PB_BUILD)/Vperipheral_bus
	@echo "Running peripheral_bus unit tb..."
	$(PB_BUILD)/Vperipheral_bus

$(PB_BUILD)/Vperipheral_bus: $(PB_RTL) $(TB_DIR)/tb_peripheral_bus.cpp
	@mkdir -p $(PB_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(PB_BUILD) \
		--top-module peripheral_bus \
		-GPB_WATCHDOG_LOG2=10 \
		-GENABLE_ACK_WATCHDOG=1 \
		$(PB_RTL) \
		$(TB_DIR)/tb_peripheral_bus.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# tb-peripheral-bus-prod -- the SAME tb_peripheral_bus.cpp, rebuilt at the
# PRODUCTION peripheral_bus parameterisation: ENABLE_ACK_WATCHDOG=0.
#
# WHY IT EXISTS.  tb-peripheral-bus above (and tb-pb-scsi) both build with
# ENABLE_ACK_WATCHDOG=1, the OPPOSITE of what fpga_top_peripherals.vh
# instantiates.  With the watchdog on, a pb_* strobe that is physically
# lost still "completes" -- as an SLVERR, PB_ACK_TIMEOUT cycles later,
# with the bytes never delivered.  That is how a permanently-dead S1
# slave hid behind a green suite.  This build removes the safety net so a
# lost strobe presents the way it does on silicon: as a hang.
#
# It runs the peripheral-reset-barrier scenarios and skips the three
# watchdog scenarios (which only exist when the watchdog is on) via
# -DPB_PROD_WATCHDOG_OFF.  PB_WATCHDOG_LOG2 is still overridden down so a
# regression that DOES arm a counter cannot cost 16 M cycles.
# ──────────────────────────────────────────────────────────────────────────────
PB_PROD_BUILD := $(BUILD_DIR)/peripheral_bus_prod

.PHONY: tb-peripheral-bus-prod
tb-peripheral-bus-prod: $(PB_PROD_BUILD)/Vperipheral_bus
	@echo "Running peripheral_bus unit tb (PRODUCTION ENABLE_ACK_WATCHDOG=0)..."
	$(PB_PROD_BUILD)/Vperipheral_bus

$(PB_PROD_BUILD)/Vperipheral_bus: $(PB_RTL) $(TB_DIR)/tb_peripheral_bus.cpp
	@mkdir -p $(PB_PROD_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(PB_PROD_BUILD) \
		--top-module peripheral_bus \
		-GPB_WATCHDOG_LOG2=10 \
		-GENABLE_ACK_WATCHDOG=0 \
		$(PB_RTL) \
		$(TB_DIR)/tb_peripheral_bus.cpp \
		-CFLAGS "-std=c++17 -DPB_PROD_WATCHDOG_OFF"

.PHONY: mame-axi-periph-bridge
mame-axi-periph-bridge: $(MAME_AXI_PERIPH_BUILD)/Vmame_axi_periph_top
	@echo "MAME AXI peripheral RTL bridge: $(MAME_AXI_PERIPH_BUILD)/Vmame_axi_periph_top"

.PHONY: mame-axi-periph-bridge-selftest
mame-axi-periph-bridge-selftest: $(MAME_AXI_PERIPH_BUILD)/Vmame_axi_periph_top
	$(MAME_AXI_PERIPH_BUILD)/Vmame_axi_periph_top --selftest

$(MAME_AXI_PERIPH_BUILD)/Vmame_axi_periph_top: $(MAME_AXI_PERIPH_RTL) $(TOOLS_DIR)/mame_axi_periph_bridge.cpp
	@mkdir -p $(MAME_AXI_PERIPH_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-j $(VERILATOR_JOBS) --threads $(VERILATOR_THREADS) \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED \
		-I$(RTL_DIR)/mac -I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(MAME_AXI_PERIPH_BUILD) \
		--top-module mame_axi_periph_top \
		$(MAME_AXI_PERIPH_RTL) \
		$(TOOLS_DIR)/mame_axi_periph_bridge.cpp \
		-CFLAGS "-std=c++17"

.PHONY: mame-patch-rom
mame-patch-rom: $(MAME_PATCH_ROM)
	@echo "MAME ROM patch tool: $(MAME_PATCH_ROM)"

$(MAME_PATCH_ROM): $(TOOLS_DIR)/mame_patch_rom.cpp $(TB_DIR)/models/rom_patch_sets.h
	@mkdir -p $(dir $@)
	$(CXX) -std=c++17 -Wall -Wextra -I$(TB_DIR)/models \
		$(TOOLS_DIR)/mame_patch_rom.cpp -o $@

# ──────────────────────────────────────────────────────────────────────────────
# Core-track unit tbs (LSU, dcache/SMC, icache, if_stage, reset-vectors,
# dcache-burst, MMU + walker, BPU, ALU, FPU, FP RAT — tb-lsu, tb-mac-top-smc,
# tb-icache, tb-if-stage, tb-reset-vectors, tb-dcache, tb-dcache-burst,
# tb-mmu, tb-mmu-walker(-boot), tb-bpu, tb-alu, tb-fpu, tb-fp-rat) were
# removed from this Makefile: they all verilated rtl/core/* files that
# moved to the cpu/ git submodule in the SoC split and no longer exist
# here.  They are still live and passing in cpu/Makefile — `cd cpu &&
# make tb-lsu` etc.
# ──────────────────────────────────────────────────────────────────────────────

# ──────────────────────────────────────────────────────────────────────────────
# SCSI (NCR 5380) unit testbench — minimum-viable stub
# ──────────────────────────────────────────────────────────────────────────────
SCSI_RTL   := $(RTL_DIR)/mac/scsi.v \
	$(RTL_DIR)/soc/sd_scsi_lba_mapper.v \
	$(RTL_DIR)/soc/vhdd_sd.v \
	$(RTL_DIR)/soc/vhdd_readahead.v \
	$(TB_DIR)/tb_scsi_vhdd_sd.v
SCSI_BUILD := $(BUILD_DIR)/scsi
SCSI_C96_PROBE_BUILD := $(BUILD_DIR)/scsi_c96_probe
SCSI_C96_REGISTER_BUILD := $(BUILD_DIR)/scsi_c96_register
SCSI_C96_INQUIRY_BUILD := $(BUILD_DIR)/scsi_c96_inquiry
SCSI_C96_READ6_BUILD := $(BUILD_DIR)/scsi_c96_read6
SCSI_C96_SM43_CHUNK_BUILD := $(BUILD_DIR)/scsi_c96_sm43_chunk
SCSI_C96_MAME_CHUNK_BUILD := $(BUILD_DIR)/scsi_c96_mame_chunk
SCSI_C96_STALL_SHAPES_BUILD := $(BUILD_DIR)/scsi_c96_stall_shapes
SCSI_C96_STUCK_SUPPLY_BUILD := $(BUILD_DIR)/scsi_c96_stuck_supply
SCSI_C96_CMDOUT_BUILD := $(BUILD_DIR)/scsi_c96_cmdout_drain
MAME_SCSI_BRIDGE_BUILD := $(BUILD_DIR)/mame_scsi_bridge

.PHONY: tb-scsi
tb-scsi: $(SCSI_BUILD)/Vtb_scsi_vhdd_sd
	@echo "Running scsi unit tb..."
	$(SCSI_BUILD)/Vtb_scsi_vhdd_sd

$(SCSI_BUILD)/Vtb_scsi_vhdd_sd: $(SCSI_RTL) $(TB_DIR)/tb_scsi.cpp
	@mkdir -p $(SCSI_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(SCSI_BUILD) \
		--top-module tb_scsi_vhdd_sd \
		$(SCSI_RTL) \
		$(TB_DIR)/tb_scsi.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# Two-target SCSI + vhdd_mux routing tb (tb-scsi-dual)
#
# One scsi.v answering to ID 0 and ID 1, a vhdd_mux, and two behavioural
# vhdd providers with byte-distinct data signatures.  Proves vh_dev_sel
# routing, dev_en gating, and the absence of cross-talk between volumes.
# ──────────────────────────────────────────────────────────────────────────────
SCSI_DUAL_RTL := $(RTL_DIR)/mac/scsi.v \
	$(RTL_DIR)/soc/vhdd_mux.v \
	$(TB_DIR)/tb_scsi_dual.v
SCSI_DUAL_BUILD := $(BUILD_DIR)/scsi_dual

.PHONY: tb-scsi-dual
tb-scsi-dual: $(SCSI_DUAL_BUILD)/Vtb_scsi_dual
	@echo "Running two-target scsi + vhdd_mux tb..."
	$(SCSI_DUAL_BUILD)/Vtb_scsi_dual

$(SCSI_DUAL_BUILD)/Vtb_scsi_dual: $(SCSI_DUAL_RTL) $(TB_DIR)/tb_scsi_dual.cpp
	@mkdir -p $(SCSI_DUAL_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(SCSI_DUAL_BUILD) \
		--top-module tb_scsi_dual \
		$(SCSI_DUAL_RTL) \
		$(TB_DIR)/tb_scsi_dual.cpp \
		-CFLAGS "-std=c++17"

.PHONY: tb-scsi-c96-probe
tb-scsi-c96-probe: $(SCSI_C96_PROBE_BUILD)/Vtb_scsi_vhdd_sd
	@echo "Running C96 TurboSCSI ROM-probe RTL tb..."
	$(SCSI_C96_PROBE_BUILD)/Vtb_scsi_vhdd_sd

$(SCSI_C96_PROBE_BUILD)/Vtb_scsi_vhdd_sd: $(SCSI_RTL) $(TB_DIR)/tb_scsi_c96_probe.cpp
	@mkdir -p $(SCSI_C96_PROBE_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(SCSI_C96_PROBE_BUILD) \
		--top-module tb_scsi_vhdd_sd \
		-GTURBOSCSI_C96=1 \
		$(SCSI_RTL) \
		$(TB_DIR)/tb_scsi_c96_probe.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SCSI 53C96 register-level unit testbench (TURBOSCSI_C96=1)
# Exercises each NCR 53C96 CSR's MAME-faithful read/write semantics.
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: tb-scsi-c96-register
tb-scsi-c96-register: $(SCSI_C96_REGISTER_BUILD)/Vtb_scsi_vhdd_sd
	@echo "Running C96 53C96 register-level RTL tb..."
	$(SCSI_C96_REGISTER_BUILD)/Vtb_scsi_vhdd_sd

$(SCSI_C96_REGISTER_BUILD)/Vtb_scsi_vhdd_sd: $(SCSI_RTL) $(TB_DIR)/tb_scsi_c96_register.cpp
	@mkdir -p $(SCSI_C96_REGISTER_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(SCSI_C96_REGISTER_BUILD) \
		--top-module tb_scsi_vhdd_sd \
		-GTURBOSCSI_C96=1 \
		$(SCSI_RTL) \
		$(TB_DIR)/tb_scsi_c96_register.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SCSI 53C96 INQUIRY end-to-end tb (TURBOSCSI_C96=1, TARGET_ID=6)
# Exercises CD_SELECT_ATN_STOP → DATA_IN → I_FUNCTION via the C96 path.
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: tb-scsi-c96-inquiry
tb-scsi-c96-inquiry: $(SCSI_C96_INQUIRY_BUILD)/Vtb_scsi_vhdd_sd
	@echo "Running C96 INQUIRY end-to-end RTL tb..."
	$(SCSI_C96_INQUIRY_BUILD)/Vtb_scsi_vhdd_sd

$(SCSI_C96_INQUIRY_BUILD)/Vtb_scsi_vhdd_sd: $(SCSI_RTL) $(TB_DIR)/tb_scsi_c96_inquiry.cpp
	@mkdir -p $(SCSI_C96_INQUIRY_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(SCSI_C96_INQUIRY_BUILD) \
		--top-module tb_scsi_vhdd_sd \
		-GTURBOSCSI_C96=1 \
		-GTARGET_ID=6 \
		$(SCSI_RTL) \
		$(TB_DIR)/tb_scsi_c96_inquiry.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SCSI 53C96 READ(6) end-to-end tb (TURBOSCSI_C96=1, TARGET_ID=6)
# Drives a READ(6) CDB through the C96 path with a mocked SD backing store.
# Validates byte-identical 512 B transfer + LBA→SD-sector bias.
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: tb-scsi-c96-read6
tb-scsi-c96-read6: $(SCSI_C96_READ6_BUILD)/Vtb_scsi_vhdd_sd
	@echo "Running C96 READ(6) end-to-end RTL tb..."
	$(SCSI_C96_READ6_BUILD)/Vtb_scsi_vhdd_sd

$(SCSI_C96_READ6_BUILD)/Vtb_scsi_vhdd_sd: $(SCSI_RTL) $(TB_DIR)/tb_scsi_c96_read6.cpp
	@mkdir -p $(SCSI_C96_READ6_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(SCSI_C96_READ6_BUILD) \
		--top-module tb_scsi_vhdd_sd \
		-GTURBOSCSI_C96=1 \
		-GTARGET_ID=6 \
		$(SCSI_RTL) \
		$(TB_DIR)/tb_scsi_c96_read6.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SCSI 53C96 non-DMA DATA IN trailing-transfer completion (TURBOSCSI_C96=1)
# The 2026-09-06 boot-wedge contract: a non-DMA CI_XFER in DATA IN moves one
# byte + I_BUS regardless of FIFO residue, and completes even with nothing
# to move.  Deliberate real-chip-over-MAME divergence — see the divergence
# box in rtl/mac/scsi.v and docs/scsi_fuzz.md.
# ──────────────────────────────────────────────────────────────────────────────
SCSI_C96_NONDMA_TRAILING_BUILD := $(BUILD_DIR)/scsi_c96_nondma_trailing

.PHONY: tb-scsi-c96-nondma-trailing
tb-scsi-c96-nondma-trailing: $(SCSI_C96_NONDMA_TRAILING_BUILD)/Vtb_scsi_vhdd_sd
	@echo "Running C96 non-DMA trailing-completion RTL tb..."
	$(SCSI_C96_NONDMA_TRAILING_BUILD)/Vtb_scsi_vhdd_sd

$(SCSI_C96_NONDMA_TRAILING_BUILD)/Vtb_scsi_vhdd_sd: $(SCSI_RTL) $(TB_DIR)/tb_scsi_c96_nondma_trailing.cpp
	@mkdir -p $(SCSI_C96_NONDMA_TRAILING_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(SCSI_C96_NONDMA_TRAILING_BUILD) \
		--top-module tb_scsi_vhdd_sd \
		-GTURBOSCSI_C96=1 \
		-GTARGET_ID=6 \
		$(SCSI_RTL) \
		$(TB_DIR)/tb_scsi_c96_nondma_trailing.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SCSI 53C96 differential-fuzz harness (TURBOSCSI_C96=1, TARGET_ID=6)
#
# Scriptable register-access executor for tools/fuzz/scsi_fuzz.py — the
# MAME-ncr53c90-golden-ref differential fuzzer.  See docs/scsi_fuzz.md.
# --public-flat-rw so the sync-point records can snapshot c96_* internals
# without popping the FIFO (mirrors the MAME side's save-item reads).
#
# TWO shapes are built from the SAME tb/tb_scsi_fuzz.cpp:
#
#   tb-scsi-fuzz-harness         (DEFAULT for `make fuzz-scsi`)
#       top = tb_pb_scsi = REAL peripheral_bus.v + REAL scsi.v + vhdd_sd,
#       driven over AXI4.  This is what puts the pseudo-DMA aperture's
#       word-splitting serializers (rd_scsi_phase_q / wr_scsi_strb_q) and
#       the scsi_dma16_lo_beat DRQ-grant carry INSIDE the fuzzer's DUT.
#       Without it no seed can reach the FSM that shipped the 2026-08-19
#       Sad Mac 0F02 — see docs/scsi_fuzz.md "Known blind spots" #3.
#       PB_WATCHDOG_LOG2=13 keeps a genuinely withheld beat's SLVERR fast
#       enough to report instead of hanging the run.
#
#   tb-scsi-fuzz-harness-direct
#       top = tb_scsi_vhdd_sd = scsi.v + vhdd_sd, the historical shape.
#       Kept as the ATTRIBUTION build: re-running a PB divergence here
#       says whether the fabric or the chip model owns it.
# ──────────────────────────────────────────────────────────────────────────────
SCSI_FUZZ_BUILD    := $(BUILD_DIR)/scsi_fuzz_pb
SCSI_FUZZ_DIR_BUILD := $(BUILD_DIR)/scsi_fuzz

SCSI_FUZZ_PB_RTL := \
	$(RTL_DIR)/mac/scsi.v \
	$(RTL_DIR)/soc/sd_scsi_lba_mapper.v \
	$(RTL_DIR)/soc/vhdd_sd.v \
	$(RTL_DIR)/soc/peripheral_bus.v \
	$(TB_DIR)/tb_pb_scsi.v

.PHONY: tb-scsi-fuzz-harness tb-scsi-fuzz-harness-direct
tb-scsi-fuzz-harness: $(SCSI_FUZZ_BUILD)/Vtb_pb_scsi
tb-scsi-fuzz-harness-direct: $(SCSI_FUZZ_DIR_BUILD)/Vtb_scsi_vhdd_sd

$(SCSI_FUZZ_BUILD)/Vtb_pb_scsi: $(SCSI_FUZZ_PB_RTL) $(TB_DIR)/tb_scsi_fuzz.cpp
	@mkdir -p $(SCSI_FUZZ_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		--public-flat-rw \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/mac -I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(SCSI_FUZZ_BUILD) \
		--top-module tb_pb_scsi \
		-GTARGET_ID=6 \
		-GPB_WATCHDOG_LOG2=13 \
		$(SCSI_FUZZ_PB_RTL) \
		$(TB_DIR)/tb_scsi_fuzz.cpp \
		-CFLAGS "-std=c++17 -DSCSI_FUZZ_PB"

$(SCSI_FUZZ_DIR_BUILD)/Vtb_scsi_vhdd_sd: $(SCSI_RTL) $(TB_DIR)/tb_scsi_fuzz.cpp
	@mkdir -p $(SCSI_FUZZ_DIR_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		--public-flat-rw \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(SCSI_FUZZ_DIR_BUILD) \
		--top-module tb_scsi_vhdd_sd \
		-GTURBOSCSI_C96=1 \
		-GTARGET_ID=6 \
		$(SCSI_RTL) \
		$(TB_DIR)/tb_scsi_fuzz.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SCSI 53C96 MEASURED-driver "bare repeat" chunk-loop tb
# (TURBOSCSI_C96=1, TARGET_ID=6)
#
# Replays the pattern captured from a healthy MAME 7.5.3 boot (275,355
# 53C96 register-access events): the transfer counter latch is written
# ONCE, and each subsequent 16-byte chunk is armed by a BARE repeat of
# command 0x90 — no FLUSH_FIFO, no TC rewrite in between.  The existing
# tb-scsi-c96-read6 chunk loop re-arms with a full flush + TC rewrite
# every chunk, so this path was untested under the ROM select form.
#
# The POSITIVE CONTROL runs FIRST and must exit 0 only when the
# assertions actually failed — a green test that cannot fail measures
# nothing.
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: tb-scsi-c96-mame-chunk
tb-scsi-c96-mame-chunk: $(SCSI_C96_MAME_CHUNK_BUILD)/Vtb_scsi_vhdd_sd
	@echo "Running C96 MAME bare-repeat POSITIVE CONTROL (the assertions must FAIL here)..."
	$(SCSI_C96_MAME_CHUNK_BUILD)/Vtb_scsi_vhdd_sd --positive-control
	@echo "Running C96 MAME bare-repeat chunk-loop RTL tb..."
	$(SCSI_C96_MAME_CHUNK_BUILD)/Vtb_scsi_vhdd_sd

$(SCSI_C96_MAME_CHUNK_BUILD)/Vtb_scsi_vhdd_sd: $(SCSI_RTL) $(TB_DIR)/tb_scsi_c96_mame_chunk.cpp
	@mkdir -p $(SCSI_C96_MAME_CHUNK_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(SCSI_C96_MAME_CHUNK_BUILD) \
		--top-module tb_scsi_vhdd_sd \
		-GTURBOSCSI_C96=1 \
		-GTARGET_ID=6 \
		$(SCSI_RTL) \
		$(TB_DIR)/tb_scsi_c96_mame_chunk.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SCSI 53C96 three-stall-shapes tb (TURBOSCSI_C96=1, TARGET_ID=6)
# Directed reproductions of the 2026-09-06/07 hardware stall fingerprints:
# (i) FIFO-residue INT park (ROM 0x40899706, status=0x11), (ii) select-
# retry cycle exit path, (iii) DMA-select CDB-tail DREQ spin (0x40898ea8,
# fifo=16).  ROM-faithful blind 16-bit word beats honouring dma_rd_ready.
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: tb-scsi-c96-stall-shapes
tb-scsi-c96-stall-shapes: $(SCSI_C96_STALL_SHAPES_BUILD)/Vtb_scsi_vhdd_sd
	@echo "Running C96 stall-shapes POSITIVE CONTROL (the assertions must FAIL here)..."
	$(SCSI_C96_STALL_SHAPES_BUILD)/Vtb_scsi_vhdd_sd --positive-control
	@echo "Running C96 stall-shapes RTL tb..."
	$(SCSI_C96_STALL_SHAPES_BUILD)/Vtb_scsi_vhdd_sd

$(SCSI_C96_STALL_SHAPES_BUILD)/Vtb_scsi_vhdd_sd: $(SCSI_RTL) $(TB_DIR)/tb_scsi_c96_stall_shapes.cpp
	@mkdir -p $(SCSI_C96_STALL_SHAPES_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(SCSI_C96_STALL_SHAPES_BUILD) \
		--top-module tb_scsi_vhdd_sd \
		-GTURBOSCSI_C96=1 \
		-GTARGET_ID=6 \
		$(SCSI_RTL) \
		$(TB_DIR)/tb_scsi_c96_stall_shapes.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SCSI 53C96 stuck-supply repro (TURBOSCSI_C96=1, TARGET_ID=6)
# Deterministic repro of the boot-#2 hardware pin at ROM 0x40899664: a
# multi-block read whose SD provider stalls mid-stream (sd_busy high, no
# done/error) leaves the chip starved with no completion path.
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: tb-scsi-c96-stuck-supply
tb-scsi-c96-stuck-supply: $(SCSI_C96_STUCK_SUPPLY_BUILD)/Vtb_scsi_vhdd_sd
	@echo "Running C96 stuck-supply repro (POST-FIX: recovery expected)..."
	$(SCSI_C96_STUCK_SUPPLY_BUILD)/Vtb_scsi_vhdd_sd

$(SCSI_C96_STUCK_SUPPLY_BUILD)/Vtb_scsi_vhdd_sd: $(SCSI_RTL) $(TB_DIR)/tb_scsi_c96_stuck_supply.cpp
	@mkdir -p $(SCSI_C96_STUCK_SUPPLY_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(SCSI_C96_STUCK_SUPPLY_BUILD) \
		--top-module tb_scsi_vhdd_sd \
		-GTURBOSCSI_C96=1 \
		-GTARGET_ID=6 \
		$(SCSI_RTL) \
		$(TB_DIR)/tb_scsi_c96_stuck_supply.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SCSI 53C96 SM4.3 chunk-loop wedge-repro tb (TURBOSCSI_C96=1, TARGET_ID=6)
# Measurement-only repro harness for the System 7.5.3 "Starting up…" HW
# boot wedge (bitstream 0xE80161D3): cmd=0x90, tcounter=16 undecremented,
# phase DATA IN, no INT — with per-transaction byte-conservation
# instrumentation in scsi.v (`ifdef VERILATOR).
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: tb-scsi-c96-sm43-chunk
tb-scsi-c96-sm43-chunk: $(SCSI_C96_SM43_CHUNK_BUILD)/Vtb_scsi_vhdd_sd
	@echo "Running C96 SM4.3 chunk-loop wedge-repro RTL tb..."
	$(SCSI_C96_SM43_CHUNK_BUILD)/Vtb_scsi_vhdd_sd

$(SCSI_C96_SM43_CHUNK_BUILD)/Vtb_scsi_vhdd_sd: $(SCSI_RTL) $(TB_DIR)/tb_scsi_c96_sm43_chunk.cpp
	@mkdir -p $(SCSI_C96_SM43_CHUNK_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(SCSI_C96_SM43_CHUNK_BUILD) \
		--top-module tb_scsi_vhdd_sd \
		-GTURBOSCSI_C96=1 \
		-GTARGET_ID=6 \
		$(SCSI_RTL) \
		$(TB_DIR)/tb_scsi_c96_sm43_chunk.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# sd_scsi_bridge core→pb read-CDC pulse-conservation measurement tb
# ──────────────────────────────────────────────────────────────────────────────
SD_BRIDGE_PULSE_BUILD := $(BUILD_DIR)/sd_bridge_pulse
.PHONY: tb-sd-bridge-pulse
tb-sd-bridge-pulse: $(SD_BRIDGE_PULSE_BUILD)/Vsd_scsi_bridge
	@echo "Running sd_scsi_bridge pulse-conservation tb..."
	$(SD_BRIDGE_PULSE_BUILD)/Vsd_scsi_bridge

$(SD_BRIDGE_PULSE_BUILD)/Vsd_scsi_bridge: $(RTL_DIR)/soc/sd_scsi_bridge.v $(TB_DIR)/tb_sd_bridge_pulse.cpp
	@mkdir -p $(SD_BRIDGE_PULSE_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-I$(RTL_DIR)/soc -I$(RTL_DIR) \
		-Mdir $(SD_BRIDGE_PULSE_BUILD) \
		--top-module sd_scsi_bridge \
		$(RTL_DIR)/soc/sd_scsi_bridge.v \
		$(TB_DIR)/tb_sd_bridge_pulse.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SCSI 53C96 System-7 command-out drain tb (TURBOSCSI_C96=1, TARGET_ID=6)
# Repro of the post-"Welcome to Macintosh" HW wedge: deferred DMA|CD_SELECT
# with the System SCSI Manager's FIFO drain-poll between the CDB prefix and
# the pseudo-DMA tail byte, plus the ATN-form MSG_OUT-first sequence.
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: tb-scsi-c96-cmdout-drain
tb-scsi-c96-cmdout-drain: $(SCSI_C96_CMDOUT_BUILD)/Vtb_scsi_vhdd_sd
	@echo "Running C96 System-7 command-out drain RTL tb..."
	$(SCSI_C96_CMDOUT_BUILD)/Vtb_scsi_vhdd_sd

$(SCSI_C96_CMDOUT_BUILD)/Vtb_scsi_vhdd_sd: $(SCSI_RTL) $(TB_DIR)/tb_scsi_c96_cmdout_drain.cpp
	@mkdir -p $(SCSI_C96_CMDOUT_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(SCSI_C96_CMDOUT_BUILD) \
		--top-module tb_scsi_vhdd_sd \
		-GTURBOSCSI_C96=1 \
		-GTARGET_ID=6 \
		$(SCSI_RTL) \
		$(TB_DIR)/tb_scsi_c96_cmdout_drain.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# TurboSCSI shim — DRQ-check / DTACK-hold + register pass-through scenarios.
# Same scsi.v as tb-scsi-c96-probe, with the new scsi_ctrl_in port driven.
# ──────────────────────────────────────────────────────────────────────────────
TURBOSCSI_BUILD := $(BUILD_DIR)/turboscsi

.PHONY: tb-turboscsi
tb-turboscsi: $(TURBOSCSI_BUILD)/Vtb_scsi_vhdd_sd
	@echo "Running TurboSCSI shim unit tb..."
	$(TURBOSCSI_BUILD)/Vtb_scsi_vhdd_sd

$(TURBOSCSI_BUILD)/Vtb_scsi_vhdd_sd: $(SCSI_RTL) $(TB_DIR)/tb_turboscsi.cpp
	@mkdir -p $(TURBOSCSI_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(TURBOSCSI_BUILD) \
		--top-module tb_scsi_vhdd_sd \
		-GTURBOSCSI_C96=1 \
		$(SCSI_RTL) \
		$(TB_DIR)/tb_turboscsi.cpp \
		-CFLAGS "-std=c++17"

.PHONY: mame-scsi-bridge
mame-scsi-bridge: $(MAME_SCSI_BRIDGE_BUILD)/Vtb_scsi_vhdd_sd
	@echo "MAME SCSI RTL bridge: $(MAME_SCSI_BRIDGE_BUILD)/Vtb_scsi_vhdd_sd"

.PHONY: mame-q700-rtl-overlay-selftest
mame-q700-rtl-overlay-selftest:
	python3 $(TOOLS_DIR)/mame_q700_rtl_overlay.py --selftest

.PHONY: mame-q700-platform-lockstep
mame-q700-platform-lockstep: $(MAME_AXI_PERIPH_BUILD)/Vmame_axi_periph_top $(MAME_PATCH_ROM)
	@mkdir -p $(MAME_PLATFORM_LOCKSTEP_DIR) $(MAME_PLATFORM_LOCKSTEP_DIR)/nvram $(MAME_ROMPATH)/macqd700 $(MAME_PLATFORM_LOCKSTEP_ACTIVE_ROMPATH)/macqd700 $(dir $(MAME_PLATFORM_LOCKSTEP_ROM))
	@if [ -n "$(strip $(MAME_PLATFORM_LOCKSTEP_PATCH))" ]; then \
		$(MAME_PATCH_ROM) --in "$(ROM)" --out "$(MAME_PLATFORM_LOCKSTEP_ROM)" --patch "$(MAME_PLATFORM_LOCKSTEP_PATCH)"; \
	else \
		if [ "$$(readlink -f "$(ROM)")" != "$$(readlink -f "$(MAME_ROMPATH)/macqd700/420dbff3.rom" 2>/dev/null || true)" ]; then \
			cp "$(ROM)" "$(MAME_ROMPATH)/macqd700/420dbff3.rom"; \
		fi; \
	fi
	@if [ "$$(readlink -f "$(MAME_ADB_ROM)")" != "$$(readlink -f "$(MAME_PLATFORM_LOCKSTEP_ACTIVE_ROMPATH)/macqd700/342s0440-b.bin" 2>/dev/null || true)" ]; then \
		cp "$(MAME_ADB_ROM)" "$(MAME_PLATFORM_LOCKSTEP_ACTIVE_ROMPATH)/macqd700/342s0440-b.bin"; \
	fi
	@test -x "$(MAME_RTL_BIN)" || (echo "missing executable MAME_RTL_BIN=$(MAME_RTL_BIN); patch/build MAME with tools/mame_q700_rtl_overlay.py or override MAME_RTL_BIN" >&2; exit 2)
	@rm -f "$(MAME_PLATFORM_LOCKSTEP_SOCKET)"
	@echo "Starting platform MMIO lockstep: every non-RAM/non-ROM/non-VRAM window is bridged or fatal."
	@$(MAME_AXI_PERIPH_BUILD)/Vmame_axi_periph_top \
		--socket "$(MAME_PLATFORM_LOCKSTEP_SOCKET)" \
		--trace-pc-change \
		--max-cpu-advance "$(MAME_PLATFORM_LOCKSTEP_MAX_CPU_ADVANCE)" \
		$(MAME_PLATFORM_LOCKSTEP_BRIDGE_PLUSARGS) \
		> "$(MAME_PLATFORM_LOCKSTEP_DIR)/bridge.log" 2>&1 & \
	bridge_pid=$$!; \
	trap 'kill $$bridge_pid >/dev/null 2>&1 || true; wait $$bridge_pid >/dev/null 2>&1 || true' EXIT INT TERM; \
	for i in $$(seq 1 100); do \
		[ -S "$(MAME_PLATFORM_LOCKSTEP_SOCKET)" ] && break; \
		sleep 0.05; \
	done; \
	if [ ! -S "$(MAME_PLATFORM_LOCKSTEP_SOCKET)" ]; then \
		echo "bridge did not create $(MAME_PLATFORM_LOCKSTEP_SOCKET)" >&2; \
		exit 2; \
	fi; \
	QT_QPA_PLATFORM=offscreen \
	MAME_RTL_BRIDGE_SOCKET="$(MAME_PLATFORM_LOCKSTEP_SOCKET)" \
	MAME_RTL_LOCKSTEP=1 \
	MAME_RTL_READ_SOURCE=rtl \
	MAME_RTL_READ_SOURCE_MAME_LABELS=VRAM \
	MAME_RTL_BRIDGE_LABELS="$(MAME_PLATFORM_LOCKSTEP_LABELS)" \
	MAME_RTL_TRACE_LABELS="$(MAME_PLATFORM_LOCKSTEP_LABELS)" \
	MAME_RTL_REQUIRE_LABELS="$(MAME_PLATFORM_LOCKSTEP_LABELS)" \
	MAME_RTL_FAIL_ON_MISSING=1 \
	MAME_RTL_FAIL_ON_UNMODELED=1 \
	MAME_RTL_LOCKSTEP_FATAL=1 \
	MAME_RTL_ACCEPT_VALIDATED_TIMING="$(MAME_PLATFORM_LOCKSTEP_ACCEPT_VALIDATED_TIMING)" \
	MAME_RTL_RTC_DATE="$(MAME_PLATFORM_LOCKSTEP_RTC_DATE)" \
	MAME_RTL_MMIO_TRACE="$(MAME_PLATFORM_LOCKSTEP_DIR)/mmio.log" \
	"$(MAME_RTL_BIN)" macqd700 \
		-rompath "$(MAME_PLATFORM_LOCKSTEP_ACTIVE_ROMPATH)" \
		-nvram_directory "$(MAME_PLATFORM_LOCKSTEP_DIR)/nvram" \
		-nothrottle -seconds_to_run "$(MAME_PLATFORM_LOCKSTEP_SECONDS)" \
		-video none -sound none -nojoy \
		> "$(MAME_PLATFORM_LOCKSTEP_DIR)/mame.log" 2>&1; \
	rc=$$?; \
	kill $$bridge_pid >/dev/null 2>&1 || true; \
	wait $$bridge_pid >/dev/null 2>&1 || true; \
	exit $$rc

$(MAME_SCSI_BRIDGE_BUILD)/Vtb_scsi_vhdd_sd: $(SCSI_RTL) $(TOOLS_DIR)/mame_scsi_bridge.cpp
	@mkdir -p $(MAME_SCSI_BRIDGE_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(MAME_SCSI_BRIDGE_BUILD) \
		--top-module tb_scsi_vhdd_sd \
		-GTURBOSCSI_C96=1 \
		$(SCSI_RTL) \
		$(TOOLS_DIR)/mame_scsi_bridge.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SCSI raw SD LBA mapper unit testbench
# ──────────────────────────────────────────────────────────────────────────────
SCSI_LBA_MAPPER_RTL   := $(RTL_DIR)/soc/sd_scsi_lba_mapper.v
SCSI_LBA_MAPPER_BUILD := $(BUILD_DIR)/sd_scsi_lba_mapper

.PHONY: tb-sd-scsi-lba-mapper
tb-sd-scsi-lba-mapper: $(SCSI_LBA_MAPPER_BUILD)/Vsd_scsi_lba_mapper
	@echo "Running sd_scsi_lba_mapper unit tb..."
	$(SCSI_LBA_MAPPER_BUILD)/Vsd_scsi_lba_mapper

$(SCSI_LBA_MAPPER_BUILD)/Vsd_scsi_lba_mapper: $(SCSI_LBA_MAPPER_RTL) $(TB_DIR)/tb_sd_scsi_lba_mapper.cpp
	@mkdir -p $(SCSI_LBA_MAPPER_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(SCSI_LBA_MAPPER_BUILD) \
		--top-module sd_scsi_lba_mapper \
		$(SCSI_LBA_MAPPER_RTL) \
		$(TB_DIR)/tb_sd_scsi_lba_mapper.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# L2 bypass-window polarity test — the RAM-disk carveout
#
# l2c_bypass.v's mask convention is INVERTED (1 = FIXED base bit).  Getting
# it backwards silently CACHES the 256 MB RAM disk instead of bypassing it:
# no error, no failing check anywhere, just a poisoned 2 MB L2.  Two builds:
# the real constants, and the same test with the mask inverted, which MUST
# fail.  A polarity test that has never been shown to notice a wrong
# polarity is not a polarity test.
# ──────────────────────────────────────────────────────────────────────────────
L2CBYP_RTL   := $(RTL_DIR)/soc/l2c_bypass.v
L2CBYP_BUILD := $(BUILD_DIR)/l2c_bypass_window
L2CBYP_VFLAGS := --cc --exe --build --assert --x-assign fast --x-initial fast -O1 \
	-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
	-Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND \
	-I$(RTL_DIR)/soc -I$(RTL_DIR) --top-module l2c_bypass

.PHONY: tb-l2c-bypass-window
tb-l2c-bypass-window: $(L2CBYP_BUILD)/inv/Vl2c_bypass $(L2CBYP_BUILD)/ok/Vl2c_bypass
	@echo "Running L2 bypass POSITIVE CONTROL (inverted mask — must FAIL)..."
	$(L2CBYP_BUILD)/inv/Vl2c_bypass
	@echo "Running L2 bypass window polarity test..."
	$(L2CBYP_BUILD)/ok/Vl2c_bypass

$(L2CBYP_BUILD)/ok/Vl2c_bypass: $(L2CBYP_RTL) $(TB_DIR)/tb_l2c_bypass_window.cpp
	@mkdir -p $(L2CBYP_BUILD)/ok
	$(VERILATOR) $(L2CBYP_VFLAGS) \
		-GNUM_WINDOWS=1 -GWIN_BASE=0x50000000 -GWIN_MASK=0xF0000000 -GWIN_EN=1 \
		-Mdir $(L2CBYP_BUILD)/ok $(L2CBYP_RTL) \
		$(TB_DIR)/tb_l2c_bypass_window.cpp \
		-CFLAGS "-std=c++17 -DEXPECT_POLARITY_OK=1"

$(L2CBYP_BUILD)/inv/Vl2c_bypass: $(L2CBYP_RTL) $(TB_DIR)/tb_l2c_bypass_window.cpp
	@mkdir -p $(L2CBYP_BUILD)/inv
	$(VERILATOR) $(L2CBYP_VFLAGS) \
		-GNUM_WINDOWS=1 -GWIN_BASE=0x50000000 -GWIN_MASK=0x0FFFFFFF -GWIN_EN=1 \
		-Mdir $(L2CBYP_BUILD)/inv $(L2CBYP_RTL) \
		$(TB_DIR)/tb_l2c_bypass_window.cpp \
		-CFLAGS "-std=c++17 -DEXPECT_POLARITY_OK=0"

# ──────────────────────────────────────────────────────────────────────────────
# vhdd_ctrl unit testbench — the vHDD AXI-Lite register file (xbar S2)
#
# Covers the host control surface AND the window-terminator behaviour it
# inherits from axil_null_slave.  The scenario that earns its keep is the
# same-cycle AW+W write: axi_wide_to_axilite presents both halves together,
# and a commit that reads the LATCHED wdata/wstrb in that case silently
# stores the previous write's data with BRESP=OKAY.
# ──────────────────────────────────────────────────────────────────────────────
# ──────────────────────────────────────────────────────────────────────────────
# scsi_trace_ring unit testbench — the 53C96 register-access trace ring.
#
# Runs the POSITIVE CONTROL first (pb_ack held low so the DUT can never
# commit an entry; every capture assertion must fail).  This tb exists to
# validate an INSTRUMENT, and an instrument whose self-test cannot fail
# would let a broken tap masquerade as "the SCSI bus went quiet".
# ──────────────────────────────────────────────────────────────────────────────
SCSI_RING_RTL   := $(RTL_DIR)/soc/scsi_trace_ring.v
SCSI_RING_BUILD := $(BUILD_DIR)/scsi_trace_ring

.PHONY: tb-scsi-trace-ring
tb-scsi-trace-ring: $(SCSI_RING_BUILD)/Vscsi_trace_ring
	@echo "Running scsi_trace_ring POSITIVE CONTROL (capture checks must FAIL here)..."
	$(SCSI_RING_BUILD)/Vscsi_trace_ring --positive-control
	@echo "Running scsi_trace_ring unit tb..."
	$(SCSI_RING_BUILD)/Vscsi_trace_ring

$(SCSI_RING_BUILD)/Vscsi_trace_ring: $(SCSI_RING_RTL) $(TB_DIR)/tb_scsi_trace_ring.cpp
	@mkdir -p $(SCSI_RING_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(SCSI_RING_BUILD) \
		--top-module scsi_trace_ring \
		$(SCSI_RING_RTL) \
		$(TB_DIR)/tb_scsi_trace_ring.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# sonic_trace_ring unit testbench — the DP83932C SONIC register-access trace
# ring.
#
# Runs the POSITIVE CONTROL first (pb_ack held low so the DUT can never
# commit an entry; every capture assertion must fail).  Same reasoning as
# tb-scsi-trace-ring: this tb validates an INSTRUMENT, and an instrument
# whose self-test cannot fail would let a broken tap masquerade as "the
# driver stopped touching the SONIC".
# ──────────────────────────────────────────────────────────────────────────────
SONIC_RING_RTL   := $(RTL_DIR)/soc/sonic_trace_ring.v
SONIC_RING_BUILD := $(BUILD_DIR)/sonic_trace_ring

.PHONY: tb-sonic-trace-ring
tb-sonic-trace-ring: $(SONIC_RING_BUILD)/Vsonic_trace_ring
	@echo "Running sonic_trace_ring POSITIVE CONTROL (capture checks must FAIL here)..."
	$(SONIC_RING_BUILD)/Vsonic_trace_ring --positive-control
	@echo "Running sonic_trace_ring unit tb..."
	$(SONIC_RING_BUILD)/Vsonic_trace_ring

$(SONIC_RING_BUILD)/Vsonic_trace_ring: $(SONIC_RING_RTL) $(TB_DIR)/tb_sonic_trace_ring.cpp
	@mkdir -p $(SONIC_RING_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(SONIC_RING_BUILD) \
		--top-module sonic_trace_ring \
		$(SONIC_RING_RTL) \
		$(TB_DIR)/tb_sonic_trace_ring.cpp \
		-CFLAGS "-std=c++17"

VHDD_CTRL_RTL   := $(RTL_DIR)/soc/vhdd_ctrl.v
VHDD_CTRL_BUILD := $(BUILD_DIR)/vhdd_ctrl

.PHONY: tb-vhdd-ctrl
tb-vhdd-ctrl: $(VHDD_CTRL_BUILD)/Vvhdd_ctrl
	@echo "Running vhdd_ctrl unit tb..."
	$(VHDD_CTRL_BUILD)/Vvhdd_ctrl

$(VHDD_CTRL_BUILD)/Vvhdd_ctrl: $(VHDD_CTRL_RTL) $(TB_DIR)/tb_vhdd_ctrl.cpp
	@mkdir -p $(VHDD_CTRL_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(VHDD_CTRL_BUILD) \
		--top-module vhdd_ctrl \
		$(VHDD_CTRL_RTL) \
		$(TB_DIR)/tb_vhdd_ctrl.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# vhdd read-ahead cache unit testbench
#
# Built TWICE from one source so the "before" number comes from the same
# harness and the same provider cost model as the "after" one:
#
#   tb-vhdd-readahead      ENABLE=1  the cache
#   tb-vhdd-readahead-off  ENABLE=0  module built out (today's behaviour)
#
# SERVE_WDOG_BITS is shortened to 16 here so the bounded-response case
# completes inside the test; the SoC instantiates the default 24.
# ──────────────────────────────────────────────────────────────────────────────
VHDD_RA_RTL       := $(RTL_DIR)/soc/vhdd_readahead.v
VHDD_RA_BUILD     := $(BUILD_DIR)/vhdd_readahead
VHDD_RA_OFF_BUILD := $(BUILD_DIR)/vhdd_readahead_off

VHDD_RA_VFLAGS := --cc --exe --build --assert \
	--x-assign fast --x-initial fast -O3 \
	-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
	-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
	-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR)

.PHONY: tb-vhdd-readahead
tb-vhdd-readahead: $(VHDD_RA_BUILD)/Vvhdd_readahead
	@echo "Running vhdd_readahead unit tb (cache ENABLED)..."
	$(VHDD_RA_BUILD)/Vvhdd_readahead

$(VHDD_RA_BUILD)/Vvhdd_readahead: $(VHDD_RA_RTL) $(RTL_DIR)/vhdd.vh $(TB_DIR)/tb_vhdd_readahead.cpp
	@mkdir -p $(VHDD_RA_BUILD)
	$(VERILATOR) $(VHDD_RA_VFLAGS) \
		-Mdir $(VHDD_RA_BUILD) \
		--top-module vhdd_readahead \
		-GENABLE=1 -GBLOCKS_PER_WAY=32 -GSERVE_WDOG_BITS=16 \
		$(VHDD_RA_RTL) \
		$(TB_DIR)/tb_vhdd_readahead.cpp \
		-CFLAGS "-std=c++17"

.PHONY: tb-vhdd-readahead-off
tb-vhdd-readahead-off: $(VHDD_RA_OFF_BUILD)/Vvhdd_readahead
	@echo "Running vhdd_readahead unit tb (cache BUILT OUT -- baseline)..."
	$(VHDD_RA_OFF_BUILD)/Vvhdd_readahead

$(VHDD_RA_OFF_BUILD)/Vvhdd_readahead: $(VHDD_RA_RTL) $(RTL_DIR)/vhdd.vh $(TB_DIR)/tb_vhdd_readahead.cpp
	@mkdir -p $(VHDD_RA_OFF_BUILD)
	$(VERILATOR) $(VHDD_RA_VFLAGS) \
		-Mdir $(VHDD_RA_OFF_BUILD) \
		--top-module vhdd_readahead \
		-GENABLE=0 -GBLOCKS_PER_WAY=32 -GSERVE_WDOG_BITS=16 \
		$(VHDD_RA_RTL) \
		$(TB_DIR)/tb_vhdd_readahead.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SCSI ↔ SD bridge unit testbench (CDC primitives in isolation)
# ──────────────────────────────────────────────────────────────────────────────
SCSI_SD_BRIDGE_RTL   := $(RTL_DIR)/soc/sd_scsi_bridge.v
SCSI_SD_BRIDGE_BUILD := $(BUILD_DIR)/sd_scsi_bridge

.PHONY: tb-sd-scsi-bridge
tb-sd-scsi-bridge: $(SCSI_SD_BRIDGE_BUILD)/Vsd_scsi_bridge
	@echo "Running sd_scsi_bridge unit tb..."
	$(SCSI_SD_BRIDGE_BUILD)/Vsd_scsi_bridge

$(SCSI_SD_BRIDGE_BUILD)/Vsd_scsi_bridge: $(SCSI_SD_BRIDGE_RTL) $(TB_DIR)/tb_sd_scsi_bridge.cpp
	@mkdir -p $(SCSI_SD_BRIDGE_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(SCSI_SD_BRIDGE_BUILD) \
		--top-module sd_scsi_bridge \
		$(SCSI_SD_BRIDGE_RTL) \
		$(TB_DIR)/tb_sd_scsi_bridge.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# SCSI ↔ SD end-to-end testbench
# (scsi.v + sd_scsi_bridge.v + sd_ctrl.v + sd_spi.v wrapped by tb_scsi_sd_e2e.v)
#
# Drives the SCSI 5380 register interface from the Mac side and a host-
# side SD-card model on the SPI pins; proves SCSI READ(6/10) / WRITE(6/10)
# move correct bytes through the real SD path with the LBA biased through
# SD_LBA_BIAS=8192.
# ──────────────────────────────────────────────────────────────────────────────
SCSI_SD_E2E_RTL := \
	$(RTL_DIR)/mac/scsi.v \
	$(RTL_DIR)/soc/sd_scsi_lba_mapper.v \
	$(RTL_DIR)/soc/vhdd_sd.v \
	$(RTL_DIR)/soc/vhdd_readahead.v \
	$(TB_DIR)/tb_scsi_vhdd_sd.v \
	$(RTL_DIR)/soc/sd_scsi_bridge.v \
	$(RTL_DIR)/board/sd_ctrl.v \
	$(RTL_DIR)/board/sd_spi.v \
	$(TB_DIR)/tb_scsi_sd_e2e.v
SCSI_SD_E2E_BUILD := $(BUILD_DIR)/scsi_sd_e2e

.PHONY: tb-scsi-sd-e2e
tb-scsi-sd-e2e: $(SCSI_SD_E2E_BUILD)/Vtb_scsi_sd_e2e
	@echo "Running scsi ↔ SD end-to-end tb..."
	$(SCSI_SD_E2E_BUILD)/Vtb_scsi_sd_e2e

$(SCSI_SD_E2E_BUILD)/Vtb_scsi_sd_e2e: $(SCSI_SD_E2E_RTL) $(TB_DIR)/tb_scsi_sd_e2e.cpp
	@mkdir -p $(SCSI_SD_E2E_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/mac -I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(SCSI_SD_E2E_BUILD) \
		--top-module tb_scsi_sd_e2e \
		$(SCSI_SD_E2E_RTL) \
		$(TB_DIR)/tb_scsi_sd_e2e.cpp \
		-CFLAGS "-std=c++17"

# ----------------------------------------------------------------------------
# tb-scsi-sd-e2e-dual — the SAME end-to-end SCSI->SD test, built in the
# shape rtl/soc/fpga_top_peripherals.vh ACTUALLY INSTANTIATES.
#
# Why this exists (2026-08-03).  Every SCSI testbench in this tree goes
# through tb/tb_scsi_vhdd_sd.v, which hard-coded TARGET_B_EN=0,
# dev_en=2'b01 and NO vhdd_mux.  fpga_top builds TARGET_B_EN=1 with
# vhdd_mux on the vhdd seam and dev_en arriving over a CDC.  So the
# dual-target configuration that ships had ZERO coverage: `make tb-all`
# was green on a shape the bitstream does not contain.  That is the
# "tests that pass while measuring nothing" failure mode, and it is
# exactly the configuration the 2026-08-03 no-boot regression landed in.
#
# The C++ driver and its byte-for-byte assertions are UNCHANGED — only
# the RTL shape under them differs.  A green run here therefore means the
# dual-target path is observably identical at the SD seam, which is the
# claim the vHDD merge made and never demonstrated.
#
# POSITIVE CONTROL, and do not trust the green without it:
#     make tb-scsi-sd-e2e-dual-noselect
# builds the identical harness with dev_en=2'b00 (SD target absent from
# the bus).  It MUST fail.  If it passes, this test is measuring nothing
# and the green above is worthless.
SCSI_SD_E2E_DUAL_BUILD  := $(BUILD_DIR)/scsi_sd_e2e_dual
SCSI_SD_E2E_NOSEL_BUILD := $(BUILD_DIR)/scsi_sd_e2e_nosel

# $(1) = build dir, $(2) = extra +define+ args
define SCSI_SD_E2E_BUILD_RULE
	@mkdir -p $(1)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		$(2) \
		-I$(RTL_DIR)/mac -I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(1) \
		--top-module tb_scsi_sd_e2e \
		$(SCSI_SD_E2E_RTL) $(RTL_DIR)/soc/vhdd_mux.v \
		$(TB_DIR)/tb_scsi_sd_e2e.cpp \
		-CFLAGS "-std=c++17"
endef

.PHONY: tb-scsi-sd-e2e-dual
tb-scsi-sd-e2e-dual: $(SCSI_SD_E2E_DUAL_BUILD)/Vtb_scsi_sd_e2e
	@echo "Running scsi ↔ SD end-to-end tb in fpga_top's DUAL-TARGET shape..."
	$(SCSI_SD_E2E_DUAL_BUILD)/Vtb_scsi_sd_e2e

$(SCSI_SD_E2E_DUAL_BUILD)/Vtb_scsi_sd_e2e: $(SCSI_SD_E2E_RTL) \
		$(RTL_DIR)/soc/vhdd_mux.v $(TB_DIR)/tb_scsi_sd_e2e.cpp
	$(call SCSI_SD_E2E_BUILD_RULE,$(SCSI_SD_E2E_DUAL_BUILD),+define+SCSI_TB_DUAL_TARGET)

# Positive control — MUST FAIL.  Same harness, SD target disabled.
.PHONY: tb-scsi-sd-e2e-dual-noselect
tb-scsi-sd-e2e-dual-noselect: $(SCSI_SD_E2E_NOSEL_BUILD)/Vtb_scsi_sd_e2e
	@echo "POSITIVE CONTROL (must FAIL): dual-target harness, dev_en=2'b00..."
	@if $(SCSI_SD_E2E_NOSEL_BUILD)/Vtb_scsi_sd_e2e; then \
		echo "  [BAD] control PASSED with no SCSI target on the bus —"; \
		echo "        tb-scsi-sd-e2e-dual is measuring nothing."; \
		exit 1; \
	else \
		echo "  [OK] control failed as required (rig can see the target)"; \
	fi

$(SCSI_SD_E2E_NOSEL_BUILD)/Vtb_scsi_sd_e2e: $(SCSI_SD_E2E_RTL) \
		$(RTL_DIR)/soc/vhdd_mux.v $(TB_DIR)/tb_scsi_sd_e2e.cpp
	$(call SCSI_SD_E2E_BUILD_RULE,$(SCSI_SD_E2E_NOSEL_BUILD),+define+SCSI_TB_DUAL_TARGET "+define+SCSI_TB_DEV_EN=2'b00")

# ----------------------------------------------------------------------------
# tb-scsi-sd-e2e-c96 — the SAME end-to-end SCSI->SD test, driven through the
# NCR 53C96 + DAFB pseudo-DMA front end that the Mac ACTUALLY USES.
#
# Why this exists (2026-08-08).  Every byte-exact SCSI->SD test in this
# tree went through the bare-5380 REQ/ACK path (TURBOSCSI_C96=0).  The
# Q700 ROM and the 7.5.3 SCSI Manager do not: they drive the 53C96 with
# DMA-form Transfer Information through the DAFB pseudo-DMA shim.  So the
# combination that runs on hardware — a MULTI-BLOCK WRITE (CMD25 ring)
# through C96 pseudo-DMA — had ZERO end-to-end coverage, and the
# producer-side ring back-pressure in scsi.v's S_DATA_OUT pseudo-DMA arm
# could be structurally dead without any test failing.
#
# The bare-5380 path back-pressures correctly (REQ is cleared by
# `t_req && ack_rise`), which is precisely why tb-scsi-sd-e2e's
# s10_write10_lba_200_4blocks_cmd25 was green over a broken ring.
SCSI_SD_E2E_C96_BUILD := $(BUILD_DIR)/scsi_sd_e2e_c96
SCSI_SD_E2E_C96_CORE100_BUILD := $(BUILD_DIR)/scsi_sd_e2e_c96_core100

.PHONY: tb-scsi-sd-e2e-c96
tb-scsi-sd-e2e-c96: $(SCSI_SD_E2E_C96_BUILD)/Vtb_scsi_sd_e2e
	@echo "Running scsi ↔ SD end-to-end tb through the 53C96 pseudo-DMA front end..."
	$(SCSI_SD_E2E_C96_BUILD)/Vtb_scsi_sd_e2e

$(SCSI_SD_E2E_C96_BUILD)/Vtb_scsi_sd_e2e: $(SCSI_SD_E2E_RTL) \
		$(RTL_DIR)/soc/vhdd_mux.v $(TB_DIR)/tb_scsi_sd_e2e.cpp
	$(call SCSI_SD_E2E_BUILD_RULE,$(SCSI_SD_E2E_C96_BUILD),+define+SCSI_E2E_C96 -CFLAGS "-DSCSI_E2E_C96")

.PHONY: tb-scsi-sd-e2e-c96-core100
tb-scsi-sd-e2e-c96-core100: $(SCSI_SD_E2E_C96_CORE100_BUILD)/Vtb_scsi_sd_e2e
	@echo "Running C96 SCSI/SD e2e at production core=100 MHz, pb=50 MHz, SPI=25 MHz..."
	$(SCSI_SD_E2E_C96_CORE100_BUILD)/Vtb_scsi_sd_e2e

$(SCSI_SD_E2E_C96_CORE100_BUILD)/Vtb_scsi_sd_e2e: $(SCSI_SD_E2E_RTL) \
		$(RTL_DIR)/soc/vhdd_mux.v $(TB_DIR)/tb_scsi_sd_e2e.cpp
	$(call SCSI_SD_E2E_BUILD_RULE,$(SCSI_SD_E2E_C96_CORE100_BUILD),+define+SCSI_E2E_C96 +define+SCSI_E2E_CORE100 -CFLAGS "-DSCSI_E2E_C96 -DSCSI_E2E_CORE100")

# ----------------------------------------------------------------------------
# tb-scsi-sd-e2e-ra / -c96-ra — the SAME end-to-end SCSI->SD tests, with the
# read-ahead cache (rtl/soc/vhdd_readahead.v) SWITCHED ON.
#
# Same RTL stack, same C++ driver, same assertions, byte for byte.  Only the
# traffic underneath changes: one CMD18 per run of RA_BLOCKS instead of one
# CMD17 per block.  Green here means the whole chain — 5380/53C96 front end,
# vhdd_mux, the cache, vhdd_sd, the CDC bridge, sd_ctrl, sd_spi and the SD
# card model — still moves exactly the right bytes with the cache in the
# path, including the write-then-read-back cases that are the coherency
# hazard.
#
# The plain (non-ra) targets keep running the cache's ENABLE=0 arm, so the
# pair is a true A/B on one netlist.
# ----------------------------------------------------------------------------
SCSI_SD_E2E_RA_BUILD     := $(BUILD_DIR)/scsi_sd_e2e_ra
SCSI_SD_E2E_C96_RA_BUILD := $(BUILD_DIR)/scsi_sd_e2e_c96_ra
SCSI_SD_E2E_PROD_BUILD := $(BUILD_DIR)/scsi_sd_e2e_production
SCSI_SD_E2E_PROD_BASE_BUILD := $(BUILD_DIR)/scsi_sd_e2e_production_baseline
SCSI_SD_E2E_PROD100_BUILD := $(BUILD_DIR)/scsi_sd_e2e_production100

# Performance baseline matching the shipping 200 MHz / 50 MHz clocks,
# 50 MHz SPI, CRC-checked reads, 32-block read-ahead ways and CMD24 writes.
.PHONY: tb-scsi-sd-perf
tb-scsi-sd-perf: $(SCSI_SD_E2E_PROD_BUILD)/Vtb_scsi_sd_e2e
	$(SCSI_SD_E2E_PROD_BUILD)/Vtb_scsi_sd_e2e

$(SCSI_SD_E2E_PROD_BUILD)/Vtb_scsi_sd_e2e: $(SCSI_SD_E2E_RTL) \
		$(RTL_DIR)/soc/vhdd_mux.v $(TB_DIR)/tb_scsi_sd_e2e.cpp
	$(call SCSI_SD_E2E_BUILD_RULE,$(SCSI_SD_E2E_PROD_BUILD),+define+SCSI_E2E_C96 +define+SCSI_E2E_READAHEAD +define+SCSI_E2E_PRODUCTION -CFLAGS "-DSCSI_E2E_C96")

.PHONY: tb-scsi-sd-perf-baseline tb-scsi-sd-perf-core100
tb-scsi-sd-perf-baseline: $(SCSI_SD_E2E_PROD_BASE_BUILD)/Vtb_scsi_sd_e2e
	$(SCSI_SD_E2E_PROD_BASE_BUILD)/Vtb_scsi_sd_e2e

$(SCSI_SD_E2E_PROD_BASE_BUILD)/Vtb_scsi_sd_e2e: $(SCSI_SD_E2E_RTL) \
		$(RTL_DIR)/soc/vhdd_mux.v $(TB_DIR)/tb_scsi_sd_e2e.cpp
	$(call SCSI_SD_E2E_BUILD_RULE,$(SCSI_SD_E2E_PROD_BASE_BUILD),+define+SCSI_E2E_C96 +define+SCSI_E2E_READAHEAD +define+SCSI_E2E_PRODUCTION +define+SCSI_E2E_LEGACY_PACE -CFLAGS "-DSCSI_E2E_C96")

tb-scsi-sd-perf-core100: $(SCSI_SD_E2E_PROD100_BUILD)/Vtb_scsi_sd_e2e
	$(SCSI_SD_E2E_PROD100_BUILD)/Vtb_scsi_sd_e2e

$(SCSI_SD_E2E_PROD100_BUILD)/Vtb_scsi_sd_e2e: $(SCSI_SD_E2E_RTL) \
		$(RTL_DIR)/soc/vhdd_mux.v $(TB_DIR)/tb_scsi_sd_e2e.cpp
	$(call SCSI_SD_E2E_BUILD_RULE,$(SCSI_SD_E2E_PROD100_BUILD),+define+SCSI_E2E_C96 +define+SCSI_E2E_READAHEAD +define+SCSI_E2E_PRODUCTION +define+SCSI_E2E_CORE100 -CFLAGS "-DSCSI_E2E_C96 -DSCSI_E2E_CORE100")

.PHONY: tb-scsi-sd-e2e-ra
tb-scsi-sd-e2e-ra: $(SCSI_SD_E2E_RA_BUILD)/Vtb_scsi_sd_e2e
	@echo "Running scsi ↔ SD end-to-end tb WITH the vhdd read-ahead cache..."
	$(SCSI_SD_E2E_RA_BUILD)/Vtb_scsi_sd_e2e

$(SCSI_SD_E2E_RA_BUILD)/Vtb_scsi_sd_e2e: $(SCSI_SD_E2E_RTL) \
		$(RTL_DIR)/soc/vhdd_mux.v $(TB_DIR)/tb_scsi_sd_e2e.cpp
	$(call SCSI_SD_E2E_BUILD_RULE,$(SCSI_SD_E2E_RA_BUILD),+define+SCSI_E2E_READAHEAD)

.PHONY: tb-scsi-sd-e2e-c96-ra
tb-scsi-sd-e2e-c96-ra: $(SCSI_SD_E2E_C96_RA_BUILD)/Vtb_scsi_sd_e2e
	@echo "Running C96 scsi ↔ SD end-to-end tb WITH the vhdd read-ahead cache..."
	$(SCSI_SD_E2E_C96_RA_BUILD)/Vtb_scsi_sd_e2e

$(SCSI_SD_E2E_C96_RA_BUILD)/Vtb_scsi_sd_e2e: $(SCSI_SD_E2E_RTL) \
		$(RTL_DIR)/soc/vhdd_mux.v $(TB_DIR)/tb_scsi_sd_e2e.cpp
	$(call SCSI_SD_E2E_BUILD_RULE,$(SCSI_SD_E2E_C96_RA_BUILD),+define+SCSI_E2E_C96 +define+SCSI_E2E_READAHEAD -CFLAGS "-DSCSI_E2E_C96")

# ----------------------------------------------------------------------------
# peripheral_bus <-> SCSI integration testbench (T1b: scsi-pb-reconcile)
# (peripheral_bus.v + scsi.v wrapped by tb_pb_scsi.v)
#
# Drives REAL AXI4 transactions into a REAL peripheral_bus.v instance
# fanned out to a REAL scsi.v (TURBOSCSI_C96=1) -- every pre-existing SCSI
# tb drives scsi.v's pb_* ports directly, bypassing peripheral_bus.v, so
# none of them exercise the DRQ-check ack-withholding handshake, the
# ack-timeout watchdog, or the concurrent-R+W address-mux arbitration
# through the real peripheral_bus.v FSM.  See tb_pb_scsi.v / tb_pb_scsi.cpp
# headers for scenario detail.
# ----------------------------------------------------------------------------
PB_SCSI_RTL := \
	$(RTL_DIR)/mac/scsi.v \
	$(RTL_DIR)/soc/sd_scsi_lba_mapper.v \
	$(RTL_DIR)/soc/vhdd_sd.v \
	$(RTL_DIR)/soc/peripheral_bus.v \
	$(TB_DIR)/tb_pb_scsi.v
PB_SCSI_BUILD := $(BUILD_DIR)/pb_scsi

.PHONY: tb-pb-scsi
tb-pb-scsi: $(PB_SCSI_BUILD)/Vtb_pb_scsi
	@echo "Running peripheral_bus <-> SCSI integration tb..."
	$(PB_SCSI_BUILD)/Vtb_pb_scsi

$(PB_SCSI_BUILD)/Vtb_pb_scsi: $(PB_SCSI_RTL) $(TB_DIR)/tb_pb_scsi.cpp
	@mkdir -p $(PB_SCSI_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		--public-flat-rw \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/mac -I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(PB_SCSI_BUILD) \
		--top-module tb_pb_scsi \
		$(PB_SCSI_RTL) \
		$(TB_DIR)/tb_pb_scsi.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# 53C96 trace ring INTEGRATION testbench (tb-scsi-trace-pb)
#
# tb-scsi-trace-ring proves the ring's own logic against hand-driven beats.
# This one proves the WIRING that a `scsi-trace` user actually depends on:
# the ring hung off a real peripheral_bus.v -> real scsi.v port, read back
# through the real vhdd_ctrl register map at the offsets
# tools/jtag_repl.tcl uses.  The ring was dead code in the build for a
# month while its unit tb passed the whole time, which is exactly the class
# of failure only an integration tb can see.
#
# The harness builds an INDEPENDENT golden model from the snooped bus, so a
# tap on the wrong wire fails the comparison instead of agreeing with it.
#
# POSITIVE CONTROL runs FIRST, with cap_en=0 (capture disabled, traffic
# unchanged): the four capture tests must all fail, and the binary exits
# non-zero if they do not.
# ──────────────────────────────────────────────────────────────────────────────
SCSI_TRACE_PB_RTL := \
	$(RTL_DIR)/mac/scsi.v \
	$(RTL_DIR)/soc/sd_scsi_lba_mapper.v \
	$(RTL_DIR)/soc/vhdd_sd.v \
	$(RTL_DIR)/soc/peripheral_bus.v \
	$(RTL_DIR)/soc/scsi_trace_ring.v \
	$(RTL_DIR)/soc/vhdd_ctrl.v \
	$(TB_DIR)/tb_pb_scsi.v \
	$(TB_DIR)/tb_scsi_trace_pb.v
SCSI_TRACE_PB_BUILD := $(BUILD_DIR)/scsi_trace_pb

.PHONY: tb-scsi-trace-pb
tb-scsi-trace-pb: $(SCSI_TRACE_PB_BUILD)/Vtb_scsi_trace_pb
	@echo "Running scsi trace-ring INTEGRATION POSITIVE CONTROL (capture off, checks must FAIL)..."
	$(SCSI_TRACE_PB_BUILD)/Vtb_scsi_trace_pb --positive-control
	@echo "Running scsi trace-ring integration tb..."
	$(SCSI_TRACE_PB_BUILD)/Vtb_scsi_trace_pb

$(SCSI_TRACE_PB_BUILD)/Vtb_scsi_trace_pb: $(SCSI_TRACE_PB_RTL) $(TB_DIR)/tb_scsi_trace_pb.cpp
	@mkdir -p $(SCSI_TRACE_PB_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/mac -I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(SCSI_TRACE_PB_BUILD) \
		--top-module tb_scsi_trace_pb \
		$(SCSI_TRACE_PB_RTL) \
		$(TB_DIR)/tb_scsi_trace_pb.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# ASC unit testbench (SONORA sound chip — Q700 variant)
#
# Standalone Verilator build — asc.v in isolation.  Exercises reset state,
# FIFO write-pointer advance, sample-tick read-pointer advance, half-empty
# IRQ edge detection, IRQ read-to-ack on both per-FIFO and interrupt-status
# registers, stereo L/R interleave, rate-divider scaling, volume attenuation,
# silent-mode tick suspension, version-register SONORA constant, FIFO clear
# via fifo_ctl bit 7, and full-FIFO write saturation.
# ──────────────────────────────────────────────────────────────────────────────
ASC_RTL   := $(RTL_DIR)/mac/asc.v
ASC_BUILD := $(BUILD_DIR)/asc

.PHONY: tb-asc
tb-asc: $(ASC_BUILD)/Vasc
	@echo "Running asc unit tb..."
	$(ASC_BUILD)/Vasc

$(ASC_BUILD)/Vasc: $(ASC_RTL) $(TB_DIR)/tb_asc.cpp
	@mkdir -p $(ASC_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(ASC_BUILD) \
		--top-module asc \
		$(ASC_RTL) \
		$(TB_DIR)/tb_asc.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# ASC MAME-trace replay harness — drives the same Vasc model with a
# bus-trace TSV captured from MAME via tools/mame_asc_capture.lua, and
# emits a WAV.  Used to compare our RTL ASC's audio output against
# MAME's pure-software EASC when both are fed identical bus traffic.
# ──────────────────────────────────────────────────────────────────────────────
ASC_REPLAY_BUILD := $(BUILD_DIR)/asc_mame_replay

.PHONY: tb-asc-mame-replay
tb-asc-mame-replay: $(ASC_REPLAY_BUILD)/Vasc_replay
	@echo "Built $(ASC_REPLAY_BUILD)/Vasc_replay"
	@echo "Run with: $(ASC_REPLAY_BUILD)/Vasc_replay <trace.tsv> <out.wav>"

# End-to-end: capture MAME ASC bus trace + replay through RTL ASC + write WAV.
# Requires: mame in $$PATH; Q700 ROM at $(ROM); adbmodem ROM at files/342s0440-b.bin.
ASC_MAME_TRACE_DIR ?= $(BUILD_DIR)/asc_mame_replay
ASC_MAME_ROM_DIR   ?= $(ASC_MAME_TRACE_DIR)/roms
ASC_MAME_TRACE     ?= $(ASC_MAME_TRACE_DIR)/mame_asc_trace.tsv
ASC_MAME_WAV       ?= $(ASC_MAME_TRACE_DIR)/our_chime_mame_driven.wav
ASC_MAME_BASELINE  ?= $(ASC_MAME_TRACE_DIR)/mame_chime_baseline.wav

.PHONY: tb-asc-mame-trace
tb-asc-mame-trace:
	@mkdir -p $(ASC_MAME_ROM_DIR)/macqd700 $(ASC_MAME_ROM_DIR)/adbmodem
	@cp -n files/420dbff3.rom $(ASC_MAME_ROM_DIR)/macqd700/ 2>/dev/null || true
	@cp -n files/342s0440-b.bin $(ASC_MAME_ROM_DIR)/adbmodem/ 2>/dev/null || true
	@echo "[asc-mame-trace] capturing MAME-side ASC bus trace -> $(ASC_MAME_TRACE)"
	@cd $(ASC_MAME_TRACE_DIR) && \
	    SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy \
	    MAME_ASC_TRACE_OUT=$(ASC_MAME_TRACE) \
	    MAME_ASC_TRACE_LIMIT=200000 \
	    mame -rompath roms macqd700 \
	         -seconds_to_run 5 -nothrottle \
	         -video none -sound none -skip_gameinfo \
	         -autoboot_delay 0 \
	         -autoboot_script $(TOOLS_DIR)/mame_asc_capture.lua \
	         -plugins >$(ASC_MAME_TRACE_DIR)/mame.log 2>&1 || true
	@wc -l $(ASC_MAME_TRACE) | awk '{print "[asc-mame-trace] events captured: " $$1}'

.PHONY: tb-asc-mame-replay-run
tb-asc-mame-replay-run: $(ASC_REPLAY_BUILD)/Vasc_replay tb-asc-mame-trace
	@echo "[asc-mame-replay] driving RTL ASC -> $(ASC_MAME_WAV)"
	@$(ASC_REPLAY_BUILD)/Vasc_replay $(ASC_MAME_TRACE) $(ASC_MAME_WAV) \
	    --trailer-ms 1500

$(ASC_REPLAY_BUILD)/Vasc_replay: $(ASC_RTL) $(TB_DIR)/tb_asc_mame_replay.cpp
	@mkdir -p $(ASC_REPLAY_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(ASC_REPLAY_BUILD) \
		--top-module asc \
		-o Vasc_replay \
		$(ASC_RTL) \
		$(TB_DIR)/tb_asc_mame_replay.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# Audio I2S serialiser unit testbench (rtl/board/audio_i2s.v)
#
# Standalone Verilator build — audio_i2s.v alone.  Parameters are
# overridden to a scaled-down clock ratio (CORE_FREQ_HZ=16,
# I2S_BCLK_FREQ_HZ=4) so one stereo frame fits in ~256 core cycles —
# keeps the tb fast while still exercising the divider and bit-count
# FSM.  Task #123 (b).
# ──────────────────────────────────────────────────────────────────────────────
AUDIO_I2S_RTL   := $(RTL_DIR)/board/audio_i2s.v
AUDIO_I2S_BUILD := $(BUILD_DIR)/audio_i2s

.PHONY: tb-audio-i2s
tb-audio-i2s: $(AUDIO_I2S_BUILD)/Vaudio_i2s
	@echo "Running audio_i2s unit tb..."
	$(AUDIO_I2S_BUILD)/Vaudio_i2s

$(AUDIO_I2S_BUILD)/Vaudio_i2s: $(AUDIO_I2S_RTL) $(TB_DIR)/tb_audio_i2s.cpp
	@mkdir -p $(AUDIO_I2S_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-GCORE_FREQ_HZ=16 -GI2S_BCLK_FREQ_HZ=4 \
		-GBITS_PER_SAMPLE=16 -GSLOT_BITS=32 \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(AUDIO_I2S_BUILD) \
		--top-module audio_i2s \
		$(AUDIO_I2S_RTL) \
		$(TB_DIR)/tb_audio_i2s.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# AXI-Lite null-slave unit testbench (rtl/soc/axil_null_slave.v)
#
# Standalone Verilator build — axil_null_slave.v alone.  This is the
# terminator sitting on the xbar S2 "DMA config" window after dma_ctrl was
# removed (rtl/soc/fpga_top_dma.vh).  Its reason to exist is protocol
# LIVENESS — a stray access to a dead window must complete rather than
# park a burst on the interconnect — so the tb is about handshakes, not
# data: AW/W in either order or together (axi_wide_to_axilite drives them
# independently), backpressure, back-to-back, reset-drops-parked-response,
# and no combinational valid->ready path.
# ──────────────────────────────────────────────────────────────────────────────
AXIL_NULL_RTL   := $(RTL_DIR)/soc/axil_null_slave.v
AXIL_NULL_BUILD := $(BUILD_DIR)/axil_null_slave

.PHONY: tb-axil-null-slave
tb-axil-null-slave: $(AXIL_NULL_BUILD)/Vaxil_null_slave
	@echo "Running axil_null_slave unit tb..."
	$(AXIL_NULL_BUILD)/Vaxil_null_slave

$(AXIL_NULL_BUILD)/Vaxil_null_slave: $(AXIL_NULL_RTL) $(TB_DIR)/tb_axil_null_slave.cpp
	@mkdir -p $(AXIL_NULL_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(AXIL_NULL_BUILD) \
		--top-module axil_null_slave \
		$(AXIL_NULL_RTL) \
		$(TB_DIR)/tb_axil_null_slave.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# Audio PWM (Σ-Δ) bridge unit testbench (rtl/board/audio_pwm.v)
#
# Standalone Verilator build — audio_pwm.v alone.  First-order error-
# feedback Σ-Δ modulator on signed-16 PCM, 1 bit per channel.  Targets
# the AN9134 carrier where only J1.35/36 (NC) are free for audio.  See
# docs/superpowers/specs/2026-05-04-an9134-pwm-audio-design.md.
# ──────────────────────────────────────────────────────────────────────────────
AUDIO_PWM_RTL   := $(RTL_DIR)/board/audio_pwm.v
AUDIO_PWM_BUILD := $(BUILD_DIR)/audio_pwm

.PHONY: tb-audio-pwm
tb-audio-pwm: $(AUDIO_PWM_BUILD)/Vaudio_pwm
	@echo "Running audio_pwm unit tb..."
	$(AUDIO_PWM_BUILD)/Vaudio_pwm

$(AUDIO_PWM_BUILD)/Vaudio_pwm: $(AUDIO_PWM_RTL) $(TB_DIR)/tb_audio_pwm.cpp
	@mkdir -p $(AUDIO_PWM_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-GCORE_FREQ_HZ=50000000 \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(AUDIO_PWM_BUILD) \
		--top-module audio_pwm \
		$(AUDIO_PWM_RTL) \
		$(TB_DIR)/tb_audio_pwm.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# GLUE unit testbench (Quadra 700 address decoder + ROM overlay alias)
#
# Standalone Verilator build — glue.v has no dependencies on the core or
# any peripheral module.  Exercises the top-level RAM/ROM/IO/VIDEO/UNMAP
# classification, the overlay alias flop consumed from VIA1, the I/O
# sub-decode (VIA1/VIA2/SCC/SCSI/ASC/IWM/DAFB + ADB alias), glue_fault
# on unmapped accesses, and the priv_violate advisory path for I/O from
# user-mode.
# ──────────────────────────────────────────────────────────────────────────────
GLUE_RTL   := $(RTL_DIR)/mac/glue.v
GLUE_BUILD := $(BUILD_DIR)/glue

.PHONY: tb-glue
tb-glue: $(GLUE_BUILD)/Vglue
	@echo "Running glue unit tb..."
	$(GLUE_BUILD)/Vglue

$(GLUE_BUILD)/Vglue: $(GLUE_RTL) $(TB_DIR)/tb_glue.cpp
	@mkdir -p $(GLUE_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(GLUE_BUILD) \
		--top-module glue \
		$(GLUE_RTL) \
		$(TB_DIR)/tb_glue.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# VIA1 unit testbench (6522 ported from ~/6522/)
# ──────────────────────────────────────────────────────────────────────────────
VIA1_RTL   := $(RTL_DIR)/mac/via1.v
VIA1_BUILD := $(BUILD_DIR)/via1

.PHONY: tb-via1
tb-via1: $(VIA1_BUILD)/Vvia1
	@echo "Running via1 unit tb..."
	$(VIA1_BUILD)/Vvia1

$(VIA1_BUILD)/Vvia1: $(VIA1_RTL) $(TB_DIR)/tb_via1.cpp
	@mkdir -p $(VIA1_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(VIA1_BUILD) \
		--top-module via1 \
		$(VIA1_RTL) \
		$(TB_DIR)/tb_via1.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# PIC16C5x unit testbench (ADB PIC microcontroller core)
# ──────────────────────────────────────────────────────────────────────────────
PIC16C5X_RTL   := $(RTL_DIR)/mac/pic16c5x.v
PIC16C5X_BUILD := $(BUILD_DIR)/pic16c5x
PIC16C5X_HEX   := $(PIC16C5X_BUILD)/pic_test.hex

.PHONY: tb-pic16c5x
tb-pic16c5x: $(PIC16C5X_BUILD)/Vpic16c5x
	@echo "Running pic16c5x unit tb..."
	$(PIC16C5X_BUILD)/Vpic16c5x

# Rebuild synthetic code when its source changes. Port tests use the PIC1654S
# open-drain read rule (external input & latch), independent of TRIS.
$(PIC16C5X_HEX): Makefile
	@mkdir -p $(PIC16C5X_BUILD)
	@printf '%s\n' \
	"rom = [0x000] * 512" \
	"code = []" \
	"labels = {}" \
	"fixups = []" \
	"def label(name): labels[name] = len(code)" \
	"def emit(word): code.append(word & 0xfff)" \
	"def movlw(k): emit(0xc00 | (k & 0xff))" \
	"def movwf(f): emit(0x020 | (f & 0x1f))" \
	"def clrf(f): emit(0x060 | (f & 0x1f))" \
	"def tris(f): emit(f & 0x00f)" \
	"def op6(op, d, f): emit((op << 6) | ((d & 1) << 5) | (f & 0x1f))" \
	"def bitop(op, b, f): emit((op << 8) | ((b & 7) << 5) | (f & 0x1f))" \
	"def xorlw(k): emit(0xf00 | (k & 0xff))" \
	"def call(name): fixups.append((len(code), 'call', name)); emit(0)" \
	"def goto(name): fixups.append((len(code), 'goto', name)); emit(0)" \
	"def retlw(k): emit(0x800 | (k & 0xff))" \
	"def check_z_set(): bitop(0x7, 2, 0x03); goto('fail')" \
	"def check_c_set(): bitop(0x7, 0, 0x03); goto('fail')" \
	"def check_c_clear(): bitop(0x6, 0, 0x03); goto('fail')" \
	"def pass_code(n): movlw(0x80 | n); movwf(0x06)" \
	"label('start')" \
	"movlw(0x00); tris(0x06)" \
	"movlw(0x5a); movwf(0x07); emit(0x040); op6(0x08, 0, 0x07); xorlw(0x5a); check_z_set(); pass_code(1)" \
	"movlw(0x01); movwf(0x08); movlw(0xff); op6(0x07, 0, 0x08); check_c_set(); check_z_set(); pass_code(2)" \
	"movlw(0x05); movwf(0x09); movlw(0x03); op6(0x02, 0, 0x09); xorlw(0x02); check_z_set(); check_c_set()" \
	"movlw(0x01); movwf(0x09); movlw(0x02); op6(0x02, 0, 0x09); check_c_clear(); pass_code(3)" \
	"clrf(0x0a); bitop(0x5, 3, 0x0a); bitop(0x4, 3, 0x0a); op6(0x08, 0, 0x0a); xorlw(0x00); check_z_set(); pass_code(4)" \
	"bitop(0x5, 2, 0x0a); bitop(0x7, 2, 0x0a); goto('fail'); pass_code(5)" \
	"goto('goto_ok'); goto('fail'); label('goto_ok'); pass_code(6)" \
	"call('sub_outer'); xorlw(0x77); check_z_set(); pass_code(7)" \
	"movlw(0xff); movwf(0x0b); op6(0x0f, 1, 0x0b); goto('fail'); pass_code(8)" \
	"movlw(0x01); movwf(0x0c); movlw(0xff); op6(0x07, 0, 0x0c)" \
	"movlw(0x80); movwf(0x0c); op6(0x0d, 1, 0x0c); check_c_set(); op6(0x08, 0, 0x0c); xorlw(0x01); check_z_set(); pass_code(9)" \
	"movlw(0x0d); movwf(0x04); movlw(0x42); movwf(0x00); op6(0x08, 0, 0x0d); xorlw(0x42); check_z_set(); pass_code(10)" \
	"movlw(0x0f); tris(0x05); pass_code(11)" \
	"movlw(0x00); tris(0x06); movlw(0xa5); movwf(0x06); op6(0x08, 0, 0x06); xorlw(0x24); check_z_set()" \
	"movlw(0xff); tris(0x06); op6(0x08, 0, 0x06); xorlw(0x24); check_z_set(); pass_code(12)" \
	"movlw(0x02); op6(0x07, 1, 0x02); goto('fail'); goto('fail'); pass_code(13)" \
	"label('done'); goto('done')" \
	"label('sub_outer'); call('sub_inner'); xorlw(0x33); check_z_set(); retlw(0x77)" \
	"label('sub_inner'); retlw(0x33)" \
	"label('fail'); movlw(0x00); tris(0x06); movlw(0xee); movwf(0x06); goto('fail')" \
	"for i, kind, name in fixups:" \
	"    code[i] = (0xa00 | (labels[name] & 0x1ff)) if kind == 'goto' else (0x900 | (labels[name] & 0xff))" \
	"for i, word in enumerate(code): rom[i] = word" \
	"rom[0x1ff] = 0xa00 | labels['start']" \
	"for word in rom: print('%03x' % word)" | python3 > $@.tmp && mv $@.tmp $@

$(PIC16C5X_BUILD)/Vpic16c5x: $(PIC16C5X_RTL) $(TB_DIR)/tb_pic16c5x.cpp $(PIC16C5X_HEX)
	@mkdir -p $(PIC16C5X_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(PIC16C5X_BUILD) \
		--top-module pic16c5x \
		-GPROGHEX=\"$(PIC16C5X_HEX)\" \
		$(PIC16C5X_RTL) \
		$(TB_DIR)/tb_pic16c5x.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# RTC unit testbench (off-chip real-time clock + PRAM)
# ──────────────────────────────────────────────────────────────────────────────
RTC_RTL   := $(RTL_DIR)/mac/rtc.v
RTC_BUILD := $(BUILD_DIR)/rtc

.PHONY: tb-rtc
tb-rtc: $(RTC_BUILD)/Vrtc
	@echo "Running rtc unit tb..."
	$(RTC_BUILD)/Vrtc

$(RTC_BUILD)/Vrtc: $(RTC_RTL) $(TB_DIR)/tb_rtc.cpp
	@mkdir -p $(RTC_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(RTC_BUILD) \
		--top-module rtc \
		$(RTC_RTL) \
		$(TB_DIR)/tb_rtc.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# ADB unit testbench (modem + keyboard + mouse + injection MMIO)
# ──────────────────────────────────────────────────────────────────────────────
ADB_RTL    := $(RTL_DIR)/mac/adb_modem.v $(RTL_DIR)/mac/adb_keyboard.v $(RTL_DIR)/mac/adb_mouse.v
ADB_TB_RTL := $(TB_DIR)/tb_adb.v
ADB_BUILD  := $(BUILD_DIR)/adb

.PHONY: tb-adb
tb-adb: $(ADB_BUILD)/Vtb_adb
	@echo "Running adb unit tb..."
	$(ADB_BUILD)/Vtb_adb

$(ADB_BUILD)/Vtb_adb: $(ADB_RTL) $(ADB_TB_RTL) $(TB_DIR)/tb_adb.cpp
	@mkdir -p $(ADB_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(ADB_BUILD) \
		--top-module tb_adb \
		$(ADB_RTL) $(ADB_TB_RTL) \
		$(TB_DIR)/tb_adb.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# ADB bit-level PHY unit testbench (adb_phy.v + adb_keyboard.v + adb_mouse.v) —
# drives real ADB-bus bit framing (Attention/Sync/cmd/stop + response
# start/16-data/stop), independent of the byte-level adb_modem.v path
# tb-adb covers.  See rtl/mac/adb_phy.v's header for why this path is
# the one real HW boot actually exercises.
# ──────────────────────────────────────────────────────────────────────────────
ADB_PHY_RTL    := $(RTL_DIR)/mac/adb_phy.v $(RTL_DIR)/mac/adb_keyboard.v $(RTL_DIR)/mac/adb_mouse.v
ADB_PHY_TB_RTL := $(TB_DIR)/tb_adb_phy.v
ADB_PHY_BUILD  := $(BUILD_DIR)/adb_phy

.PHONY: tb-adb-phy
tb-adb-phy: $(ADB_PHY_BUILD)/Vtb_adb_phy
	@echo "Running adb_phy bit-level unit tb..."
	$(ADB_PHY_BUILD)/Vtb_adb_phy

$(ADB_PHY_BUILD)/Vtb_adb_phy: $(ADB_PHY_RTL) $(ADB_PHY_TB_RTL) $(TB_DIR)/tb_adb_phy.cpp
	@mkdir -p $(ADB_PHY_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(ADB_PHY_BUILD) \
		--top-module tb_adb_phy \
		$(ADB_PHY_RTL) $(ADB_PHY_TB_RTL) \
		$(TB_DIR)/tb_adb_phy.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# REAL-FIRMWARE ADB chain integration testbench: the genuine 342S0440-B
# PIC firmware (adb_pic_modem.v + pic16c5x.v + rtl/mac/adb_pic_fw.hex) is
# the ADB HOST driving adb_phy.v + adb_keyboard.v + adb_mouse.v, with the
# C++ side playing the VIA1/68k role at the CB1/CB2/state-pin level.
# Covers what tb-adb-phy structurally cannot: timing/framing agreement
# between adb_phy.v and the real firmware, the firmware's autonomous
# idle-state autopoll, SRQ signalling, and the unsolicited VIA-SR
# notification path — the 2026-07-21 "injected mouse event never drains /
# boot stalls on VIA1 SR polling" hardware bug regression.
# ──────────────────────────────────────────────────────────────────────────────
ADB_PICPHY_RTL    := $(RTL_DIR)/mac/adb_pic_modem.v $(RTL_DIR)/mac/pic16c5x.v \
                     $(RTL_DIR)/mac/adb_phy.v $(RTL_DIR)/mac/adb_keyboard.v \
                     $(RTL_DIR)/mac/adb_mouse.v
ADB_PICPHY_TB_RTL := $(TB_DIR)/tb_adb_pic_phy.v
ADB_PICPHY_BUILD  := $(BUILD_DIR)/adb_pic_phy

.PHONY: prepare-adb-firmware tb-adb-pic-phy
prepare-adb-firmware:
	python3 $(TOOLS_DIR)/prepare_adb_firmware.py "$(MAME_ADB_ROM)"

$(RTL_DIR)/mac/adb_pic_fw.hex: $(TOOLS_DIR)/prepare_adb_firmware.py
	python3 $(TOOLS_DIR)/prepare_adb_firmware.py "$(MAME_ADB_ROM)"

tb-adb-pic-phy: $(ADB_PICPHY_BUILD)/Vtb_adb_pic_phy
	@echo "Running real-firmware ADB chain integration tb..."
	$(ADB_PICPHY_BUILD)/Vtb_adb_pic_phy

$(ADB_PICPHY_BUILD)/Vtb_adb_pic_phy: $(ADB_PICPHY_RTL) $(ADB_PICPHY_TB_RTL) \
		$(TB_DIR)/tb_adb_pic_phy.cpp $(RTL_DIR)/mac/adb_pic_fw.hex
	@mkdir -p $(ADB_PICPHY_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(ADB_PICPHY_BUILD) \
		--top-module tb_adb_pic_phy \
		$(ADB_PICPHY_RTL) $(ADB_PICPHY_TB_RTL) \
		$(TB_DIR)/tb_adb_pic_phy.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# ADB event-injection unit testbench (adb_inject.v + adb_keyboard.v +
# adb_mouse.v) — covers the host-side MMIO producer (JTAG-AXI / m68k /
# future USB-HID bridge byte writes through the pb_* face, including the
# word-aligned alias registers) feeding the device models, and the
# device-bus TALK-poll consumer path.  tb-adb-phy covers the bit-level
# framing below this; tb-peripheral-bus covers the AXI decode above it.
# ──────────────────────────────────────────────────────────────────────────────
ADB_INJ_RTL    := $(RTL_DIR)/mac/adb_inject.v $(RTL_DIR)/mac/adb_keyboard.v $(RTL_DIR)/mac/adb_mouse.v
ADB_INJ_TB_RTL := $(TB_DIR)/tb_adb_inject.v
ADB_INJ_BUILD  := $(BUILD_DIR)/adb_inject

.PHONY: tb-adb-inject
tb-adb-inject: $(ADB_INJ_BUILD)/Vtb_adb_inject
	@echo "Running adb_inject unit tb..."
	$(ADB_INJ_BUILD)/Vtb_adb_inject

$(ADB_INJ_BUILD)/Vtb_adb_inject: $(ADB_INJ_RTL) $(ADB_INJ_TB_RTL) $(TB_DIR)/tb_adb_inject.cpp
	@mkdir -p $(ADB_INJ_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(ADB_INJ_BUILD) \
		--top-module tb_adb_inject \
		$(ADB_INJ_RTL) $(ADB_INJ_TB_RTL) \
		$(TB_DIR)/tb_adb_inject.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# Q700 Ethernet PROM + SONIC register-block unit testbench.
# ──────────────────────────────────────────────────────────────────────────────
Q700_ETH_SONIC_RTL   := $(RTL_DIR)/mac/q700_eth_sonic.v
Q700_ETH_SONIC_BUILD := $(BUILD_DIR)/q700_eth_sonic
Q700_SONIC_TX_RTL    := $(RTL_DIR)/mac/q700_sonic_tx.sv
Q700_SONIC_TX_BUILD  := $(BUILD_DIR)/q700_sonic_tx
Q700_SONIC_RX_RTL    := $(RTL_DIR)/mac/q700_sonic_rx.sv
Q700_SONIC_RX_BUILD  := $(BUILD_DIR)/q700_sonic_rx
Q700_ETH_SHARE_RTL   := $(RTL_DIR)/board/q700_eth_link.sv
Q700_ETH_SHARE_BUILD := $(BUILD_DIR)/q700_eth_stream_share
NET_BLOCK_FRAMER_RTL   := $(RTL_DIR)/board/net_block_framer.sv
NET_BLOCK_FRAMER_BUILD := $(BUILD_DIR)/net_block_framer
VHDD_NET_RTL           := $(RTL_DIR)/board/vhdd_net.sv
VHDD_NET_BUILD         := $(BUILD_DIR)/vhdd_net

.PHONY: tb-q700-eth-stream-share
tb-q700-eth-stream-share: $(Q700_ETH_SHARE_BUILD)/Vq700_eth_stream_share
	$(Q700_ETH_SHARE_BUILD)/Vq700_eth_stream_share

# ── toggle-per-event CDC receiver (race audit 2026-09-18) ───────────────
# q700_toggle_rx lives in q700_eth_link.sv but OUTSIDE its `ifdef ETH_ENABLE
# guard, precisely so it can be unit-tested without the vendor MAC/MMCM/
# IDELAY primitives.  The -mut target is the negative control: it rebuilds
# the module with the reset-skew priming deleted and REQUIRES the test to
# reject it.
Q700_TOGGLE_RX_BUILD := $(BUILD_DIR)/q700_toggle_rx

.PHONY: tb-q700-toggle-rx
tb-q700-toggle-rx: $(Q700_TOGGLE_RX_BUILD)/Vq700_toggle_rx
	$(Q700_TOGGLE_RX_BUILD)/Vq700_toggle_rx

.PHONY: tb-q700-toggle-rx-mut
tb-q700-toggle-rx-mut: $(Q700_TOGGLE_RX_BUILD)/mut/Vq700_toggle_rx
	@if $(Q700_TOGGLE_RX_BUILD)/mut/Vq700_toggle_rx >/dev/null 2>&1; then \
		echo "ERROR: no-priming mutant unexpectedly passed -- the test is not a negative control"; exit 1; \
	else echo "PASS: test rejects the no-priming mutant"; fi

$(Q700_TOGGLE_RX_BUILD)/Vq700_toggle_rx: $(Q700_ETH_SHARE_RTL) $(TB_DIR)/tb_q700_toggle_rx.cpp
	@mkdir -p $(Q700_TOGGLE_RX_BUILD)
	$(VERILATOR) --cc --exe --build --assert --x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_TOGGLE_RX_BUILD) --top-module q700_toggle_rx \
		$(Q700_ETH_SHARE_RTL) $(TB_DIR)/tb_q700_toggle_rx.cpp \
		-CFLAGS "-std=c++17"

$(Q700_TOGGLE_RX_BUILD)/mut/Vq700_toggle_rx: $(Q700_ETH_SHARE_RTL) $(TB_DIR)/tb_q700_toggle_rx.cpp
	@mkdir -p $(Q700_TOGGLE_RX_BUILD)/mut
	$(VERILATOR) --cc --exe --build --assert -DQ700_TOGGLE_RX_MUTANT_NO_PRIME \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_TOGGLE_RX_BUILD)/mut --top-module q700_toggle_rx \
		$(Q700_ETH_SHARE_RTL) $(TB_DIR)/tb_q700_toggle_rx.cpp \
		-CFLAGS "-std=c++17"

# RED-VERIFY: each mutant breaks one load-bearing rule of the shared MAC.  The
# target PASSES when the unit test REJECTS the mutant -- a test that survives its
# own mutant proves nothing.
.PHONY: tb-q700-eth-stream-share-mut
tb-q700-eth-stream-share-mut: $(Q700_ETH_SHARE_BUILD)/mut/Vq700_eth_stream_share \
		$(Q700_ETH_SHARE_BUILD)/sticky_mut/Vq700_eth_stream_share \
		$(Q700_ETH_SHARE_BUILD)/rawlb_mut/Vq700_eth_stream_share \
		$(Q700_ETH_SHARE_BUILD)/bcast_mut/Vq700_eth_stream_share
	@if $(Q700_ETH_SHARE_BUILD)/mut/Vq700_eth_stream_share >/dev/null 2>&1; then \
		echo "ERROR: beat-granular TX arbiter mutant unexpectedly passed"; exit 1; \
	else echo "PASS: test rejects beat-granular TX arbiter mutant"; fi
	@if $(Q700_ETH_SHARE_BUILD)/sticky_mut/Vq700_eth_stream_share >/dev/null 2>&1; then \
		echo "ERROR: sticky-grant mutant unexpectedly passed"; exit 1; \
	else echo "PASS: test rejects sticky-grant mutant"; fi
	@if $(Q700_ETH_SHARE_BUILD)/rawlb_mut/Vq700_eth_stream_share >/dev/null 2>&1; then \
		echo "ERROR: unlatched-loopback mutant unexpectedly passed"; exit 1; \
	else echo "PASS: test rejects unlatched-loopback mutant"; fi
	@if $(Q700_ETH_SHARE_BUILD)/bcast_mut/Vq700_eth_stream_share >/dev/null 2>&1; then \
		echo "ERROR: absent-client broadcast mutant unexpectedly passed"; exit 1; \
	else echo "PASS: test rejects absent-client broadcast mutant"; fi

$(Q700_ETH_SHARE_BUILD)/Vq700_eth_stream_share: $(Q700_ETH_SHARE_RTL) $(TB_DIR)/tb_q700_eth_stream_share.cpp
	@mkdir -p $(Q700_ETH_SHARE_BUILD)
	$(VERILATOR) --cc --exe --build --assert --x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_ETH_SHARE_BUILD) --top-module q700_eth_stream_share \
		$(Q700_ETH_SHARE_RTL) $(TB_DIR)/tb_q700_eth_stream_share.cpp \
		-CFLAGS "-std=c++17"

$(Q700_ETH_SHARE_BUILD)/mut/Vq700_eth_stream_share: $(Q700_ETH_SHARE_RTL) $(TB_DIR)/tb_q700_eth_stream_share.cpp
	@mkdir -p $(Q700_ETH_SHARE_BUILD)/mut
	$(VERILATOR) --cc --exe --build --assert -DQ700_ETH_SHARE_MUTANT_BEAT_ARBITER \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_ETH_SHARE_BUILD)/mut --top-module q700_eth_stream_share \
		$(Q700_ETH_SHARE_RTL) $(TB_DIR)/tb_q700_eth_stream_share.cpp \
		-CFLAGS "-std=c++17"

$(Q700_ETH_SHARE_BUILD)/sticky_mut/Vq700_eth_stream_share: $(Q700_ETH_SHARE_RTL) $(TB_DIR)/tb_q700_eth_stream_share.cpp
	@mkdir -p $(Q700_ETH_SHARE_BUILD)/sticky_mut
	$(VERILATOR) --cc --exe --build --assert -DQ700_ETH_SHARE_MUTANT_STICKY_GRANT \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_ETH_SHARE_BUILD)/sticky_mut --top-module q700_eth_stream_share \
		$(Q700_ETH_SHARE_RTL) $(TB_DIR)/tb_q700_eth_stream_share.cpp \
		-CFLAGS "-std=c++17"

$(Q700_ETH_SHARE_BUILD)/rawlb_mut/Vq700_eth_stream_share: $(Q700_ETH_SHARE_RTL) $(TB_DIR)/tb_q700_eth_stream_share.cpp
	@mkdir -p $(Q700_ETH_SHARE_BUILD)/rawlb_mut
	$(VERILATOR) --cc --exe --build --assert -DQ700_ETH_SHARE_MUTANT_RAW_LOOPBACK \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_ETH_SHARE_BUILD)/rawlb_mut --top-module q700_eth_stream_share \
		$(Q700_ETH_SHARE_RTL) $(TB_DIR)/tb_q700_eth_stream_share.cpp \
		-CFLAGS "-std=c++17"

$(Q700_ETH_SHARE_BUILD)/bcast_mut/Vq700_eth_stream_share: $(Q700_ETH_SHARE_RTL) $(TB_DIR)/tb_q700_eth_stream_share.cpp
	@mkdir -p $(Q700_ETH_SHARE_BUILD)/bcast_mut
	$(VERILATOR) --cc --exe --build --assert -DQ700_ETH_SHARE_MUTANT_BCAST_ABSENT \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_ETH_SHARE_BUILD)/bcast_mut --top-module q700_eth_stream_share \
		$(Q700_ETH_SHARE_RTL) $(TB_DIR)/tb_q700_eth_stream_share.cpp \
		-CFLAGS "-std=c++17"

.PHONY: tb-net-block-framer
tb-net-block-framer: $(NET_BLOCK_FRAMER_BUILD)/Vnet_block_framer
	$(NET_BLOCK_FRAMER_BUILD)/Vnet_block_framer

$(NET_BLOCK_FRAMER_BUILD)/Vnet_block_framer: $(NET_BLOCK_FRAMER_RTL) $(TB_DIR)/tb_net_block_framer.cpp
	@mkdir -p $(NET_BLOCK_FRAMER_BUILD)
	$(VERILATOR) --cc --exe --build --assert --x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(NET_BLOCK_FRAMER_BUILD) --top-module net_block_framer \
		$(NET_BLOCK_FRAMER_RTL) $(TB_DIR)/tb_net_block_framer.cpp -CFLAGS "-std=c++17"

.PHONY: tb-vhdd-net
tb-vhdd-net: $(VHDD_NET_BUILD)/Vvhdd_net
	@# Positive control FIRST: with the host disconnected every data check
	@# must fail, which is what makes the real run's silence meaningful.
	@# The binary itself reports the inversion -- it exits NON-zero if the
	@# control run failed to fail -- so the exit code just propagates.
	$(VHDD_NET_BUILD)/Vvhdd_net control
	$(VHDD_NET_BUILD)/Vvhdd_net

# Short timeouts so the bounded-response paths (retransmit, retry
# exhaustion, master stall) are reachable in a few thousand cycles instead
# of the seconds-scale values the real design uses.
$(VHDD_NET_BUILD)/Vvhdd_net: $(VHDD_NET_RTL) $(TB_DIR)/tb_vhdd_net.cpp
	@mkdir -p $(VHDD_NET_BUILD)
	$(VERILATOR) --cc --exe --build --assert --x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-GREPLY_TIMEOUT=2000 -GSTALL_TIMEOUT=20000 -GMAX_RETRIES=3 \
		-Mdir $(VHDD_NET_BUILD) --top-module vhdd_net \
		$(VHDD_NET_RTL) $(TB_DIR)/tb_vhdd_net.cpp -CFLAGS "-std=c++17"

.PHONY: tb-q700-sonic-rx
tb-q700-sonic-rx: $(Q700_SONIC_RX_BUILD)/Vq700_sonic_rx
	$(Q700_SONIC_RX_BUILD)/Vq700_sonic_rx
	$(Q700_SONIC_RX_BUILD)/Vq700_sonic_rx wide
	@for mode in narrow wide; do \
	  for length in 60 61 62 63 64 124 128; do \
	    $(Q700_SONIC_RX_BUILD)/Vq700_sonic_rx $$mode $$length || exit $$?; \
	  done; \
	done

.PHONY: tb-q700-sonic-rx-mut
tb-q700-sonic-rx-mut: $(Q700_SONIC_RX_BUILD)/mut/Vq700_sonic_rx
	@if $(Q700_SONIC_RX_BUILD)/mut/Vq700_sonic_rx >/dev/null 2>&1; then \
		echo "ERROR: early-RDA RX mutant unexpectedly passed"; exit 1; \
	else echo "PASS: q700_sonic_rx test rejects early-RDA mutant"; fi

.PHONY: tb-q700-sonic-rx-cam-mut
tb-q700-sonic-rx-cam-mut: $(Q700_SONIC_RX_BUILD)/cam_mut/Vq700_sonic_rx $(Q700_SONIC_RX_BUILD)/fcs_mut/Vq700_sonic_rx $(Q700_SONIC_RX_BUILD)/lookahead_mut/Vq700_sonic_rx
	@if $(Q700_SONIC_RX_BUILD)/cam_mut/Vq700_sonic_rx >/dev/null 2>&1; then \
		echo "ERROR: unswapped-CAM RX mutant unexpectedly passed"; exit 1; \
	else echo "PASS: q700_sonic_rx test rejects unswapped-CAM mutant"; fi
	@if $(Q700_SONIC_RX_BUILD)/fcs_mut/Vq700_sonic_rx >/dev/null 2>&1; then \
		echo "ERROR: missing-FCS RX mutant unexpectedly passed"; exit 1; \
	else echo "PASS: q700_sonic_rx test rejects missing-FCS mutant"; fi
	@if $(Q700_SONIC_RX_BUILD)/lookahead_mut/Vq700_sonic_rx >/dev/null 2>&1; then \
		echo "ERROR: short-lookahead RX mutant unexpectedly passed"; exit 1; \
	else echo "PASS: q700_sonic_rx test rejects short-lookahead mutant"; fi

$(Q700_SONIC_RX_BUILD)/fcs_mut/Vq700_sonic_rx: $(Q700_SONIC_RX_RTL) $(TB_DIR)/tb_q700_sonic_rx.cpp
	@mkdir -p $(Q700_SONIC_RX_BUILD)/fcs_mut
	$(VERILATOR) --cc --exe --build --assert -DSONIC_RX_MUTANT_NO_FCS \
		--x-assign fast --x-initial fast -O3 -Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_SONIC_RX_BUILD)/fcs_mut --top-module q700_sonic_rx \
		$(Q700_SONIC_RX_RTL) $(TB_DIR)/tb_q700_sonic_rx.cpp -CFLAGS "-std=c++17"

$(Q700_SONIC_RX_BUILD)/lookahead_mut/Vq700_sonic_rx: $(Q700_SONIC_RX_RTL) $(TB_DIR)/tb_q700_sonic_rx.cpp
	@mkdir -p $(Q700_SONIC_RX_BUILD)/lookahead_mut
	$(VERILATOR) --cc --exe --build --assert -DSONIC_RX_MUTANT_SHORT_LOOKAHEAD \
		--x-assign fast --x-initial fast -O3 -Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_SONIC_RX_BUILD)/lookahead_mut --top-module q700_sonic_rx \
		$(Q700_SONIC_RX_RTL) $(TB_DIR)/tb_q700_sonic_rx.cpp -CFLAGS "-std=c++17"

$(Q700_SONIC_RX_BUILD)/cam_mut/Vq700_sonic_rx: $(Q700_SONIC_RX_RTL) $(TB_DIR)/tb_q700_sonic_rx.cpp
	@mkdir -p $(Q700_SONIC_RX_BUILD)/cam_mut
	$(VERILATOR) --cc --exe --build --assert -DSONIC_RX_MUTANT_CAM_NO_SWAP \
		--x-assign fast --x-initial fast -O3 -Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_SONIC_RX_BUILD)/cam_mut --top-module q700_sonic_rx \
		$(Q700_SONIC_RX_RTL) $(TB_DIR)/tb_q700_sonic_rx.cpp -CFLAGS "-std=c++17"

$(Q700_SONIC_RX_BUILD)/mut/Vq700_sonic_rx: $(Q700_SONIC_RX_RTL) $(TB_DIR)/tb_q700_sonic_rx.cpp
	@mkdir -p $(Q700_SONIC_RX_BUILD)/mut
	$(VERILATOR) --cc --exe --build --assert -DSONIC_RX_MUTANT_RDA_EARLY \
		--x-assign fast --x-initial fast -O3 -Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_SONIC_RX_BUILD)/mut --top-module q700_sonic_rx \
		$(Q700_SONIC_RX_RTL) $(TB_DIR)/tb_q700_sonic_rx.cpp -CFLAGS "-std=c++17"

$(Q700_SONIC_RX_BUILD)/Vq700_sonic_rx: $(Q700_SONIC_RX_RTL) $(TB_DIR)/tb_q700_sonic_rx.cpp
	@mkdir -p $(Q700_SONIC_RX_BUILD)
	$(VERILATOR) --cc --exe --build --assert --x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_SONIC_RX_BUILD) --top-module q700_sonic_rx \
		$(Q700_SONIC_RX_RTL) $(TB_DIR)/tb_q700_sonic_rx.cpp -CFLAGS "-std=c++17"

.PHONY: tb-q700-sonic-tx
tb-q700-sonic-tx: $(Q700_SONIC_TX_BUILD)/Vq700_sonic_tx
	$(Q700_SONIC_TX_BUILD)/Vq700_sonic_tx

.PHONY: tb-q700-sonic-tx-mut
tb-q700-sonic-tx-mut: $(Q700_SONIC_TX_BUILD)/link_mut/Vq700_sonic_tx $(Q700_SONIC_TX_BUILD)/prime_mut/Vq700_sonic_tx
	@if $(Q700_SONIC_TX_BUILD)/link_mut/Vq700_sonic_tx >/dev/null 2>&1; then \
		echo "ERROR: link-address CTDA mutant unexpectedly passed"; exit 1; \
	else echo "PASS: q700_sonic_tx test rejects link-address CTDA mutant"; fi
	@if $(Q700_SONIC_TX_BUILD)/prime_mut/Vq700_sonic_tx >/dev/null 2>&1; then \
		echo "ERROR: no-prime packet-BRAM mutant unexpectedly passed"; exit 1; \
	else echo "PASS: q700_sonic_tx test rejects no-prime packet-BRAM mutant"; fi

$(Q700_SONIC_TX_BUILD)/link_mut/Vq700_sonic_tx: $(Q700_SONIC_TX_RTL) $(TB_DIR)/tb_q700_sonic_tx.cpp
	@mkdir -p $(Q700_SONIC_TX_BUILD)/link_mut
	$(VERILATOR) --cc --exe --build --assert -DSONIC_TX_MUTANT_LINK_ADDR \
		--x-assign fast --x-initial fast -O3 -Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_SONIC_TX_BUILD)/link_mut --top-module q700_sonic_tx \
		$(Q700_SONIC_TX_RTL) $(TB_DIR)/tb_q700_sonic_tx.cpp -CFLAGS "-std=c++17"

$(Q700_SONIC_TX_BUILD)/prime_mut/Vq700_sonic_tx: $(Q700_SONIC_TX_RTL) $(TB_DIR)/tb_q700_sonic_tx.cpp
	@mkdir -p $(Q700_SONIC_TX_BUILD)/prime_mut
	$(VERILATOR) --cc --exe --build --assert -DSONIC_TX_MUTANT_NO_PRIME \
		--x-assign fast --x-initial fast -O3 -Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_SONIC_TX_BUILD)/prime_mut --top-module q700_sonic_tx \
		$(Q700_SONIC_TX_RTL) $(TB_DIR)/tb_q700_sonic_tx.cpp -CFLAGS "-std=c++17"

$(Q700_SONIC_TX_BUILD)/Vq700_sonic_tx: $(Q700_SONIC_TX_RTL) $(TB_DIR)/tb_q700_sonic_tx.cpp
	@mkdir -p $(Q700_SONIC_TX_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_SONIC_TX_BUILD) --top-module q700_sonic_tx \
		$(Q700_SONIC_TX_RTL) $(TB_DIR)/tb_q700_sonic_tx.cpp \
		-CFLAGS "-std=c++17"

.PHONY: tb-q700-eth-sonic
tb-q700-eth-sonic: $(Q700_ETH_SONIC_BUILD)/Vq700_eth_sonic
	@echo "Running q700_eth_sonic unit tb..."
	$(Q700_ETH_SONIC_BUILD)/Vq700_eth_sonic

$(Q700_ETH_SONIC_BUILD)/Vq700_eth_sonic: $(Q700_ETH_SONIC_RTL) $(TB_DIR)/tb_q700_eth_sonic.cpp
	@mkdir -p $(Q700_ETH_SONIC_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(Q700_ETH_SONIC_BUILD) \
		--top-module q700_eth_sonic \
		$(Q700_ETH_SONIC_RTL) \
		$(TB_DIR)/tb_q700_eth_sonic.cpp \
		-CFLAGS "-std=c++17"

ETH_DEBUG_BUILD := $(BUILD_DIR)/eth_debug_regs
.PHONY: tb-eth-debug-regs
tb-eth-debug-regs: $(ETH_DEBUG_BUILD)/Veth_debug_regs
	@echo "Running Ethernet debug CSR unit tb..."
	$(ETH_DEBUG_BUILD)/Veth_debug_regs

$(ETH_DEBUG_BUILD)/Veth_debug_regs: $(RTL_DIR)/soc/eth_debug_regs.sv $(TB_DIR)/tb_eth_debug_regs.cpp
	@mkdir -p $(ETH_DEBUG_BUILD)
	$(VERILATOR) --cc --exe --build --assert --x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-DECLFILENAME \
		-Mdir $(ETH_DEBUG_BUILD) --top-module eth_debug_regs \
		$(RTL_DIR)/soc/eth_debug_regs.sv $(TB_DIR)/tb_eth_debug_regs.cpp \
		-CFLAGS "-std=c++17"

.PHONY: tb-q700-eth-sonic-engine
# Keep variants as siblings: Verilator's VPATH includes "..", so nesting
# under the stub build can reuse its C++ object without PACKET_ENGINE_TEST.
tb-q700-eth-sonic-engine: $(Q700_ETH_SONIC_BUILD)_engine/Vq700_eth_sonic
	$(Q700_ETH_SONIC_BUILD)_engine/Vq700_eth_sonic

$(Q700_ETH_SONIC_BUILD)_engine/Vq700_eth_sonic: $(Q700_ETH_SONIC_RTL) $(TB_DIR)/tb_q700_eth_sonic.cpp
	@mkdir -p $(Q700_ETH_SONIC_BUILD)_engine
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
		-Mdir $(Q700_ETH_SONIC_BUILD)_engine --top-module q700_eth_sonic \
		-GPACKET_ENGINE=1 $(Q700_ETH_SONIC_RTL) $(TB_DIR)/tb_q700_eth_sonic.cpp \
		-CFLAGS "-std=c++17 -DPACKET_ENGINE_TEST"

# ──────────────────────────────────────────────────────────────────────────────
# SWIM/IWM probe-safe stub unit testbench.
#
# Standalone Verilator build: rtl/mac/iwm_stub.v exercises the conservative
# floppy controller contract used by tb_rom_boot.cpp and the live platform
# peripheral bus.
# ──────────────────────────────────────────────────────────────────────────────
IWM_STUB_RTL   := $(RTL_DIR)/mac/iwm_stub.v
IWM_STUB_BUILD := $(BUILD_DIR)/iwm_stub

.PHONY: tb-iwm
tb-iwm: $(IWM_STUB_BUILD)/Viwm_stub
	@echo "Running iwm_stub unit tb..."
	$(IWM_STUB_BUILD)/Viwm_stub

$(IWM_STUB_BUILD)/Viwm_stub: $(IWM_STUB_RTL) $(TB_DIR)/tb_iwm.cpp
	@mkdir -p $(IWM_STUB_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/core/decode -I$(RTL_DIR)/core -I$(RTL_DIR) \
		-Mdir $(IWM_STUB_BUILD) \
		--top-module iwm_stub \
		$(IWM_STUB_RTL) \
		$(TB_DIR)/tb_iwm.cpp \
		-CFLAGS "-std=c++17"

# rat/rob/iq_int/iq_fp/iq_mem/commit/exception/predecode/decode-probe/
# decode-ea-helper-check/tb-decode-fpu/tb-decode-shadow unit tbs removed:
# all verilated rtl/core/* files that moved to the cpu/ submodule.  Live in
# cpu/Makefile (`cd cpu && make tb-rat` etc).

# ──────────────────────────────────────────────────────────────────────────────
# IRQ aggregator unit testbench
# ──────────────────────────────────────────────────────────────────────────────
IRQAGG_RTL   := $(RTL_DIR)/mac/irq_agg.v
IRQAGG_BUILD := $(BUILD_DIR)/irq_agg

.PHONY: tb-irq-agg
tb-irq-agg: $(IRQAGG_BUILD)/Virq_agg
	@echo "Running irq_agg unit tb..."
	$(IRQAGG_BUILD)/Virq_agg

$(IRQAGG_BUILD)/Virq_agg: $(IRQAGG_RTL) $(TB_DIR)/tb_irq_agg.cpp
	@mkdir -p $(IRQAGG_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(IRQAGG_BUILD) \
		--top-module irq_agg \
		$(IRQAGG_RTL) \
		$(TB_DIR)/tb_irq_agg.cpp \
		-CFLAGS "-std=c++17"

# tb-exception-uop-gen removed: wrapped rtl/core/exception_uop_gen.vh,
# which moved to the cpu/ submodule — `cd cpu && make tb-exception-uop-gen`.

# ──────────────────────────────────────────────────────────────────────────────
# Debug-full-reset overlay re-arm unit testbench (task #256)
#
# Standalone Verilog DUT (tb/tb_debug_full_reset.v) that mirrors the
# overlay re-arm gating from rtl/fpga_top_clocks.vh + the rearm FF from
# rtl/fpga_top_cpu.vh + the VIA1 ORB[3] reset path from rtl/mac/via1.v.
# Validates that vio_boot_ctrl[3] (jtag_debug_full_reset) re-arms the
# reset_overlay_active_q flag AND resets VIA1's overlay-bit input,
# without affecting the legacy CPU-only-halt semantics of bit[2].
# ──────────────────────────────────────────────────────────────────────────────
DBGRST_RTL   := $(TB_DIR)/tb_debug_full_reset.v
DBGRST_BUILD := $(BUILD_DIR)/debug_full_reset

.PHONY: tb-debug-full-reset
tb-debug-full-reset: $(DBGRST_BUILD)/Vdebug_full_reset_dut
	@echo "Running debug_full_reset overlay re-arm tb..."
	$(DBGRST_BUILD)/Vdebug_full_reset_dut

$(DBGRST_BUILD)/Vdebug_full_reset_dut: $(DBGRST_RTL) $(TB_DIR)/tb_debug_full_reset.cpp
	@mkdir -p $(DBGRST_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-Mdir $(DBGRST_BUILD) \
		--top-module debug_full_reset_dut \
		$(DBGRST_RTL) \
		$(TB_DIR)/tb_debug_full_reset.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# cpu_rst_stretch — minimum-pulse stretcher on the cpu_rst OR.  Validates
# that a 1-cycle glitch on dbg_soft_rst (or any other OR input) produces
# an 8-cycle cpu_rst, not a 1-cycle pulse that could leave PRF/ROB/RAT
# half-reset.
# ──────────────────────────────────────────────────────────────────────────────
CPU_RST_STR_RTL   := $(TB_DIR)/tb_cpu_rst_stretch.v
CPU_RST_STR_BUILD := $(BUILD_DIR)/cpu_rst_stretch

.PHONY: tb-cpu-rst-stretch
tb-cpu-rst-stretch: $(CPU_RST_STR_BUILD)/Vcpu_rst_stretch_dut
	@echo "Running cpu_rst_stretch tb..."
	$(CPU_RST_STR_BUILD)/Vcpu_rst_stretch_dut

$(CPU_RST_STR_BUILD)/Vcpu_rst_stretch_dut: $(CPU_RST_STR_RTL) $(TB_DIR)/tb_cpu_rst_stretch.cpp
	@mkdir -p $(CPU_RST_STR_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-Mdir $(CPU_RST_STR_BUILD) \
		--top-module cpu_rst_stretch_dut \
		$(CPU_RST_STR_RTL) \
		$(TB_DIR)/tb_cpu_rst_stretch.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# boot_release_gate — directed regression for the boot_rom_ready / cpu_rst
# release gate from rtl/fpga_top_clocks.vh.  Validates that a JTAG
# release-CPU bit (vio_boot_ctrl[1]) cannot fire the CPU during a
# debug_full_reset, and that boot_fsm_rst re-arms on the same pulse.
# ──────────────────────────────────────────────────────────────────────────────
BOOT_REL_RTL   := $(TB_DIR)/tb_boot_release_gate.v
BOOT_REL_BUILD := $(BUILD_DIR)/boot_release_gate

.PHONY: tb-boot-release-gate
tb-boot-release-gate: $(BOOT_REL_BUILD)/Vboot_release_gate_dut
	@echo "Running boot_release_gate tb..."
	$(BOOT_REL_BUILD)/Vboot_release_gate_dut

$(BOOT_REL_BUILD)/Vboot_release_gate_dut: $(BOOT_REL_RTL) $(TB_DIR)/tb_boot_release_gate.cpp
	@mkdir -p $(BOOT_REL_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-Mdir $(BOOT_REL_BUILD) \
		--top-module boot_release_gate_dut \
		$(BOOT_REL_RTL) \
		$(TB_DIR)/tb_boot_release_gate.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# reset_debounce — sync + debounce filter for cpu_resetn / btn[3] pads.
# Validates cold-release timing, bouncy-press one-edge property, and
# short-glitch rejection.
# ──────────────────────────────────────────────────────────────────────────────
RST_DEBOUNCE_RTL   := $(RTL_DIR)/board/reset_debounce.v
RST_DEBOUNCE_BUILD := $(BUILD_DIR)/reset_debounce
RST_DEBOUNCE_CYC   ?= 8

.PHONY: tb-reset-debounce
tb-reset-debounce: $(RST_DEBOUNCE_BUILD)/Vreset_debounce
	@echo "Running reset_debounce unit tb (DEBOUNCE_CYCLES=$(RST_DEBOUNCE_CYC))..."
	$(RST_DEBOUNCE_BUILD)/Vreset_debounce

$(RST_DEBOUNCE_BUILD)/Vreset_debounce: $(RST_DEBOUNCE_RTL) $(TB_DIR)/tb_reset_debounce.cpp
	@mkdir -p $(RST_DEBOUNCE_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-GDEBOUNCE_CYCLES=$(RST_DEBOUNCE_CYC) \
		-Mdir $(RST_DEBOUNCE_BUILD) \
		--top-module reset_debounce \
		$(RST_DEBOUNCE_RTL) \
		$(TB_DIR)/tb_reset_debounce.cpp \
		-CFLAGS "-std=c++17 -DDEBOUNCE_CYCLES=$(RST_DEBOUNCE_CYC)"

# Momentary-button polarity variant (IDLE_OUT_N=1) — regression for the
# real-HW boot-NMI bug (2026-07-17): NMI btn[1] / debug-full-reset
# btn[2] must power up already agreeing with "not pressed" so no
# spurious edge reaches irq_agg.v.  See tb_reset_debounce.cpp's
# "Scenario 4" header comment for the full story.
RST_DEBOUNCE_IDLEHI_BUILD := $(BUILD_DIR)/reset_debounce_idle_high

.PHONY: tb-reset-debounce-idle-high
tb-reset-debounce-idle-high: $(RST_DEBOUNCE_IDLEHI_BUILD)/Vreset_debounce
	@echo "Running reset_debounce unit tb (IDLE_OUT_N=1, DEBOUNCE_CYCLES=$(RST_DEBOUNCE_CYC))..."
	$(RST_DEBOUNCE_IDLEHI_BUILD)/Vreset_debounce

$(RST_DEBOUNCE_IDLEHI_BUILD)/Vreset_debounce: $(RST_DEBOUNCE_RTL) $(TB_DIR)/tb_reset_debounce.cpp
	@mkdir -p $(RST_DEBOUNCE_IDLEHI_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-GDEBOUNCE_CYCLES=$(RST_DEBOUNCE_CYC) -GIDLE_OUT_N=1 \
		-Mdir $(RST_DEBOUNCE_IDLEHI_BUILD) \
		--top-module reset_debounce \
		$(RST_DEBOUNCE_RTL) \
		$(TB_DIR)/tb_reset_debounce.cpp \
		-CFLAGS "-std=c++17 -DDEBOUNCE_CYCLES=$(RST_DEBOUNCE_CYC) -DTB_IDLE_OUT_N=1"

# ──────────────────────────────────────────────────────────────────────────────
# debug-full-reset edge-detect / pulse-stretch / watchdog wrapper
# Standalone Verilog DUT (tb/tb_dbg_rst_pulse.v) that mirrors the inline
# pulse-shaping logic from rtl/fpga_top_clocks.vh.  Validates that the
# vio_boot_ctrl[3] / btn[2] level inputs become an edge-triggered
# fixed-length pulse with stuck-input watchdog recovery.
# ──────────────────────────────────────────────────────────────────────────────
DBG_RST_PULSE_RTL   := $(TB_DIR)/tb_dbg_rst_pulse.v
DBG_RST_PULSE_BUILD := $(BUILD_DIR)/dbg_rst_pulse
# Sim-friendly defaults; matches the SIM_MODEL branch in fpga_top_clocks.vh.
DBG_RST_PULSE_PCYC  ?= 8
DBG_RST_PULSE_SCYC  ?= 64

.PHONY: tb-dbg-rst-pulse
tb-dbg-rst-pulse: $(DBG_RST_PULSE_BUILD)/Vdbg_rst_pulse_dut
	@echo "Running debug_full_reset pulse-shaper tb..."
	$(DBG_RST_PULSE_BUILD)/Vdbg_rst_pulse_dut

$(DBG_RST_PULSE_BUILD)/Vdbg_rst_pulse_dut: $(DBG_RST_PULSE_RTL) $(TB_DIR)/tb_dbg_rst_pulse.cpp
	@mkdir -p $(DBG_RST_PULSE_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-GPULSE_CYCLES=$(DBG_RST_PULSE_PCYC) \
		-GSTUCK_CYCLES=$(DBG_RST_PULSE_SCYC) \
		-Mdir $(DBG_RST_PULSE_BUILD) \
		--top-module dbg_rst_pulse_dut \
		$(DBG_RST_PULSE_RTL) \
		$(TB_DIR)/tb_dbg_rst_pulse.cpp \
		-CFLAGS "-std=c++17 -DPULSE_CYCLES=$(DBG_RST_PULSE_PCYC) -DSTUCK_CYCLES=$(DBG_RST_PULSE_SCYC)"

# ──────────────────────────────────────────────────────────────────────────────
# PRAM-clear one-shot (vio_boot_ctrl[4] -> u_rtc.pram_clear)
#
# Unlike tb-dbg-rst-pulse above (which verilates a hand-written MIRROR of the
# inline RTL, and can therefore drift out of sync with it), this DUT is
# GENERATED from rtl/soc/fpga_top_clocks.vh by extracting the shipped logic
# verbatim between the PRAM_CLEAR_ONESHOT_BEGIN/END markers.  The test can
# only ever exercise the real one-shot.
#
# What it protects: PRAM is battery-backed (rtc.v never clears it on reset),
# so pram_clear is the only escape hatch — and it is driven from a LEVEL
# VIO probe-out.  If the edge-detect regressed, a bit left set would hold
# PRAM permanently cleared and silently swallow every Mac OS PRAM write.
# ──────────────────────────────────────────────────────────────────────────────
PRAM_CLEAR_SRC    := $(RTL_DIR)/soc/fpga_top_clocks.vh
PRAM_CLEAR_BUILD  := $(BUILD_DIR)/pram_clear_pulse
PRAM_CLEAR_GEN    := $(PRAM_CLEAR_BUILD)/pram_clear_oneshot_dut.v
PRAM_CLEAR_PCYC   ?= 8
PRAM_CLEAR_PCNTW  ?= 5

.PHONY: tb-pram-clear-pulse
tb-pram-clear-pulse: $(PRAM_CLEAR_BUILD)/Vpram_clear_oneshot_dut
	@echo "Running pram_clear one-shot tb..."
	$(PRAM_CLEAR_BUILD)/Vpram_clear_oneshot_dut

$(PRAM_CLEAR_GEN): $(PRAM_CLEAR_SRC) $(TOOLS_DIR)/extract_pram_clear_oneshot.py
	@mkdir -p $(PRAM_CLEAR_BUILD)
	python3 $(TOOLS_DIR)/extract_pram_clear_oneshot.py $(PRAM_CLEAR_SRC) $@

$(PRAM_CLEAR_BUILD)/Vpram_clear_oneshot_dut: $(PRAM_CLEAR_GEN) $(TB_DIR)/tb_pram_clear_pulse.cpp
	@mkdir -p $(PRAM_CLEAR_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE \
		-GPULSE_CYCLES=$(PRAM_CLEAR_PCYC) \
		-GPCNT_W=$(PRAM_CLEAR_PCNTW) \
		-Mdir $(PRAM_CLEAR_BUILD) \
		--top-module pram_clear_oneshot_dut \
		$(PRAM_CLEAR_GEN) \
		$(TB_DIR)/tb_pram_clear_pulse.cpp \
		-CFLAGS "-std=c++17 -DPULSE_CYCLES=$(PRAM_CLEAR_PCYC)"

# tb-mul-div removed: verilated rtl/core/execute/mul_div.v, which moved to
# the cpu/ submodule — `cd cpu && make tb-mul-div`.

# ──────────────────────────────────────────────────────────────────────────────
# Q700 ROM cold-boot bring-up harness (task #101) — RETIRED
#
# This was a standalone flat-mac_top build (rtl/core/* + rtl/mac/* +
# rtl/mac_top.v via RTL_SRCS) with tb_rom_boot.cpp in the main-cpp slot,
# loading files/420dbff3.rom at RESET_PC=0x4000_002A (the Q700 ROM reset
# vector entry).  rtl/core/* moved to the cpu/ submodule in the SoC split,
# so $(ROMBOOT_BUILD)/Vmac_top below (and every tb-rom-boot-*-smoke /
# rom-boot-snapshot-* target that depends on it) now fails loud with a
# pointer to the fpga_top-based replacement (tb-fpga-top-rom /
# tb-via1-lockstep / tb-axi-lockstep / tb-dafb-lockstep /
# tb-scc-uart-loopback).  ROMBOOT_OUTPUT_ROOT / ROM / scout-rom below stay
# live — they're used by other (working) targets.
# ──────────────────────────────────────────────────────────────────────────────
ROMBOOT_BUILD := $(BUILD_DIR)/rom_boot

# Default ROM: Quadra 700 Universal (macqd700 in MAME — stored checksum
# 0x420dbff3, 1 MB, SHA1 7a8ee468d16e64f2ad10cb8d1a45e6f07cc9e212).  This
# matches our Q700-class peripheral RTL (discrete VIA1+VIA2 / NCR 5380 /
# Z80 SCC / ASC / DAFB / MEMCjr).  Override via `make tb-rom-boot
# ROM=<path>` to point at a different image (e.g. a keeper LC 630 dump
# for chipset-divergence study).
ROM ?= $(PROJ_ROOT)/files/420dbff3.rom
ROMBOOT_SCRATCH_MIN_MB ?= 512
ROMBOOT_OUTPUT_ROOT ?= $(shell \
	root=/dev/shm/m68k-ooo; \
	fallback="$(BUILD_DIR)/sim"; \
	min_mb="$(ROMBOOT_SCRATCH_MIN_MB)"; \
	if [ -d /dev/shm ] && [ -w /dev/shm ]; then \
		avail=$$(df -Pm /dev/shm 2>/dev/null | awk 'NR==2 {print $$4}'); \
		if [ -n "$$avail" ] && [ "$$avail" -ge "$$min_mb" ]; then \
			printf '%s\n' "$$root"; \
			exit 0; \
		fi; \
	fi; \
	printf '%s\n' "$$fallback")

.PHONY: scout-rom
scout-rom:
	@mkdir -p $(BUILD_DIR)/scout
	m68k-linux-gnu-objdump -D -b binary -m m68k:68040 --adjust-vma=0x40000000 \
		--start-address=$(SCOUT_START) --stop-address=$(SCOUT_STOP) \
		$(ROM) > $(BUILD_DIR)/scout/rom_scout.txt
	@echo "Scout disassembly: $(BUILD_DIR)/scout/rom_scout.txt"
	@wc -l $(BUILD_DIR)/scout/rom_scout.txt

SCOUT_START ?= 0x40002f80
SCOUT_STOP  ?= 0x40004010

.PHONY: tb-rom-boot
tb-rom-boot:
	@echo "tb-rom-boot was removed: use tb-fpga-top-rom for ROM execution through full RTL." >&2
	@exit 2

ROMBOOT_PERIPH_EVENT_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_periph_events.log
ROMBOOT_PERIPH_EVENT_MAX ?= 70000
ROMBOOT_PERIPH_EVENT_TIMEOUT ?= 4000000
ROMBOOT_ASC_EVENT_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_asc_events.log
ROMBOOT_ASC_WAV ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_asc.wav

PERIPH_EVENT_LOG_TEST_BUILD := $(BUILD_DIR)/periph_event_log
MEM_MODEL_TEST_BUILD := $(BUILD_DIR)/mem_model

.PHONY: tb-periph-event-log
tb-periph-event-log: $(PERIPH_EVENT_LOG_TEST_BUILD)/tb_periph_event_log
	@echo "Running peripheral event logger host unit tb..."
	$(PERIPH_EVENT_LOG_TEST_BUILD)/tb_periph_event_log

.PHONY: tb-mem-model
tb-mem-model: $(MEM_MODEL_TEST_BUILD)/tb_mem_model
	@echo "Running host MemModel unit tb..."
	$(MEM_MODEL_TEST_BUILD)/tb_mem_model

.PHONY: tb-model-rtl-consistency
tb-model-rtl-consistency:
	@echo "tb-model-rtl-consistency was removed: use full fpga_top RTL simulation instead." >&2
	@exit 2

MODEL_RTL_EQ_BUILD := $(BUILD_DIR)/model_rtl_eq
MODEL_RTL_EQ_RTL := $(RTL_DIR)/mac/glue.v

.PHONY: tb-model-rtl-eq
tb-model-rtl-eq:
	@echo "tb-model-rtl-eq was removed: use full fpga_top RTL simulation instead." >&2
	@exit 2

$(MODEL_RTL_EQ_BUILD)/Vglue: \
		$(MODEL_RTL_EQ_RTL) \
		$(TB_DIR)/tb_model_rtl_eq.cpp \
		$(TB_DIR)/models/rom_boot_bus.cpp \
		$(TB_DIR)/models/rom_boot_bus.h \
		$(TB_DIR)/models/mem_model.cpp \
		$(TB_DIR)/models/mem_model.h \
		$(TB_DIR)/models/m68k_bus.h
	@mkdir -p $(MODEL_RTL_EQ_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST \
		-I$(RTL_DIR)/mac -I$(RTL_DIR) \
		-Mdir $(MODEL_RTL_EQ_BUILD) \
		--top-module glue \
		$(MODEL_RTL_EQ_RTL) \
		$(TB_DIR)/tb_model_rtl_eq.cpp \
		$(TB_DIR)/models/rom_boot_bus.cpp \
		$(TB_DIR)/models/mem_model.cpp \
		-CFLAGS "-std=c++17 -I$(TB_DIR)"

$(MEM_MODEL_TEST_BUILD)/tb_mem_model: \
		$(TB_DIR)/tb_mem_model.cpp \
		$(TB_DIR)/models/mem_model.cpp $(TB_DIR)/models/mem_model.h
	@mkdir -p $(MEM_MODEL_TEST_BUILD)
	$(CXX) -std=c++17 -Wall -Wextra -I$(TB_DIR) \
		$(TB_DIR)/tb_mem_model.cpp \
		$(TB_DIR)/models/mem_model.cpp \
		-o $@

$(PERIPH_EVENT_LOG_TEST_BUILD)/tb_periph_event_log: \
		$(TB_DIR)/tb_periph_event_log.cpp \
		$(TB_DIR)/periph_event_log.cpp $(TB_DIR)/periph_event_log.h
	@mkdir -p $(PERIPH_EVENT_LOG_TEST_BUILD)
	$(CXX) -std=c++17 -Wall -Wextra -I$(TB_DIR) \
		$(TB_DIR)/tb_periph_event_log.cpp \
		$(TB_DIR)/periph_event_log.cpp \
		-o $@

.PHONY: tb-rom-boot-periph-events
tb-rom-boot-periph-events: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking ROM boot peripheral model event logging"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_OUTPUT_ROOT)/rom_boot_periph_events_trace.log \
		+timeout=$(ROMBOOT_PERIPH_EVENT_TIMEOUT) \
		+max_insts=$(ROMBOOT_PERIPH_EVENT_MAX) \
		+periph_event_log=$(ROMBOOT_PERIPH_EVENT_LOG) \
		+periph_event_log_limit=256 \
		+periph_event_filter=VIA1,VIA2,ADB,VBL,RTC,PRAM,SCSI,SCC,ASC,DAFB,VRAM \
		+no_waves
	@python3 $(TB_DIR)/check_periph_event_log.py "$(ROMBOOT_PERIPH_EVENT_LOG)" \
		--expect VIA1\>=1 \
		--expect VIA2\>=1 \
		--expect ADB\>=1 \
		--expect VBL\>=1 \
		--expect RTC\>=1 \
		--expect PRAM\>=0 \
		--expect SCSI\>=0 \
		--expect SCC\>=0 \
		--expect ASC\>=0 \
		--expect DAFB\>=0 \
		--expect VRAM\>=0 \
		--expect-event VIA1.read\>=1 \
		--expect-event VIA1.write\>=1 \
		--expect-event VIA2.read\>=1 \
		--expect-event VIA2.write\>=1 \
		--expect-event ADB.pcr_write\>=1 \
		--expect-event ADB.acr_write\>=1 \
		--expect-event RTC.select\>=1 \
		--expect-event RTC.clk_fall\>=1 \
		--expect-event RTC.deselect\>=1 \
		--expect-event VBL.ier_set\>=1 \
		--expect-event VBL.ier_clear\>=1
	@echo "Peripheral model event log smoke passed: $(ROMBOOT_PERIPH_EVENT_LOG)"

.PHONY: rom-boot-asc-wav
rom-boot-asc-wav:
	@echo "Exporting ASC FIFO activity to WAV"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@python3 $(TOOLS_DIR)/asc_wav_from_periph_log.py \
		"$(ROMBOOT_ASC_EVENT_LOG)" "$(ROMBOOT_ASC_WAV)"
	@echo "rom-boot-asc-wav: wav at $(ROMBOOT_ASC_WAV)"

ROMBOOT_VIA1_T1_EVENT_LOG ?= $(BUILD_DIR)/sim/rom_boot_via1_t1_events.log
ROMBOOT_RTC_EVENT_LOG ?= $(BUILD_DIR)/sim/rom_boot_rtc_events.log
ROMBOOT_ADB_EVENT_LOG ?= $(BUILD_DIR)/sim/rom_boot_adb_events.log
ROMBOOT_SCC_ASC_EVENT_LOG ?= $(BUILD_DIR)/sim/rom_boot_scc_asc_events.log
ROMBOOT_SCSI_EVENT_LOG ?= $(BUILD_DIR)/sim/rom_boot_scsi_events.log

.PHONY: tb-rom-boot-via1-t1-smoke
tb-rom-boot-via1-t1-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking VIA1 Timer1 (VBL) countdown in the ROM harness stub"
	@mkdir -p $(BUILD_DIR)/sim
	$(ROMBOOT_BUILD)/Vmac_top +via1_t1_selftest \
		+via1_timer_div=1 \
		+periph_event_log=$(ROMBOOT_VIA1_T1_EVENT_LOG) \
		+periph_event_log_limit=0 \
		+no_waves
	@grep -q "VIA1.timer1_start" "$(ROMBOOT_VIA1_T1_EVENT_LOG)"
	@grep -q "VBL.t1_wrap" "$(ROMBOOT_VIA1_T1_EVENT_LOG)"
	@echo "VIA1 Timer1 smoke passed: $(ROMBOOT_VIA1_T1_EVENT_LOG)"

.PHONY: tb-rom-boot-harness-strictness-smoke
tb-rom-boot-harness-strictness-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking strict overlay / AXI response harness modes"
	$(ROMBOOT_BUILD)/Vmac_top +harness_strictness_selftest +no_waves

.PHONY: tb-rom-boot-rtc-smoke
tb-rom-boot-rtc-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking VIA1-driven RTC/PRAM side-channel in the ROM harness stub"
	@mkdir -p $(BUILD_DIR)/sim
	$(ROMBOOT_BUILD)/Vmac_top +rtc_sidechannel_selftest \
		+periph_event_log=$(ROMBOOT_RTC_EVENT_LOG) \
		+periph_event_filter=RTC,PRAM \
		+periph_event_log_limit=0 \
		+no_waves
	@grep -q "RTC.cmd_read" "$(ROMBOOT_RTC_EVENT_LOG)"
	@grep -q "PRAM.seconds_read" "$(ROMBOOT_RTC_EVENT_LOG)"
	@grep -q "PRAM.pram_write" "$(ROMBOOT_RTC_EVENT_LOG)"
	@echo "RTC side-channel smoke passed: $(ROMBOOT_RTC_EVENT_LOG)"

.PHONY: tb-rom-boot-adb-smoke
tb-rom-boot-adb-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking VIA1 SR/ADB shadow visibility in the ROM harness stub"
	@mkdir -p $(BUILD_DIR)/sim
	$(ROMBOOT_BUILD)/Vmac_top +adb_shadow_selftest \
		+periph_event_log=$(ROMBOOT_ADB_EVENT_LOG) \
		+periph_event_filter=VIA1,ADB \
		+periph_event_log_limit=0 \
		+no_waves
	@grep -q "VIA1.write" "$(ROMBOOT_ADB_EVENT_LOG)"
	@grep -q "ADB.sr_write" "$(ROMBOOT_ADB_EVENT_LOG)"
	@grep -q "ADB.shift_complete" "$(ROMBOOT_ADB_EVENT_LOG)"
	@grep -q "ADB.sr_read" "$(ROMBOOT_ADB_EVENT_LOG)"
	@grep -q "ADB.sr_irq_clear" "$(ROMBOOT_ADB_EVENT_LOG)"
	@grep -q "ADB.sr_read_clear" "$(ROMBOOT_ADB_EVENT_LOG)"
	@echo "ADB shadow smoke passed: $(ROMBOOT_ADB_EVENT_LOG)"

.PHONY: tb-rom-boot-scc-asc-smoke
tb-rom-boot-scc-asc-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking SCC/ASC ROM harness register visibility"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@mkdir -p $(dir $(ROMBOOT_SCC_ASC_EVENT_LOG))
	$(ROMBOOT_BUILD)/Vmac_top +scc_asc_selftest \
		+periph_event_log=$(ROMBOOT_SCC_ASC_EVENT_LOG) \
		+periph_event_filter=SCC,ASC \
		+periph_event_log_limit=64 \
		+no_waves
	@grep -q "SCC.reg_read" "$(ROMBOOT_SCC_ASC_EVENT_LOG)"
	@grep -q "SCC.reg_write" "$(ROMBOOT_SCC_ASC_EVENT_LOG)"
	@grep -q "SCC.data_read" "$(ROMBOOT_SCC_ASC_EVENT_LOG)"
	@grep -q "SCC.data_write" "$(ROMBOOT_SCC_ASC_EVENT_LOG)"
	@grep -q "ASC.read" "$(ROMBOOT_SCC_ASC_EVENT_LOG)"
	@grep -q "ASC.write" "$(ROMBOOT_SCC_ASC_EVENT_LOG)"
	@grep -q "reg=FIFO_A" "$(ROMBOOT_SCC_ASC_EVENT_LOG)"
	@grep -q "reg=FIFO_B" "$(ROMBOOT_SCC_ASC_EVENT_LOG)"
	@echo "SCC/ASC ROM harness smoke passed: $(ROMBOOT_SCC_ASC_EVENT_LOG)"

.PHONY: tb-rom-boot-scsi-smoke
tb-rom-boot-scsi-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking TurboSCSI window visibility in the ROM harness stub"
	@mkdir -p $(BUILD_DIR)/sim
	$(ROMBOOT_BUILD)/Vmac_top +scsi_window_selftest \
		+periph_event_log=$(ROMBOOT_SCSI_EVENT_LOG) \
		+periph_event_filter=SCSI \
		+periph_event_log_limit=0 \
		+no_waves
	@grep -q "SCSI.reg_write" "$(ROMBOOT_SCSI_EVENT_LOG)"
	@grep -q "SCSI.reg_read" "$(ROMBOOT_SCSI_EVENT_LOG)"
	@grep -q "SCSI.selection_phase" "$(ROMBOOT_SCSI_EVENT_LOG)"
	@grep -q "SCSI.command_phase" "$(ROMBOOT_SCSI_EVENT_LOG)"
	@grep -q "SCSI.status_phase" "$(ROMBOOT_SCSI_EVENT_LOG)"
	@grep -q "SCSI.interrupt_phase" "$(ROMBOOT_SCSI_EVENT_LOG)"
	@grep -q "SCSI.sequence_phase" "$(ROMBOOT_SCSI_EVENT_LOG)"
	@grep -q "SCSI.cdb_byte" "$(ROMBOOT_SCSI_EVENT_LOG)"
	@grep -q "SCSI.raw_block_read" "$(ROMBOOT_SCSI_EVENT_LOG)"
	@grep -q "SCSI.dma_write" "$(ROMBOOT_SCSI_EVENT_LOG)"
	@grep -q "SCSI.dma_read" "$(ROMBOOT_SCSI_EVENT_LOG)"
	@echo "SCSI window smoke passed: $(ROMBOOT_SCSI_EVENT_LOG)"

ROMBOOT_SNAPSHOT_DIR ?= $(BUILD_DIR)/rom_boot_checkpoints
ROMBOOT_SNAPSHOT_POINTS ?= 250,500,1000,2000,5000,10000,25000,50000,100000,250000,500000,1000000,1600000
ROMBOOT_SNAPSHOT_MAX ?= 1800000
ROMBOOT_SNAPSHOT_TIMEOUT ?= 10000000
ROMBOOT_RESUME_MAX ?= 1
ROMBOOT_RESUME_TIMEOUT ?= 200000
ROMBOOT_DEEP_START ?= $(ROMBOOT_SNAPSHOT_DIR)/q700.final.vlt
ROMBOOT_DEEP_CYCLE_POINTS ?= 4000000,8000000,16000000,32000000,40000000
ROMBOOT_DEEP_STOP_CYCLE ?= 40000000
ROMBOOT_DEEP_MAX ?= 50000000
ROMBOOT_DESCRIPTOR_SMOKE_LOG ?= $(BUILD_DIR)/sim/rom_boot_descriptor_smoke.log
ROMBOOT_DESCRIPTOR_SMOKE_TRACE ?= $(BUILD_DIR)/sim/rom_boot_descriptor_smoke_trace.log
ROMBOOT_DESCRIPTOR_SMOKE_MAX ?= 120000
ROMBOOT_DESCRIPTOR_SMOKE_TIMEOUT ?= 5000000
ROMBOOT_FASTDIAG_LOG ?= $(BUILD_DIR)/sim/rom_boot_fastdiag_smoke.log
ROMBOOT_FASTDIAG_TRACE ?= $(BUILD_DIR)/sim/rom_boot_fastdiag_smoke_trace.log
ROMBOOT_FASTDIAG_PC ?= 0x40846e3c
ROMBOOT_FASTDIAG_MAX ?= 20000
ROMBOOT_FASTDIAG_TIMEOUT ?= 500000
ROMBOOT_CHIME_DELAY_LOG ?= $(BUILD_DIR)/sim/rom_boot_chime_delay_smoke.log
ROMBOOT_CHIME_DELAY_TRACE ?= $(BUILD_DIR)/sim/rom_boot_chime_delay_smoke_trace.log
ROMBOOT_CHIME_DELAY_PC ?= 0x40846ef2
ROMBOOT_CHIME_DELAY_MAX ?= 900000
ROMBOOT_CHIME_DELAY_TIMEOUT ?= 4000000
ROMBOOT_TIMER_DELAY_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_timer_delay_smoke.log
ROMBOOT_TIMER_DELAY_EVENTS ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_timer_delay_smoke.events.log
ROMBOOT_TIMER_DELAY_LASTN ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_timer_delay_smoke_lastn.log
ROMBOOT_TIMER_DELAY_PC ?= 0x408005b0
ROMBOOT_TIMER_DELAY_MAX ?= 8000000
ROMBOOT_TIMER_DELAY_TIMEOUT ?= 25000000
# Frontier runs default to the stock ROM.  Patch sets are still useful for
# bounded, labelled experiments, but they are not a trustworthy source of
# truth for ROM bring-up unless the same milestone reproduces without them.
ROMBOOT_FRONTIER_PATCHES ?=
ROMBOOT_FRONTIER_PATCH_ARG = $(if $(strip $(ROMBOOT_FRONTIER_PATCHES)),+rom_patch=$(ROMBOOT_FRONTIER_PATCHES),)
ROMBOOT_FRONTIER_PC_TAG ?= $(patsubst 0x%,%,$(ROMBOOT_TIMER_DELAY_PC))
ROMBOOT_FRONTIER_CHECKPOINT_DIR ?= $(ROMBOOT_SNAPSHOT_DIR)/frontier
ROMBOOT_FRONTIER_CHECKPOINT ?= $(ROMBOOT_FRONTIER_CHECKPOINT_DIR)/q700.timer_delay_$(ROMBOOT_FRONTIER_PC_TAG).vlt
ROMBOOT_FRONTIER_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_frontier_checkpoint.log
ROMBOOT_FRONTIER_EVENTS ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_frontier_checkpoint.events.log
ROMBOOT_FRONTIER_LASTN ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_frontier_checkpoint_lastn.log
ROMBOOT_FRONTIER_TRACE ?= /dev/null
ROMBOOT_FRONTIER_RESTORE_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_frontier_restore.log
ROMBOOT_FRONTIER_RESTORE_TRACE ?= /dev/null
ROMBOOT_FRONTIER_RESTORE_MAX ?= 1
ROMBOOT_FRONTIER_RESTORE_TIMEOUT ?= 200000
ROMBOOT_ARCH_CHECKPOINT ?= $(ROMBOOT_OUTPUT_ROOT)/q700.arch_checkpoint.txt
ROMBOOT_ARCH_CHECKPOINT_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_checkpoint.log
ROMBOOT_ARCH_CHECKPOINT_TRACE ?= /dev/null
ROMBOOT_ARCH_CHECKPOINT_MAX ?= 1024
ROMBOOT_ARCH_CHECKPOINT_TIMEOUT ?= 1000000
ROMBOOT_ARCH_CHECKPOINT_EXTRA ?=
ROMBOOT_ARCH_CHECKPOINT_SMOKE ?= $(ROMBOOT_OUTPUT_ROOT)/q700.arch_checkpoint_smoke.txt
ROMBOOT_ARCH_CHECKPOINT_SMOKE_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_checkpoint_smoke.log
ROMBOOT_ARCH_CHECKPOINT_SMOKE_TRACE ?= /dev/null
ROMBOOT_ARCH_CHECKPOINT_SMOKE_MAX ?= 64
ROMBOOT_ARCH_CHECKPOINT_SMOKE_TIMEOUT ?= 300000
ROMBOOT_ARCH_CHECKPOINT_SUMMARY := python3 $(TOOLS_DIR)/rom_boot_arch_checkpoint_summary.py
ROMBOOT_ARCH_COMPARE := python3 $(TOOLS_DIR)/rom_boot_arch_compare.py
ROMBOOT_ARCH_RESUME ?= $(ROMBOOT_ARCH_CHECKPOINT)
ROMBOOT_ARCH_RESUME_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_resume.log
ROMBOOT_ARCH_RESUME_TRACE ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_resume_trace.log
ROMBOOT_ARCH_RESUME_MAX ?= 2048
ROMBOOT_ARCH_RESUME_TIMEOUT ?= 1000000
ROMBOOT_ARCH_RESUME_EXTRA ?=
ROMBOOT_ARCH_REPLAY_SMOKE ?= $(ROMBOOT_OUTPUT_ROOT)/q700.arch_replay_smoke.txt
ROMBOOT_ARCH_REPLAY_CAPTURE_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_replay_capture.log
ROMBOOT_ARCH_REPLAY_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_replay.log
ROMBOOT_ARCH_REPLAY_TRACE ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_replay_trace.log
ROMBOOT_ARCH_REPLAY_CAPTURE_MAX ?= 64
ROMBOOT_ARCH_REPLAY_CAPTURE_TIMEOUT ?= 300000
ROMBOOT_ARCH_REPLAY_MAX ?= 65
ROMBOOT_ARCH_REPLAY_TIMEOUT ?= 300000
ROMBOOT_ARCH_ROUNDTRIP_DIR ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_roundtrip
ROMBOOT_ARCH_ROUNDTRIP_SEED ?= $(ROMBOOT_ARCH_ROUNDTRIP_DIR)/q700.seed.txt
ROMBOOT_ARCH_ROUNDTRIP_CONTINUOUS ?= $(ROMBOOT_ARCH_ROUNDTRIP_DIR)/q700.continuous.txt
ROMBOOT_ARCH_ROUNDTRIP_REPLAYED ?= $(ROMBOOT_ARCH_ROUNDTRIP_DIR)/q700.replayed.txt
ROMBOOT_ARCH_ROUNDTRIP_SEED_LOG ?= $(ROMBOOT_ARCH_ROUNDTRIP_DIR)/seed.log
ROMBOOT_ARCH_ROUNDTRIP_CONTINUOUS_LOG ?= $(ROMBOOT_ARCH_ROUNDTRIP_DIR)/continuous.log
ROMBOOT_ARCH_ROUNDTRIP_REPLAYED_LOG ?= $(ROMBOOT_ARCH_ROUNDTRIP_DIR)/replayed.log
ROMBOOT_ARCH_ROUNDTRIP_SEED_MAX ?= 64
ROMBOOT_ARCH_ROUNDTRIP_FINAL_MAX ?= 192
ROMBOOT_ARCH_ROUNDTRIP_TIMEOUT ?= 500000
ROMBOOT_ARCH_ENDPC ?= $(ROMBOOT_END_BREAKPOINT_PC)
ROMBOOT_ARCH_ENDPC_CHECKPOINT ?= $(ROMBOOT_OUTPUT_ROOT)/q700.arch_endpc_smoke.txt
ROMBOOT_ARCH_ENDPC_CAPTURE_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_endpc_capture.log
ROMBOOT_ARCH_ENDPC_REPLAY_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_endpc_replay.log
ROMBOOT_ARCH_ENDPC_REPLAY_TRACE ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_endpc_replay_trace.log
ROMBOOT_ARCH_ENDPC_CAPTURE_MAX ?= 128
ROMBOOT_ARCH_ENDPC_CAPTURE_TIMEOUT ?= 300000
ROMBOOT_ARCH_ENDPC_REPLAY_MAX ?= 129
ROMBOOT_ARCH_ENDPC_REPLAY_TIMEOUT ?= 300000
ROMBOOT_ARCH_ALINE_FRONTIER_PC ?= 0x40809a96
ROMBOOT_ARCH_ALINE_FRONTIER_PC_HIT ?= 1
ROMBOOT_ARCH_ALINE_FRONTIER_CHECKPOINT ?= $(ROMBOOT_OUTPUT_ROOT)/q700.arch_aline_frontier.txt
ROMBOOT_ARCH_ALINE_FRONTIER_CAPTURE_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_aline_frontier_capture.log
ROMBOOT_ARCH_ALINE_FRONTIER_REPLAY_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_aline_frontier_replay.log
ROMBOOT_ARCH_ALINE_FRONTIER_REPLAY_TRACE ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_aline_frontier_replay_trace.log
ROMBOOT_ARCH_ALINE_FRONTIER_CACHE_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_aline_frontier_cache.log
ROMBOOT_ARCH_ALINE_FRONTIER_DATA_WATCH ?= 0x00000400-0x000007ff,0x0000051c,0x000013c0-0x000014e0,0x408ca0e0-0x408ca3ff
ROMBOOT_ARCH_ALINE_FRONTIER_CAPTURE_MAX ?= 7600000
ROMBOOT_ARCH_ALINE_FRONTIER_CAPTURE_TIMEOUT ?= 26000000
ROMBOOT_ARCH_ALINE_FRONTIER_REPLAY_MAX ?= 7524750
ROMBOOT_ARCH_ALINE_FRONTIER_REPLAY_TIMEOUT ?= 1000000
ROMBOOT_END_BREAKPOINT_PC ?= 0x4000002a
ROMBOOT_END_BREAKPOINT_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_end_breakpoint.log
ROMBOOT_END_BREAKPOINT_TRACE ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_end_breakpoint_trace.log
ROMBOOT_END_BREAKPOINT_LASTN ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_end_breakpoint_lastn.log
ROMBOOT_OVERLAY_CLEAR_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_overlay_clear.log
ROMBOOT_OVERLAY_CLEAR_TRACE ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_overlay_clear_trace.log
ROMBOOT_DATA_WATCH_SMOKE_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_data_watch_smoke.log
ROMBOOT_DATA_WATCH_SMOKE_TRACE ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_data_watch_smoke_trace.log
ROMBOOT_DATA_WATCH_SMOKE_WATCH ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_data_watch_smoke.watch.log
ROMBOOT_DATA_WATCH_SMOKE_SPEC ?= 0x00000000-0xffffffff
ROMBOOT_CACHE_EVENT_SMOKE_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_cache_event_smoke.log
ROMBOOT_CACHE_EVENT_SMOKE_EVENTS ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_cache_event_smoke.events.log
ROMBOOT_CACHE_EVENT_SMOKE_LIMIT ?= 4
ROMBOOT_CACHE_EVENT_SMOKE_MAX ?= 64
ROMBOOT_CACHE_EVENT_SMOKE_TIMEOUT ?= 200000
ROMBOOT_DISPLAY_WATCH_SMOKE_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_display_watch_smoke.log
ROMBOOT_DISPLAY_WATCH_SMOKE_EVENTS ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_display_watch_smoke.events.log
ROMBOOT_WATCHDOG_SMOKE_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_watchdog_smoke.log
ROMBOOT_OUTCOME_SMOKE_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_outcome_smoke.log
ROMBOOT_FAULT_KNOB_SMOKE_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_fault_knob_smoke.log
ROMBOOT_FAULT_KNOB_SMOKE_TRACE ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_fault_knob_smoke_trace.log
ROMBOOT_FAULT_KNOB_SMOKE_LASTN ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_fault_knob_smoke_lastn.log
ROMBOOT_FAULT_KNOB_SMOKE_DUMP ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_fault_knob_smoke_fault
ROMBOOT_STOP_SUMMARY_STOP ?= +stop_on_rom_faults
ROMBOOT_STOP_SUMMARY_EXTRA ?=
ROMBOOT_STOP_SUMMARY_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_stop_summary.log
ROMBOOT_STOP_SUMMARY_TRACE ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_stop_summary_trace.log
ROMBOOT_STOP_SUMMARY_LASTN ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_stop_summary_lastn.log
ROMBOOT_STOP_SUMMARY_REPORT ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_stop_summary_report.log
ROMBOOT_STOP_SUMMARY_LASTN_CYCLES ?= 4096
ROMBOOT_STOP_SUMMARY_TIMEOUT ?= 25000000
ROMBOOT_STOP_SUMMARY_MAX ?= 2000000
ROMBOOT_STOP_SUMMARY_SMOKE_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_stop_summary_smoke.log
ROMBOOT_STOP_SUMMARY_SMOKE_TRACE ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_stop_summary_smoke_trace.log
ROMBOOT_STOP_SUMMARY_SMOKE_LASTN ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_stop_summary_smoke_lastn.log
ROMBOOT_STOP_SUMMARY_SMOKE_REPORT ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_stop_summary_smoke_report.log
ROMBOOT_STOP_SUMMARY_SMOKE_STOP ?= +stop_on_rom_faults
ROMBOOT_STOP_SUMMARY_SMOKE_EXTRA ?=
ROMBOOT_STOP_SUMMARY_SMOKE_LASTN_CYCLES ?= 1024
ROMBOOT_STOP_SUMMARY_SMOKE_TIMEOUT ?= 25000000
ROMBOOT_STOP_SUMMARY_SMOKE_MAX ?= 2000000
ROMBOOT_STOP_EXC_SMOKE_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_stop_exc_smoke.log
ROMBOOT_STOP_EXC_SMOKE_TRACE ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_stop_exc_smoke_trace.log
ROMBOOT_STOP_EXC_SMOKE_LASTN ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_stop_exc_smoke_lastn.log
ROMBOOT_STOP_EXC_SMOKE_REPORT ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_stop_exc_smoke_report.log
ROMBOOT_STOP_EXC_SMOKE_STOP ?= +stop_on_exc=2,4,11
ROMBOOT_STOP_EXC_SMOKE_EXTRA ?=
ROMBOOT_STOP_EXC_SMOKE_LASTN_CYCLES ?= 1024
ROMBOOT_STOP_EXC_SMOKE_TIMEOUT ?= 25000000
ROMBOOT_STOP_EXC_SMOKE_MAX ?= 2000000
ROMBOOT_TRACE_ENDERS_SMOKE_LOG ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_trace_enders_smoke.log
ROMBOOT_TRACE_ENDERS_SMOKE_TRACE ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_trace_enders_smoke_trace.log
ROMBOOT_TRACE_ENDERS_SMOKE_LASTN ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_trace_enders_smoke_lastn.log
ROMBOOT_SNAPSHOT_SMOKE_SPEC ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_snapshot_smoke.tsv
ROMBOOT_SNAPSHOT_SMOKE_LOG_DIR ?= $(ROMBOOT_OUTPUT_ROOT)/rom_boot_snapshot_smoke_logs
ROMBOOT_CHECKPOINT_INVENTORY := python3 $(TOOLS_DIR)/rom_boot_checkpoint_inventory.py

.PHONY: rom-boot-snapshots
rom-boot-snapshots: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Capturing Q700 ROM boot checkpoints into $(ROMBOOT_SNAPSHOT_DIR)"
	@mkdir -p $(ROMBOOT_SNAPSHOT_DIR) $(ROMBOOT_OUTPUT_ROOT)
	$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_OUTPUT_ROOT)/rom_boot_snapshots_trace.log \
		+timeout=$(ROMBOOT_SNAPSHOT_TIMEOUT) \
		+max_insts=$(ROMBOOT_SNAPSHOT_MAX) \
		+checkpoint_prefix=$(ROMBOOT_SNAPSHOT_DIR)/q700 \
		+checkpoint_points=$(ROMBOOT_SNAPSHOT_POINTS) \
		+save_state=$(ROMBOOT_SNAPSHOT_DIR)/q700.final.vlt \
		+no_waves
	@$(ROMBOOT_CHECKPOINT_INVENTORY) "$(ROMBOOT_SNAPSHOT_DIR)" \
		--expected-commit "$(ROMBOOT_SNAPSHOT_POINTS)" \
		--require-final --forbid-unknown --require-current-metadata

.PHONY: rom-boot-deep-snapshots
rom-boot-deep-snapshots: $(ROMBOOT_BUILD)/Vmac_top
	@test -e "$(ROMBOOT_DEEP_START)" || (echo "missing ROMBOOT_DEEP_START=$(ROMBOOT_DEEP_START); run make rom-boot-snapshots first" >&2; exit 2)
	@echo "Capturing deep Q700 ROM boot checkpoints from $(ROMBOOT_DEEP_START)"
	@mkdir -p $(ROMBOOT_SNAPSHOT_DIR) $(ROMBOOT_OUTPUT_ROOT)
	$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_OUTPUT_ROOT)/rom_boot_deep_snapshots_trace.log \
		+restore_state=$(ROMBOOT_DEEP_START) \
		+stop_cycle=$(ROMBOOT_DEEP_STOP_CYCLE) \
		+max_insts=$(ROMBOOT_DEEP_MAX) \
		+checkpoint_prefix=$(ROMBOOT_SNAPSHOT_DIR)/q700 \
		+checkpoint_cycle_points=$(ROMBOOT_DEEP_CYCLE_POINTS) \
		+no_waves
	@$(MAKE) rom-boot-checkpoint-inventory

.PHONY: tb-rom-boot-resume
tb-rom-boot-resume: $(ROMBOOT_BUILD)/Vmac_top
	@test -n "$(SNAPSHOT)" || (echo "usage: make tb-rom-boot-resume SNAPSHOT=<path> [ROMBOOT_RESUME_MAX=<n>]" >&2; exit 2)
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_OUTPUT_ROOT)/rom_boot_resume_trace.log \
		+restore_state=$(SNAPSHOT) \
		+max_insts=$(ROMBOOT_RESUME_MAX) \
		+timeout=$(ROMBOOT_RESUME_TIMEOUT) \
		+no_waves

.PHONY: rom-boot-end-breakpoint-smoke
rom-boot-end-breakpoint-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking ROM boot explicit end breakpoint"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_END_BREAKPOINT_TRACE) \
		+end_pc=$(ROMBOOT_END_BREAKPOINT_PC) \
		+lastn_trace=8 \
		+lastn_trace_path=$(ROMBOOT_END_BREAKPOINT_LASTN) \
		+timeout=200000 \
		+max_insts=32 \
		+no_waves > $(ROMBOOT_END_BREAKPOINT_LOG) 2>&1
	@grep -q "reason:     end-breakpoint pc=$(ROMBOOT_END_BREAKPOINT_PC) hit=1" "$(ROMBOOT_END_BREAKPOINT_LOG)"
	@grep -q "reason=end-breakpoint pc=$(ROMBOOT_END_BREAKPOINT_PC) hit=1" "$(ROMBOOT_END_BREAKPOINT_LASTN)"
	@echo "ROM boot end-breakpoint smoke passed (log: $(ROMBOOT_END_BREAKPOINT_LOG), last-N: $(ROMBOOT_END_BREAKPOINT_LASTN))"

.PHONY: rom-boot-overlay-clear-smoke
rom-boot-overlay-clear-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking high-ROM fetch clears the Q700 reset overlay"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_OVERLAY_CLEAR_TRACE) \
		+timeout=200000 \
		+max_insts=4 \
		+no_waves > $(ROMBOOT_OVERLAY_CLEAR_LOG) 2>&1
	@grep -q "overlay auto-cleared" "$(ROMBOOT_OVERLAY_CLEAR_LOG)"
	@grep -q "overlay:    0 (ROM cleared it)" "$(ROMBOOT_OVERLAY_CLEAR_LOG)"
	@echo "ROM boot overlay-clear smoke passed (log: $(ROMBOOT_OVERLAY_CLEAR_LOG))"

.PHONY: rom-boot-data-watch-smoke
rom-boot-data-watch-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking ROM boot data watchpoint logging"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_DATA_WATCH_SMOKE_TRACE) \
		+data_watch=$(ROMBOOT_DATA_WATCH_SMOKE_SPEC) \
		+data_watch_log=$(ROMBOOT_DATA_WATCH_SMOKE_WATCH) \
		+data_watch_limit=4 \
		+timeout=200000 \
		+max_insts=256 \
		+no_waves > $(ROMBOOT_DATA_WATCH_SMOKE_LOG) 2>&1
	@grep -q "data watch:" "$(ROMBOOT_DATA_WATCH_SMOKE_LOG)"
	@grep -q "data watch summary:" "$(ROMBOOT_DATA_WATCH_SMOKE_LOG)"
	@grep -q "^\[data-watch\]" "$(ROMBOOT_DATA_WATCH_SMOKE_WATCH)"
	@echo "ROM boot data watchpoint smoke passed (log: $(ROMBOOT_DATA_WATCH_SMOKE_LOG), watch: $(ROMBOOT_DATA_WATCH_SMOKE_WATCH))"

.PHONY: rom-boot-cache-event-smoke
rom-boot-cache-event-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking ROM boot cache frontier event logging"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=/dev/null \
		+cache_event_log=$(ROMBOOT_CACHE_EVENT_SMOKE_EVENTS) \
		+cache_event_log_limit=$(ROMBOOT_CACHE_EVENT_SMOKE_LIMIT) \
		+timeout=$(ROMBOOT_CACHE_EVENT_SMOKE_TIMEOUT) \
		+max_insts=$(ROMBOOT_CACHE_EVENT_SMOKE_MAX) \
		+no_waves > $(ROMBOOT_CACHE_EVENT_SMOKE_LOG) 2>&1
	@grep -q "cache frontier log:" "$(ROMBOOT_CACHE_EVENT_SMOKE_LOG)"
	@grep -q "^# rom-boot cache frontier event log limit=" "$(ROMBOOT_CACHE_EVENT_SMOKE_EVENTS)"
	@grep -q "cache-event summary:" "$(ROMBOOT_CACHE_EVENT_SMOKE_EVENTS)"
	@grep -q "^# low-memory snapshot 0x00000000..0x000005ff" "$(ROMBOOT_CACHE_EVENT_SMOKE_EVENTS)"
	@grep -q "^lowmem 0x00000000" "$(ROMBOOT_CACHE_EVENT_SMOKE_EVENTS)"
	@grep -q "^lowmem 0x00000400" "$(ROMBOOT_CACHE_EVENT_SMOKE_EVENTS)"
	@echo "ROM boot cache frontier event smoke passed (log: $(ROMBOOT_CACHE_EVENT_SMOKE_LOG), events: $(ROMBOOT_CACHE_EVENT_SMOKE_EVENTS))"

.PHONY: rom-boot-display-watch-smoke
rom-boot-display-watch-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking ROM boot DAFB first-write and pixel/VRAM display watch summaries"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +display_watch_selftest \
		+periph_event_log=$(ROMBOOT_DISPLAY_WATCH_SMOKE_EVENTS) \
		+periph_event_log_limit=16 \
		+periph_event_filter=DAFB,VRAM \
		+no_waves > $(ROMBOOT_DISPLAY_WATCH_SMOKE_LOG) 2>&1
	@grep -q "display dafb-reg  DAFB registers 0xf9800000..0xf9800fff r=0 w=1 first=w .* addr=0xf9800024 value=0x11223344" "$(ROMBOOT_DISPLAY_WATCH_SMOKE_LOG)"
	@grep -q "display dafb-vram DAFB pixel/VRAM aperture 0xf9000000..0xf91fffff r=1 w=1" "$(ROMBOOT_DISPLAY_WATCH_SMOKE_LOG)"
	@grep -q "display_watch_selftest: dafb_reg=PASS dafb_vram=PASS" "$(ROMBOOT_DISPLAY_WATCH_SMOKE_LOG)"
	@grep -q "category=DAFB count=1" "$(ROMBOOT_DISPLAY_WATCH_SMOKE_EVENTS)"
	@grep -q "category=VRAM count=2" "$(ROMBOOT_DISPLAY_WATCH_SMOKE_EVENTS)"
	@grep -q "event=write addr=0xf9800024 value=0x11223344 detail=window=0xf9800000..0xf9800fff,off=0x024,reg=FIRST_HIT,semantic=rom_first_dafb_write" "$(ROMBOOT_DISPLAY_WATCH_SMOKE_EVENTS)"
	@grep -q "event=write addr=0xf9000020 value=0x55667788 detail=pixel-vram-aperture,off=0x00020,op=write,first_activity=1" "$(ROMBOOT_DISPLAY_WATCH_SMOKE_EVENTS)"
	@echo "ROM boot display watch smoke passed (log: $(ROMBOOT_DISPLAY_WATCH_SMOKE_LOG), events: $(ROMBOOT_DISPLAY_WATCH_SMOKE_EVENTS))"

.PHONY: rom-boot-watchdog-smoke
rom-boot-watchdog-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking ROM boot terminal-watchdog helpers"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +watchdog_selftest \
		+stuck_pc_threshold=2 \
		+no_progress_cycles=3 \
		+no_waves > $(ROMBOOT_WATCHDOG_SMOKE_LOG) 2>&1
	@grep -q "watchdog_selftest: stuck_pc=PASS no_progress=PASS" "$(ROMBOOT_WATCHDOG_SMOKE_LOG)"
	@echo "ROM boot watchdog smoke passed (log: $(ROMBOOT_WATCHDOG_SMOKE_LOG))"

.PHONY: rom-boot-outcome-smoke
rom-boot-outcome-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking ROM boot Sad Mac / disk-prompt outcome stops"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom_outcome_selftest \
		+disk_prompt_hit=2 \
		+no_waves > $(ROMBOOT_OUTCOME_SMOKE_LOG) 2>&1
	@grep -q "rom_outcome_selftest: sad=PASS disk=PASS" "$(ROMBOOT_OUTCOME_SMOKE_LOG)"
	@echo "ROM boot outcome smoke passed (log: $(ROMBOOT_OUTCOME_SMOKE_LOG))"

.PHONY: rom-boot-fault-knob-smoke
rom-boot-fault-knob-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking bounded ROM fault-observability knobs"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@rm -rf "$(ROMBOOT_FAULT_KNOB_SMOKE_DUMP)"
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_FAULT_KNOB_SMOKE_TRACE) \
		+stop_on_illegal \
		+stop_on_ifetch_berr \
		+fault_dump_dir=$(ROMBOOT_FAULT_KNOB_SMOKE_DUMP) \
		+fault_dump_bytes=64 \
		+stuck_pc_threshold=64 \
		+no_progress_cycles=1000 \
		+lastn_trace=8 \
		+lastn_trace_path=$(ROMBOOT_FAULT_KNOB_SMOKE_LASTN) \
		+timeout=200000 \
		+max_insts=64 \
		+no_waves > $(ROMBOOT_FAULT_KNOB_SMOKE_LOG) 2>&1
	@grep -q "stop_on_exc: 4" "$(ROMBOOT_FAULT_KNOB_SMOKE_LOG)"
	@grep -q "stop_on_ifetch_berr: enabled" "$(ROMBOOT_FAULT_KNOB_SMOKE_LOG)"
	@grep -q "watchdogs: stuck_pc_threshold=64 no_progress_cycles=1000" "$(ROMBOOT_FAULT_KNOB_SMOKE_LOG)"
	@grep -q "reason=" "$(ROMBOOT_FAULT_KNOB_SMOKE_LASTN)"
	@test -f "$(ROMBOOT_FAULT_KNOB_SMOKE_DUMP)/manifest.tsv"
	@echo "ROM boot fault-observability knob smoke passed (log: $(ROMBOOT_FAULT_KNOB_SMOKE_LOG), lastn: $(ROMBOOT_FAULT_KNOB_SMOKE_LASTN))"

.PHONY: rom-boot-stop-summary
rom-boot-stop-summary: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Running Q700 ROM boot stop-summary helper and summarizing the last-N trace"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_STOP_SUMMARY_TRACE) \
		$(ROMBOOT_STOP_SUMMARY_STOP) \
		+timeout=$(ROMBOOT_STOP_SUMMARY_TIMEOUT) \
		+max_insts=$(ROMBOOT_STOP_SUMMARY_MAX) \
		+lastn_trace=$(ROMBOOT_STOP_SUMMARY_LASTN_CYCLES) \
		+lastn_trace_path=$(ROMBOOT_STOP_SUMMARY_LASTN) \
		+no_waves $(ROMBOOT_STOP_SUMMARY_EXTRA) > $(ROMBOOT_STOP_SUMMARY_LOG) 2>&1
	@python3 $(TOOLS_DIR)/rom_boot_stop_summary.py \
		"$(ROMBOOT_STOP_SUMMARY_LASTN)" \
		--log "$(ROMBOOT_STOP_SUMMARY_LOG)" \
		--trace "$(ROMBOOT_STOP_SUMMARY_TRACE)" \
		--compact > "$(ROMBOOT_STOP_SUMMARY_REPORT)"
	@cat "$(ROMBOOT_STOP_SUMMARY_REPORT)"

.PHONY: rom-boot-stop-summary-smoke
rom-boot-stop-summary-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking ROM boot stop-summary helper and shorthand stop-on-exc path"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(MAKE) rom-boot-stop-summary \
		ROMBOOT_STOP_SUMMARY_LOG=$(ROMBOOT_STOP_SUMMARY_SMOKE_LOG) \
		ROMBOOT_STOP_SUMMARY_TRACE=$(ROMBOOT_STOP_SUMMARY_SMOKE_TRACE) \
		ROMBOOT_STOP_SUMMARY_LASTN=$(ROMBOOT_STOP_SUMMARY_SMOKE_LASTN) \
		ROMBOOT_STOP_SUMMARY_REPORT=$(ROMBOOT_STOP_SUMMARY_SMOKE_REPORT) \
		ROMBOOT_STOP_SUMMARY_STOP='$(ROMBOOT_STOP_SUMMARY_SMOKE_STOP)' \
		ROMBOOT_STOP_SUMMARY_EXTRA='$(ROMBOOT_STOP_SUMMARY_SMOKE_EXTRA)' \
		ROMBOOT_STOP_SUMMARY_LASTN_CYCLES=$(ROMBOOT_STOP_SUMMARY_SMOKE_LASTN_CYCLES) \
		ROMBOOT_STOP_SUMMARY_TIMEOUT=$(ROMBOOT_STOP_SUMMARY_SMOKE_TIMEOUT) \
		ROMBOOT_STOP_SUMMARY_MAX=$(ROMBOOT_STOP_SUMMARY_SMOKE_MAX)
	@if grep -q "\[rom-boot-stop-summary\] summary: reason=stop-exc" "$(ROMBOOT_STOP_SUMMARY_SMOKE_REPORT)"; then \
		grep -Eq "focus_rob_vec=(2|4|11)" "$(ROMBOOT_STOP_SUMMARY_SMOKE_REPORT)"; \
	else \
		grep -q "\[rom-boot-stop-summary\] summary: reason=max-insts-reached" "$(ROMBOOT_STOP_SUMMARY_SMOKE_REPORT)"; \
	fi
	@grep -q "stop_on_exc: 2 4 11" "$(ROMBOOT_STOP_SUMMARY_SMOKE_LOG)"
	@echo "ROM boot stop-summary smoke passed (report: $(ROMBOOT_STOP_SUMMARY_SMOKE_REPORT), log: $(ROMBOOT_STOP_SUMMARY_SMOKE_LOG))"

.PHONY: rom-boot-stop-exc-smoke
rom-boot-stop-exc-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking boundary-based +stop_on_exc consumer selftest"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top \
		+stop_on_exc=2,4,11 \
		+stop_on_exc_selftest \
		> "$(ROMBOOT_STOP_EXC_SMOKE_LOG)" 2>&1
	@grep -q "stop_on_exc_selftest: stopped=PASS armed=PASS" "$(ROMBOOT_STOP_EXC_SMOKE_LOG)"
	@grep -q "source=boundary" "$(ROMBOOT_STOP_EXC_SMOKE_LOG)"
	@echo "ROM boot explicit-stop-on-exc smoke passed (log: $(ROMBOOT_STOP_EXC_SMOKE_LOG))"

.PHONY: rom-boot-trace-enders-smoke
rom-boot-trace-enders-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking ROM boot trace ender classification"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@tmpdir=$$(mktemp -d); trap "rm -rf $$tmpdir" EXIT; \
	rom="$$tmpdir/jump_unmapped.rom"; \
	python3 -c 'import sys; rom=bytearray(64); rom[0:4]=(0x420dbff3).to_bytes(4,"big"); rom[4:8]=(0x2a).to_bytes(4,"big"); rom[0x2a:0x30]=bytes.fromhex("4ef9deadbef0"); open(sys.argv[1],"wb").write(rom)' "$$rom"; \
	$(ROMBOOT_BUILD)/Vmac_top +rom="$$rom" \
		+trace=$(ROMBOOT_TRACE_ENDERS_SMOKE_TRACE) \
		+end_on_ifetch_berr \
		+lastn_trace=8 \
		+lastn_trace_path=$(ROMBOOT_TRACE_ENDERS_SMOKE_LASTN) \
		+timeout=200000 \
		+max_insts=32 \
		+no_waves > $(ROMBOOT_TRACE_ENDERS_SMOKE_LOG) 2>&1
	@grep -q "end_on_ifetch_berr: enabled" "$(ROMBOOT_TRACE_ENDERS_SMOKE_LOG)"
	@grep -q "reason:     end-ifetch-berr addr=0xdeadbef0" "$(ROMBOOT_TRACE_ENDERS_SMOKE_LOG)"
	@grep -q "reason=end-ifetch-berr addr=0xdeadbef0" "$(ROMBOOT_TRACE_ENDERS_SMOKE_LASTN)"
	@echo "ROM boot trace ender smoke passed (log: $(ROMBOOT_TRACE_ENDERS_SMOKE_LOG), last-N: $(ROMBOOT_TRACE_ENDERS_SMOKE_LASTN))"

.PHONY: rom-boot-descriptor-smoke
rom-boot-descriptor-smoke: $(FPGA_TOP_ROM_BUILD)/Vfpga_top
	@echo "Smoke checking no-patch Q700 ROM descriptor selection through fpga_top RTL"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(CPU_MODEL_RUN_PREFIX)$(FPGA_TOP_ROM_BUILD)/Vfpga_top +rom=$(ROM) \
		+ppm=$(BUILD_DIR)/fpga_top_rom/descriptor_smoke.ppm \
		+max_insts=$(ROMBOOT_DESCRIPTOR_SMOKE_MAX) \
		+timeout=$(ROMBOOT_DESCRIPTOR_SMOKE_TIMEOUT) \
		+expect_q700_feature=0xc108ffc7 \
		> $(ROMBOOT_DESCRIPTOR_SMOKE_LOG) 2>&1
	@if ! grep -q "q700_descriptor_selected=1" "$(ROMBOOT_DESCRIPTOR_SMOKE_LOG)"; then \
		echo "Q700 descriptor table entry was not accepted" >&2; \
		tail -80 "$(ROMBOOT_DESCRIPTOR_SMOKE_LOG)" >&2; \
		exit 1; \
	fi
	@if ! grep -q "q700_descriptor_selected=1 entry=0x4080390c feature=0xc108ffc7" "$(ROMBOOT_DESCRIPTOR_SMOKE_LOG)"; then \
		echo "Q700 descriptor feature word did not match fpga_top RTL state" >&2; \
		tail -80 "$(ROMBOOT_DESCRIPTOR_SMOKE_LOG)" >&2; \
		exit 1; \
	fi
	@echo "Q700 descriptor RTL smoke passed (log: $(ROMBOOT_DESCRIPTOR_SMOKE_LOG))"

.PHONY: rom-boot-fastdiag-smoke
rom-boot-fastdiag-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking opt-in ROM diagnostic-loop patches"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_FASTDIAG_TRACE) \
		+rom_patch=diag-loops \
		+end_pc=$(ROMBOOT_FASTDIAG_PC) \
		+timeout=$(ROMBOOT_FASTDIAG_TIMEOUT) \
		+max_insts=$(ROMBOOT_FASTDIAG_MAX) \
		+no_waves > $(ROMBOOT_FASTDIAG_LOG) 2>&1
	@grep -q "+rom_patch=checksum-fast off=0x4751c" "$(ROMBOOT_FASTDIAG_LOG)"
	@grep -q "+rom_patch=meminit-fast off=0x4753e" "$(ROMBOOT_FASTDIAG_LOG)"
	@if grep -q "+rom_patch=diag-loops-unsafe off=0x4752c" "$(ROMBOOT_FASTDIAG_LOG)"; then \
		echo "fast diagnostic run used unsafe whole-helper RAM skip" >&2; \
		tail -80 "$(ROMBOOT_FASTDIAG_LOG)" >&2; \
		exit 1; \
	fi
	@grep -q "reason:     end-breakpoint pc=$(ROMBOOT_FASTDIAG_PC) hit=1" "$(ROMBOOT_FASTDIAG_LOG)"
	@if [ "$$(grep -c '^40847516 ' "$(ROMBOOT_FASTDIAG_TRACE)")" -gt 8 ]; then \
		echo "fast diagnostic run did not bound the checksum loop" >&2; \
		tail -80 "$(ROMBOOT_FASTDIAG_TRACE)" >&2; \
		exit 1; \
	fi
	@if grep -q "^40807116 " "$(ROMBOOT_FASTDIAG_TRACE)"; then \
		echo "fast diagnostic run unexpectedly entered ASC chime delay loop" >&2; \
		tail -80 "$(ROMBOOT_FASTDIAG_TRACE)" >&2; \
		exit 1; \
	fi
	@echo "ROM boot fast diagnostic patch smoke passed (log: $(ROMBOOT_FASTDIAG_LOG), trace: $(ROMBOOT_FASTDIAG_TRACE))"

.PHONY: rom-boot-chime-delay-smoke
rom-boot-chime-delay-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking opt-in ASC chime-delay ROM patch"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_CHIME_DELAY_TRACE) \
		+rom_patch=clean-fastdiag,chime-delay \
		+end_pc=$(ROMBOOT_CHIME_DELAY_PC) \
		+timeout=$(ROMBOOT_CHIME_DELAY_TIMEOUT) \
		+max_insts=$(ROMBOOT_CHIME_DELAY_MAX) \
		+no_waves > $(ROMBOOT_CHIME_DELAY_LOG) 2>&1
	@grep -q "+rom_patch=chime-delay off=0x07118" "$(ROMBOOT_CHIME_DELAY_LOG)"
	@grep -q "+rom_patch=meminit-fast off=0x4753e" "$(ROMBOOT_CHIME_DELAY_LOG)"
	@grep -q "reason:     end-breakpoint pc=$(ROMBOOT_CHIME_DELAY_PC) hit=1" "$(ROMBOOT_CHIME_DELAY_LOG)"
	@echo "ROM boot chime-delay patch smoke passed (log: $(ROMBOOT_CHIME_DELAY_LOG), trace: $(ROMBOOT_CHIME_DELAY_TRACE))"

.PHONY: rom-boot-timer-delay-smoke
rom-boot-timer-delay-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking opt-in timer-delay ROM patch"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+rom_patch=diag-loops,chime-delay,timer-delay \
		+end_pc=$(ROMBOOT_TIMER_DELAY_PC) \
		+stop_on_exc=2,4,11 \
		+timeout=$(ROMBOOT_TIMER_DELAY_TIMEOUT) \
		+max_insts=$(ROMBOOT_TIMER_DELAY_MAX) \
		+periph_event_log=$(ROMBOOT_TIMER_DELAY_EVENTS) \
		+periph_event_filter=DAFB,VRAM \
		+periph_event_log_limit=256 \
		+lastn_trace=128 \
		+lastn_trace_path=$(ROMBOOT_TIMER_DELAY_LASTN) \
		+no_waves > $(ROMBOOT_TIMER_DELAY_LOG) 2>&1
	@grep -q "+rom_patch=timer-delay off=0x00888" "$(ROMBOOT_TIMER_DELAY_LOG)"
	@grep -q "+rom_patch=timer-delay off=0x0088a" "$(ROMBOOT_TIMER_DELAY_LOG)"
	@grep -q "reason:     end-breakpoint pc=$(ROMBOOT_TIMER_DELAY_PC) hit=1" "$(ROMBOOT_TIMER_DELAY_LOG)"
	@grep -q "display dafb-reg .* w=1" "$(ROMBOOT_TIMER_DELAY_LOG)"
	@grep -q "display dafb-vram .* w=0" "$(ROMBOOT_TIMER_DELAY_LOG)"
	@echo "ROM boot timer-delay patch smoke passed (log: $(ROMBOOT_TIMER_DELAY_LOG), events: $(ROMBOOT_TIMER_DELAY_EVENTS), lastn: $(ROMBOOT_TIMER_DELAY_LASTN))"

.PHONY: rom-boot-frontier-checkpoint
rom-boot-frontier-checkpoint: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Saving current ROM frontier checkpoint at $(ROMBOOT_TIMER_DELAY_PC)"
	@mkdir -p $(ROMBOOT_FRONTIER_CHECKPOINT_DIR) $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_FRONTIER_TRACE) \
		$(ROMBOOT_FRONTIER_PATCH_ARG) \
		+end_pc=$(ROMBOOT_TIMER_DELAY_PC) \
		+stop_on_rom_faults \
		+timeout=$(ROMBOOT_TIMER_DELAY_TIMEOUT) \
		+max_insts=$(ROMBOOT_TIMER_DELAY_MAX) \
		+save_state=$(ROMBOOT_FRONTIER_CHECKPOINT) \
		+periph_event_log=$(ROMBOOT_FRONTIER_EVENTS) \
		+periph_event_filter=DAFB,VRAM \
		+periph_event_log_limit=256 \
		+lastn_trace=128 \
		+lastn_trace_path=$(ROMBOOT_FRONTIER_LASTN) \
		+no_waves > $(ROMBOOT_FRONTIER_LOG) 2>&1
	@grep -q "reason:     end-breakpoint pc=$(ROMBOOT_TIMER_DELAY_PC) hit=1" "$(ROMBOOT_FRONTIER_LOG)"
	@grep -q "checkpoint saved: $(ROMBOOT_FRONTIER_CHECKPOINT)" "$(ROMBOOT_FRONTIER_LOG)"
	@echo "ROM frontier checkpoint saved: $(ROMBOOT_FRONTIER_CHECKPOINT)"
	@echo "  log: $(ROMBOOT_FRONTIER_LOG)"
	@echo "  last-N: $(ROMBOOT_FRONTIER_LASTN)"

.PHONY: rom-boot-frontier-restore-smoke
rom-boot-frontier-restore-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@test -e "$(ROMBOOT_FRONTIER_CHECKPOINT)" || (echo "missing ROMBOOT_FRONTIER_CHECKPOINT=$(ROMBOOT_FRONTIER_CHECKPOINT); run make rom-boot-frontier-checkpoint first" >&2; exit 2)
	@echo "Smoke restoring current ROM frontier checkpoint $(ROMBOOT_FRONTIER_CHECKPOINT)"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_FRONTIER_RESTORE_TRACE) \
		+restore_state=$(ROMBOOT_FRONTIER_CHECKPOINT) \
		$(ROMBOOT_FRONTIER_PATCH_ARG) \
		+max_insts=$(ROMBOOT_FRONTIER_RESTORE_MAX) \
		+timeout=$(ROMBOOT_FRONTIER_RESTORE_TIMEOUT) \
		+no_waves > $(ROMBOOT_FRONTIER_RESTORE_LOG) 2>&1
	@grep -q "checkpoint restored: $(ROMBOOT_FRONTIER_CHECKPOINT)" "$(ROMBOOT_FRONTIER_RESTORE_LOG)"
	@grep -q "last_pc:    $(ROMBOOT_TIMER_DELAY_PC)" "$(ROMBOOT_FRONTIER_RESTORE_LOG)"
	@echo "ROM frontier checkpoint restore smoke passed (log: $(ROMBOOT_FRONTIER_RESTORE_LOG))"

.PHONY: rom-boot-arch-checkpoint
rom-boot-arch-checkpoint: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Saving portable ROM boot architectural checkpoint to $(ROMBOOT_ARCH_CHECKPOINT)"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_ARCH_CHECKPOINT_TRACE) \
		+max_insts=$(ROMBOOT_ARCH_CHECKPOINT_MAX) \
		+timeout=$(ROMBOOT_ARCH_CHECKPOINT_TIMEOUT) \
		+arch_checkpoint=$(ROMBOOT_ARCH_CHECKPOINT) \
		+no_waves $(ROMBOOT_ARCH_CHECKPOINT_EXTRA) > $(ROMBOOT_ARCH_CHECKPOINT_LOG) 2>&1
	@grep -q "arch checkpoint saved: $(ROMBOOT_ARCH_CHECKPOINT)" "$(ROMBOOT_ARCH_CHECKPOINT_LOG)"
	@echo "ROM architectural checkpoint saved: $(ROMBOOT_ARCH_CHECKPOINT)"
	@echo "  log: $(ROMBOOT_ARCH_CHECKPOINT_LOG)"

.PHONY: rom-boot-arch-resume
rom-boot-arch-resume: $(ROMBOOT_BUILD)/Vmac_top
	@test -e "$(ROMBOOT_ARCH_RESUME)" || (echo "missing ROMBOOT_ARCH_RESUME=$(ROMBOOT_ARCH_RESUME); run make rom-boot-arch-checkpoint or pass ROMBOOT_ARCH_RESUME=<path>" >&2; exit 2)
	@echo "Resuming ROM boot from portable architectural checkpoint $(ROMBOOT_ARCH_RESUME)"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_ARCH_RESUME_TRACE) \
		+arch_replay=$(ROMBOOT_ARCH_RESUME) \
		+max_insts=$(ROMBOOT_ARCH_RESUME_MAX) \
		+timeout=$(ROMBOOT_ARCH_RESUME_TIMEOUT) \
		+no_waves $(ROMBOOT_ARCH_RESUME_EXTRA) > $(ROMBOOT_ARCH_RESUME_LOG) 2>&1
	@grep -q "arch replay restored: $(ROMBOOT_ARCH_RESUME)" "$(ROMBOOT_ARCH_RESUME_LOG)"
	@echo "ROM architectural resume completed (checkpoint: $(ROMBOOT_ARCH_RESUME), log: $(ROMBOOT_ARCH_RESUME_LOG), trace: $(ROMBOOT_ARCH_RESUME_TRACE))"

.PHONY: rom-boot-arch-checkpoint-smoke
rom-boot-arch-checkpoint-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking portable ROM boot architectural checkpoint capture"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_ARCH_CHECKPOINT_SMOKE_TRACE) \
		+max_insts=$(ROMBOOT_ARCH_CHECKPOINT_SMOKE_MAX) \
		+timeout=$(ROMBOOT_ARCH_CHECKPOINT_SMOKE_TIMEOUT) \
		+arch_checkpoint=$(ROMBOOT_ARCH_CHECKPOINT_SMOKE) \
		+no_waves > $(ROMBOOT_ARCH_CHECKPOINT_SMOKE_LOG) 2>&1
	@$(ROMBOOT_ARCH_CHECKPOINT_SUMMARY) "$(ROMBOOT_ARCH_CHECKPOINT_SMOKE)" \
		--check-replayable --compact
	@grep -q "arch checkpoint saved: $(ROMBOOT_ARCH_CHECKPOINT_SMOKE)" "$(ROMBOOT_ARCH_CHECKPOINT_SMOKE_LOG)"
	@echo "ROM architectural checkpoint smoke passed (checkpoint: $(ROMBOOT_ARCH_CHECKPOINT_SMOKE), log: $(ROMBOOT_ARCH_CHECKPOINT_SMOKE_LOG))"

.PHONY: rom-boot-arch-replay-smoke
rom-boot-arch-replay-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking portable ROM boot architectural checkpoint replay"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=/dev/null \
		+max_insts=$(ROMBOOT_ARCH_REPLAY_CAPTURE_MAX) \
		+timeout=$(ROMBOOT_ARCH_REPLAY_CAPTURE_TIMEOUT) \
		+arch_checkpoint=$(ROMBOOT_ARCH_REPLAY_SMOKE) \
		+no_waves > $(ROMBOOT_ARCH_REPLAY_CAPTURE_LOG) 2>&1
	@$(ROMBOOT_ARCH_CHECKPOINT_SUMMARY) "$(ROMBOOT_ARCH_REPLAY_SMOKE)" \
		--check-replayable --compact
	@grep -q "arch checkpoint saved: $(ROMBOOT_ARCH_REPLAY_SMOKE)" "$(ROMBOOT_ARCH_REPLAY_CAPTURE_LOG)"
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_ARCH_REPLAY_TRACE) \
		+arch_replay=$(ROMBOOT_ARCH_REPLAY_SMOKE) \
		+max_insts=$(ROMBOOT_ARCH_REPLAY_MAX) \
		+timeout=$(ROMBOOT_ARCH_REPLAY_TIMEOUT) \
		+no_waves > $(ROMBOOT_ARCH_REPLAY_LOG) 2>&1
	@grep -q "arch replay restored: $(ROMBOOT_ARCH_REPLAY_SMOKE)" "$(ROMBOOT_ARCH_REPLAY_LOG)"
	@grep -Eq "io_state=(reset|via1-v1) overlay=[01]" "$(ROMBOOT_ARCH_REPLAY_LOG)"
	@grep -q "reason:     max-insts-reached" "$(ROMBOOT_ARCH_REPLAY_LOG)"
	@echo "ROM architectural replay smoke passed (checkpoint: $(ROMBOOT_ARCH_REPLAY_SMOKE), log: $(ROMBOOT_ARCH_REPLAY_LOG), trace: $(ROMBOOT_ARCH_REPLAY_TRACE))"

.PHONY: rom-boot-arch-roundtrip-smoke
rom-boot-arch-roundtrip-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking architectural replay round-trip against a continuous short run"
	@mkdir -p $(ROMBOOT_ARCH_ROUNDTRIP_DIR)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=/dev/null \
		+max_insts=$(ROMBOOT_ARCH_ROUNDTRIP_SEED_MAX) \
		+timeout=$(ROMBOOT_ARCH_ROUNDTRIP_TIMEOUT) \
		+arch_checkpoint=$(ROMBOOT_ARCH_ROUNDTRIP_SEED) \
		+no_waves > $(ROMBOOT_ARCH_ROUNDTRIP_SEED_LOG) 2>&1
	@$(ROMBOOT_ARCH_CHECKPOINT_SUMMARY) "$(ROMBOOT_ARCH_ROUNDTRIP_SEED)" \
		--check-replayable --compact
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=/dev/null \
		+max_insts=$(ROMBOOT_ARCH_ROUNDTRIP_FINAL_MAX) \
		+timeout=$(ROMBOOT_ARCH_ROUNDTRIP_TIMEOUT) \
		+arch_checkpoint=$(ROMBOOT_ARCH_ROUNDTRIP_CONTINUOUS) \
		+no_waves > $(ROMBOOT_ARCH_ROUNDTRIP_CONTINUOUS_LOG) 2>&1
	@$(ROMBOOT_ARCH_CHECKPOINT_SUMMARY) "$(ROMBOOT_ARCH_ROUNDTRIP_CONTINUOUS)" \
		--check-replayable --compact
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=/dev/null \
		+arch_replay=$(ROMBOOT_ARCH_ROUNDTRIP_SEED) \
		+max_insts=$(ROMBOOT_ARCH_ROUNDTRIP_FINAL_MAX) \
		+timeout=$(ROMBOOT_ARCH_ROUNDTRIP_TIMEOUT) \
		+arch_checkpoint=$(ROMBOOT_ARCH_ROUNDTRIP_REPLAYED) \
		+no_waves > $(ROMBOOT_ARCH_ROUNDTRIP_REPLAYED_LOG) 2>&1
	@$(ROMBOOT_ARCH_CHECKPOINT_SUMMARY) "$(ROMBOOT_ARCH_ROUNDTRIP_REPLAYED)" \
		--check-replayable --compact
	@$(ROMBOOT_ARCH_COMPARE) "$(ROMBOOT_ARCH_ROUNDTRIP_CONTINUOUS)" \
		"$(ROMBOOT_ARCH_ROUNDTRIP_REPLAYED)" --check-replayable
	@echo "ROM architectural round-trip smoke passed (dir: $(ROMBOOT_ARCH_ROUNDTRIP_DIR))"

.PHONY: rom-boot-arch-endpc-replay-smoke
rom-boot-arch-endpc-replay-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Smoke checking end-PC architectural checkpoint replay at $(ROMBOOT_ARCH_ENDPC)"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=/dev/null \
		+end_pc=$(ROMBOOT_ARCH_ENDPC) \
		+max_insts=$(ROMBOOT_ARCH_ENDPC_CAPTURE_MAX) \
		+timeout=$(ROMBOOT_ARCH_ENDPC_CAPTURE_TIMEOUT) \
		+arch_checkpoint=$(ROMBOOT_ARCH_ENDPC_CHECKPOINT) \
		+no_waves > $(ROMBOOT_ARCH_ENDPC_CAPTURE_LOG) 2>&1
	@grep -q "reason:     end-breakpoint pc=$(ROMBOOT_ARCH_ENDPC)" "$(ROMBOOT_ARCH_ENDPC_CAPTURE_LOG)"
	@$(ROMBOOT_ARCH_CHECKPOINT_SUMMARY) "$(ROMBOOT_ARCH_ENDPC_CHECKPOINT)" \
		--check-replayable --compact
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_ARCH_ENDPC_REPLAY_TRACE) \
		+arch_replay=$(ROMBOOT_ARCH_ENDPC_CHECKPOINT) \
		+max_insts=$(ROMBOOT_ARCH_ENDPC_REPLAY_MAX) \
		+timeout=$(ROMBOOT_ARCH_ENDPC_REPLAY_TIMEOUT) \
		+no_waves > $(ROMBOOT_ARCH_ENDPC_REPLAY_LOG) 2>&1
	@grep -q "arch replay restored: $(ROMBOOT_ARCH_ENDPC_CHECKPOINT)" "$(ROMBOOT_ARCH_ENDPC_REPLAY_LOG)"
	@grep -Eq "arch replay (IO/peripheral state reset|restored VIA1/RTC-line state)" "$(ROMBOOT_ARCH_ENDPC_REPLAY_LOG)"
	@grep -Eq "io_state=(reset|via1-v1) overlay=[01]" "$(ROMBOOT_ARCH_ENDPC_REPLAY_LOG)"
	@grep -q "reason:     max-insts-reached" "$(ROMBOOT_ARCH_ENDPC_REPLAY_LOG)"
	@echo "ROM end-PC architectural replay smoke passed (checkpoint: $(ROMBOOT_ARCH_ENDPC_CHECKPOINT), log: $(ROMBOOT_ARCH_ENDPC_REPLAY_LOG), trace: $(ROMBOOT_ARCH_ENDPC_REPLAY_TRACE))"

.PHONY: rom-boot-arch-aline-frontier-replay
rom-boot-arch-aline-frontier-replay: $(ROMBOOT_BUILD)/Vmac_top
	@echo "Capturing pre-dispatch A-line frontier architectural checkpoint at $(ROMBOOT_ARCH_ALINE_FRONTIER_PC) hit $(ROMBOOT_ARCH_ALINE_FRONTIER_PC_HIT)"
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=/dev/null \
		$(ROMBOOT_FRONTIER_PATCH_ARG) \
		+end_pc=$(ROMBOOT_ARCH_ALINE_FRONTIER_PC) \
		+end_pc_hit=$(ROMBOOT_ARCH_ALINE_FRONTIER_PC_HIT) \
		+data_watch=$(ROMBOOT_ARCH_ALINE_FRONTIER_DATA_WATCH) \
		+data_watch_log=$(ROMBOOT_OUTPUT_ROOT)/rom_boot_arch_aline_frontier_data_watch.log \
		+data_watch_limit=20000 \
		+cache_event_log=$(ROMBOOT_ARCH_ALINE_FRONTIER_CACHE_LOG) \
		+cache_event_log_limit=40000 \
		+max_insts=$(ROMBOOT_ARCH_ALINE_FRONTIER_CAPTURE_MAX) \
		+timeout=$(ROMBOOT_ARCH_ALINE_FRONTIER_CAPTURE_TIMEOUT) \
		+arch_checkpoint=$(ROMBOOT_ARCH_ALINE_FRONTIER_CHECKPOINT) \
		+no_waves > $(ROMBOOT_ARCH_ALINE_FRONTIER_CAPTURE_LOG) 2>&1
	@grep -q "reason:     end-breakpoint pc=$(ROMBOOT_ARCH_ALINE_FRONTIER_PC)" "$(ROMBOOT_ARCH_ALINE_FRONTIER_CAPTURE_LOG)"
	@$(ROMBOOT_ARCH_CHECKPOINT_SUMMARY) "$(ROMBOOT_ARCH_ALINE_FRONTIER_CHECKPOINT)" \
		--check-replayable --compact
	@$(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_ARCH_ALINE_FRONTIER_REPLAY_TRACE) \
		$(ROMBOOT_FRONTIER_PATCH_ARG) \
		+arch_replay=$(ROMBOOT_ARCH_ALINE_FRONTIER_CHECKPOINT) \
		+max_insts=$(ROMBOOT_ARCH_ALINE_FRONTIER_REPLAY_MAX) \
		+timeout=$(ROMBOOT_ARCH_ALINE_FRONTIER_REPLAY_TIMEOUT) \
		+no_waves > $(ROMBOOT_ARCH_ALINE_FRONTIER_REPLAY_LOG) 2>&1
	@grep -q "arch replay restored: $(ROMBOOT_ARCH_ALINE_FRONTIER_CHECKPOINT)" "$(ROMBOOT_ARCH_ALINE_FRONTIER_REPLAY_LOG)"
	@grep -Eq "arch replay (IO/peripheral state reset|restored VIA1/RTC-line state)" "$(ROMBOOT_ARCH_ALINE_FRONTIER_REPLAY_LOG)"
	@grep -Eq "io_state=(reset|via1-v1) overlay=[01]" "$(ROMBOOT_ARCH_ALINE_FRONTIER_REPLAY_LOG)"
	@grep -q "reason:     max-insts-reached" "$(ROMBOOT_ARCH_ALINE_FRONTIER_REPLAY_LOG)"
	@echo "ROM A-line frontier architectural replay passed (checkpoint: $(ROMBOOT_ARCH_ALINE_FRONTIER_CHECKPOINT), log: $(ROMBOOT_ARCH_ALINE_FRONTIER_REPLAY_LOG), trace: $(ROMBOOT_ARCH_ALINE_FRONTIER_REPLAY_TRACE))"

.PHONY: rom-boot-arch-checkpoint-summary
rom-boot-arch-checkpoint-summary:
	@test -e "$(ROMBOOT_ARCH_CHECKPOINT_SMOKE)" || \
		(echo "missing $(ROMBOOT_ARCH_CHECKPOINT_SMOKE); run make rom-boot-arch-checkpoint-smoke first" >&2; exit 2)
	@$(ROMBOOT_ARCH_CHECKPOINT_SUMMARY) "$(ROMBOOT_ARCH_CHECKPOINT_SMOKE)" \
		--check-replayable --compact

.PHONY: rom-boot-snapshot-smoke
rom-boot-snapshot-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@$(MAKE) rom-boot-checkpoint-inventory
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT) $(ROMBOOT_SNAPSHOT_SMOKE_LOG_DIR)
	@$(ROMBOOT_CHECKPOINT_INVENTORY) "$(ROMBOOT_SNAPSHOT_DIR)" \
		--expected-commit "$(ROMBOOT_SNAPSHOT_POINTS)" \
		--expected-cycle "$(ROMBOOT_DEEP_CYCLE_POINTS)" \
		--require-final --forbid-unknown --require-current-metadata \
		--list-smoke-specs > "$(ROMBOOT_SNAPSHOT_SMOKE_SPEC)"
	@set -e; \
	while IFS="	" read -r f committed pc kind point; do \
		base=$$(basename "$$f" .vlt); \
		log="$(ROMBOOT_SNAPSHOT_SMOKE_LOG_DIR)/$$base.log"; \
		echo "Smoke restoring $$f (kind=$$kind point=$$point expected committed=$$committed pc=$$pc)"; \
		if $(MAKE) tb-rom-boot-resume SNAPSHOT="$$f" ROMBOOT_RESUME_MAX=1 ROMBOOT_RESUME_TIMEOUT=$(ROMBOOT_RESUME_TIMEOUT) > "$$log" 2>&1; then \
			$(ROMBOOT_CHECKPOINT_INVENTORY) "$(ROMBOOT_SNAPSHOT_DIR)" \
				--verify-restore-log "$$log" --snapshot "$$f" \
				--expected-committed "$$committed" --expected-pc "$$pc"; \
		else \
			echo "restore failed for $$f (log: $$log)" >&2; \
			tail -80 "$$log" >&2; \
			exit 1; \
		fi; \
	done < "$(ROMBOOT_SNAPSHOT_SMOKE_SPEC)"

.PHONY: rom-boot-checkpoint-inventory
rom-boot-checkpoint-inventory:
	@$(ROMBOOT_CHECKPOINT_INVENTORY) "$(ROMBOOT_SNAPSHOT_DIR)" \
		--expected-commit "$(ROMBOOT_SNAPSHOT_POINTS)" \
		--expected-cycle "$(ROMBOOT_DEEP_CYCLE_POINTS)" \
		--require-final --forbid-unknown --require-current-metadata

.PHONY: rom-boot-snapshot-corrupt-smoke
rom-boot-snapshot-corrupt-smoke: $(ROMBOOT_BUILD)/Vmac_top
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	@printf 'not a Q700 checkpoint\n' > $(ROMBOOT_OUTPUT_ROOT)/q700.corrupt.vlt
	@set -e; \
	ulimit -c 0; \
	if $(ROMBOOT_BUILD)/Vmac_top +rom=$(ROM) \
		+trace=$(ROMBOOT_OUTPUT_ROOT)/rom_boot_corrupt_restore_trace.log \
		+restore_state=$(ROMBOOT_OUTPUT_ROOT)/q700.corrupt.vlt \
		+max_insts=1 +timeout=$(ROMBOOT_RESUME_TIMEOUT) +no_waves \
		> $(ROMBOOT_OUTPUT_ROOT)/rom_boot_corrupt_restore.log 2>&1; then \
		echo "corrupt checkpoint restore unexpectedly succeeded" >&2; \
		rm -f $(ROMBOOT_OUTPUT_ROOT)/q700.corrupt.vlt; \
		exit 1; \
	else \
		echo "corrupt checkpoint restore rejected as expected (log: $(ROMBOOT_OUTPUT_ROOT)/rom_boot_corrupt_restore.log)"; \
		rm -f $(ROMBOOT_OUTPUT_ROOT)/q700.corrupt.vlt; \
	fi

# $(ROMBOOT_BUILD)/Vmac_top was the flat mac_top build (rtl/core/* +
# rtl/mac/* + rtl/mac_top.v, RTL_SRCS/RTL_DEPS) with tb_rom_boot.cpp in the
# main-cpp slot — same retired monorepo top as `make sim` (see the CPU RTL
# note near the top of this file).  Every tb-rom-boot-*-smoke /
# rom-boot-snapshot-* target below still lists this as a prerequisite; make
# that prerequisite fail loud with one message instead of letting each one
# hit its own confusing "file not found" from a stale RTL_SRCS/RTL_DEPS
# list.  Use tb-fpga-top-rom / tb-via1-lockstep / tb-axi-lockstep /
# tb-dafb-lockstep / tb-scc-uart-loopback (fpga_top-based, CPU=m68k) for
# the equivalent full-stack ROM-boot coverage that still works.
$(ROMBOOT_BUILD)/Vmac_top:
	@echo "tb-rom-boot-* / rom-boot-snapshot-* were removed: rtl/core/* moved to the cpu/ submodule, so the flat mac_top ROM-boot harness no longer builds from this repo. Use tb-fpga-top-rom / tb-via1-lockstep / tb-axi-lockstep / tb-dafb-lockstep / tb-scc-uart-loopback instead." >&2
	@exit 2

# ──────────────────────────────────────────────────────────────────────────────
# async_fifo unit testbench (CDC primitive — Gray-coded dual-clock FIFO)
#
# Standalone Verilator build — async_fifo.v in isolation, driven by
# tb/tb_async_fifo.cpp which models two independent virtual clocks.
# ──────────────────────────────────────────────────────────────────────────────
ASYNC_FIFO_RTL   := $(RTL_DIR)/board/async_fifo.v
ASYNC_FIFO_BUILD := $(BUILD_DIR)/async_fifo

.PHONY: tb-async-fifo
tb-async-fifo: $(ASYNC_FIFO_BUILD)/Vasync_fifo
	@echo "Running async_fifo unit tb..."
	$(ASYNC_FIFO_BUILD)/Vasync_fifo

$(ASYNC_FIFO_BUILD)/Vasync_fifo: $(ASYNC_FIFO_RTL) $(TB_DIR)/tb_async_fifo.cpp
	@mkdir -p $(ASYNC_FIFO_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(ASYNC_FIFO_BUILD) \
		--top-module async_fifo \
		$(ASYNC_FIFO_RTL) \
		$(TB_DIR)/tb_async_fifo.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# axi_async_bridge unit testbench (AXI4 CDC bridge over 5 async_fifos)
#
# Standalone Verilator build — axi_async_bridge.v + async_fifo.v, driven
# by tb/tb_axi_async_bridge.cpp which models two independent clocks and
# implements a software AXI4 master + slave model.
# ──────────────────────────────────────────────────────────────────────────────
AXI_ASYNC_RTL   := \
	$(RTL_DIR)/board/async_fifo.v \
	$(RTL_DIR)/board/pulse_cdc.v \
	$(RTL_DIR)/soc/axi_bridge_stale_sink.v \
	$(RTL_DIR)/soc/axi_bridge_w_pad.v \
	$(RTL_DIR)/soc/axi_async_bridge.v
AXI_ASYNC_BUILD := $(BUILD_DIR)/axi_async_bridge

.PHONY: tb-axi-async-bridge
tb-axi-async-bridge: $(AXI_ASYNC_BUILD)/Vaxi_async_bridge
	@echo "Running axi_async_bridge unit tb..."
	$(AXI_ASYNC_BUILD)/Vaxi_async_bridge

$(AXI_ASYNC_BUILD)/Vaxi_async_bridge: $(AXI_ASYNC_RTL) $(TB_DIR)/tb_axi_async_bridge.cpp
	@mkdir -p $(AXI_ASYNC_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(AXI_ASYNC_BUILD) \
		--top-module axi_async_bridge \
		$(AXI_ASYNC_RTL) \
		$(TB_DIR)/tb_axi_async_bridge.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# pb lane-shim equivalence testbench (S1 narrowed-payload CDC)
#
# Proves rtl/soc/axi_pb_s1_cdc.v (axi_pb_lane_narrow -> axi_async_bridge#(32)
# -> axi_pb_lane_widen, rtl/soc/axi_pb_lane_shim.v) is behaviourally identical
# to the 128-bit axi_async_bridge it replaces on the xbar-S1 -> peripheral_bus
# path.  tb/tb_pb_lane_shim.v runs both chains in parallel against two copies
# of a slave model that replicates peripheral_bus.v's byte-lane policy exactly;
# tb/tb_pb_lane_shim.cpp drives the identical randomized + directed stream into
# both and compares responses, addressed-lane read data, and the final captured
# write memories.
# ──────────────────────────────────────────────────────────────────────────────
PB_LANE_SHIM_RTL := \
	$(RTL_DIR)/board/async_fifo.v \
	$(RTL_DIR)/board/pulse_cdc.v \
	$(RTL_DIR)/soc/axi_bridge_stale_sink.v \
	$(RTL_DIR)/soc/axi_bridge_w_pad.v \
	$(RTL_DIR)/soc/axi_async_bridge.v \
	$(RTL_DIR)/soc/axi_pb_lane_shim.v \
	$(RTL_DIR)/soc/axi_pb_s1_cdc.v \
	$(TB_DIR)/tb_pb_lane_shim.v
PB_LANE_SHIM_BUILD := $(BUILD_DIR)/pb_lane_shim

.PHONY: tb-pb-lane-shim
tb-pb-lane-shim: $(PB_LANE_SHIM_BUILD)/Vtb_pb_lane_shim
	@echo "Running pb lane-shim equivalence tb..."
	$(PB_LANE_SHIM_BUILD)/Vtb_pb_lane_shim

$(PB_LANE_SHIM_BUILD)/Vtb_pb_lane_shim: $(PB_LANE_SHIM_RTL) $(TB_DIR)/tb_pb_lane_shim.cpp
	@mkdir -p $(PB_LANE_SHIM_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-I$(RTL_DIR)/soc -I$(RTL_DIR)/board -I$(RTL_DIR) \
		-Mdir $(PB_LANE_SHIM_BUILD) \
		--top-module tb_pb_lane_shim \
		$(PB_LANE_SHIM_RTL) \
		$(TB_DIR)/tb_pb_lane_shim.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# l2c (L2 system cache) unit testbench -- rtl/soc/l2c.v + submodules.
#
# tb_l2c.cpp drives tb_l2c.v (a thin AXI4 pass-through
# wrapper fixing the bypass-window test parameters) via a concurrent-
# outstanding AXI4 master model, backed by a backpressuring, latency-
# randomizing AXI4 memory model on l2c's own m_axi master port.
#
# Two builds share tb_l2c.cpp:
#   tb-l2c            -- normal active-cache model (L2_BYPASS_ALL=0).
#   tb-l2c-bypass-all -- L2_BYPASS_ALL=1 pass-through-only model, built
#                        with a distinct --prefix so both coexist; only
#                        runs the pass-through equivalence check (see
#                        BYPASS_ALL_BUILD in tb_l2c.cpp).
# ──────────────────────────────────────────────────────────────────────────────
L2C_RTL := \
	$(RTL_DIR)/soc/l2c_pri8.v \
	$(RTL_DIR)/soc/l2c_reset.v \
	$(RTL_DIR)/soc/l2c_tags.v \
	$(RTL_DIR)/soc/l2c_data.v \
	$(RTL_DIR)/soc/l2c_victim_sel.v \
	$(RTL_DIR)/soc/l2c_mshr.v \
	$(RTL_DIR)/soc/l2c_victim.v \
	$(RTL_DIR)/soc/l2c_bypass.v \
	$(RTL_DIR)/soc/l2c_ctrl.v \
	$(RTL_DIR)/soc/l2c.v \
	$(TB_DIR)/tb_l2c.v
L2C_BUILD      := $(BUILD_DIR)/l2c
L2C_BYPASS_BUILD := $(BUILD_DIR)/l2c_bypass_all
# ── STRESS SIZING IS LOAD-BEARING (race audit 2026-09-18) ─────────────────
# The randomized scoreboard is the ONLY thing that catches three real,
# silent-data-corruption guards in l2c_ctrl.v -- `hit_wr_blocked_c`
# ("Rule 2"), `victim_push_ready` in way_ok_c, and `!mshr_inst_valid` in
# miss_ok_c.  Every one of them can be deleted with `make tb-l2c` still
# reporting 65 PASS / 0 FAIL.  Measured detection, one mutant per guard:
#
#   config                     Rule 2   victim_push_ready   !mshr_inst_valid
#   3 seeds x 50,000 (old)      0/3          2/3                 1/3
#   5 seeds x 100,000 (now)     4/5          5/5                >=3/5
#
# The old default MISSED Rule 2 entirely, so do not shrink these back for
# runtime: the whole target costs ~12 s at the old size and well under a
# minute at this one.  If you add seeds, keep the existing three first so
# historical failures stay reproducible.
L2C_STRESS_OPS ?= 100000
L2C_STRESS_SEEDS ?= 0x13579bdf 0x2468ace0 0xdeadbeef 0xc0ffee11 0x5eed1234

.PHONY: tb-l2c
tb-l2c: $(L2C_BUILD)/Vtb_l2c
	@echo "Running l2c unit tb..."
	$(L2C_BUILD)/Vtb_l2c

# Mutation-testing build (scratch -Mdir so it never clobbers tb-l2c's).
# Used by the RED-verification harness described in the bypass-pipelining
# scenarios; not part of any gate.
.PHONY: tb-l2c-mut
tb-l2c-mut:
	@mkdir -p $(BUILD_DIR)/l2c_mut
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-SELRANGE \
		-Irtl/soc -Mdir $(BUILD_DIR)/l2c_mut --top-module tb_l2c \
		$(L2C_RTL) $(TB_DIR)/tb_l2c.cpp -CFLAGS "-std=c++17"
	-timeout 600 $(BUILD_DIR)/l2c_mut/Vtb_l2c

# ── ARRAY COLLISION RED-VERIFY (race audit 2026-09-18) ─────────────────────
# Both l2c arrays are read with a non-blocking RHS read, so SIMULATION gives
# read-first collision semantics for free.  SILICON DOES NOT PROMISE THAT:
# URAM288 has no write-mode attribute at all (UG573 ch.2), its collision
# behaviour is defined only by which physical port the tool put the read on,
# and UG901's RW_ADDR_COLLISION entry says Vivado defaults this RAM shape --
# simple dual port, independent read/write addresses -- to WRITE_FIRST "for
# best timing" while calling the collision output "unpredictable".
#
# That is a simulator-versus-silicon divergence, and it is invisible to every
# ordinary test precisely because Verilog satisfies it for free.  These
# targets are what make it visible: they rebuild the arrays so that a read
# colliding with a same-address write returns POISON (or the new data) and
# require the entire suite to pass ANYWAY.  Passing is the proof that no
# collided read is ever consumed; l2c's real protections are set_haz_c's
# accept-time refusal and skew_hazard_c's re-read, not the array semantics.
#
# Measured when introduced: ~2,600 data-array and ~5,900 tag-array collisions
# in the directed suite alone, and ~8,000 / ~19,500 per stress seed.  The
# sensitivity controls (poisoning 1.6%% of ALL reads instead of only colliding
# ones) fail immediately -- data reads with a DATA MISMATCH, tag reads with
# l2c_tags' own uniqueness assertion -- so these models demonstrably have
# teeth.
L2C_COLLISION_DEFS := L2C_ARRAY_COLLISION_HOSTILE L2C_ARRAY_COLLISION_WRITEFIRST

.PHONY: tb-l2c-collision
tb-l2c-collision:
	@set -e; for d in $(L2C_COLLISION_DEFS); do \
		echo "=== l2c array-collision build: $$d ==="; \
		mkdir -p $(L2C_BUILD)_coll_$$d; \
		$(VERILATOR) --cc --exe --build --assert \
			--x-assign fast --x-initial fast -O3 \
			-D$$d \
			-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
			-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
			-Wno-WIDTHEXPAND -Wno-SELRANGE \
			-Irtl/soc -Mdir $(L2C_BUILD)_coll_$$d --top-module tb_l2c \
			$(L2C_RTL) $(TB_DIR)/tb_l2c.cpp -CFLAGS "-std=c++17" >/dev/null; \
		$(L2C_BUILD)_coll_$$d/Vtb_l2c; \
		for seed in $(L2C_STRESS_SEEDS); do \
			echo "  stress seed=$$seed ($$d)"; \
			L2C_SKIP_DIRECTED=1 L2C_SEED=$$seed L2C_RAND_OPS=$(L2C_STRESS_OPS) \
				$(L2C_BUILD)_coll_$$d/Vtb_l2c; \
		done; \
	done
	@echo "PASS: no collided array read is consumed under either hostile model"

.PHONY: tb-l2c-stress
tb-l2c-stress: $(L2C_BUILD)/Vtb_l2c
	@set -e; for seed in $(L2C_STRESS_SEEDS); do \
		echo "Running l2c randomized stress seed=$$seed ops=$(L2C_STRESS_OPS)..."; \
		L2C_SKIP_DIRECTED=1 L2C_SEED=$$seed L2C_RAND_OPS=$(L2C_STRESS_OPS) \
			$(L2C_BUILD)/Vtb_l2c; \
	done

$(L2C_BUILD)/Vtb_l2c: $(L2C_RTL) $(TB_DIR)/tb_l2c.cpp
	@mkdir -p $(L2C_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-SELRANGE \
		-Irtl/soc \
		-Mdir $(L2C_BUILD) \
		--top-module tb_l2c \
		$(L2C_RTL) \
		$(TB_DIR)/tb_l2c.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# l2c_bypass depth sweep (docs/l2c_perf.md S14).
#
# One Verilated build per BYPASS_SLOTS value against the same tb_l2c.cpp
# harness, so the depth-vs-throughput curve behind the shipped default is
# reproducible without editing RTL.  BYPASS_SLOTS=1 is the reference point:
# it reproduces the pre-2026-08-20 serialized engine (one DDR round trip
# per 16 B beat).  Run the perf suite, not the directed suite:
#
#   make tb-l2c-bypdepth              # correctness at every depth
#   L2C_PERF=1 L2C_SKIP_DIRECTED=1 L2C_RAND_OPS=0 \
#       build/l2c_byp<N>/Vtb_l2c      # the perf table at depth N
# ──────────────────────────────────────────────────────────────────────────────
L2C_BYP_DEPTHS ?= 1 2 4 8 16 32

.PHONY: tb-l2c-bypdepth
tb-l2c-bypdepth: $(foreach d,$(L2C_BYP_DEPTHS),tb-l2c-bypdepth-$(d))

define L2C_BYPDEPTH_RULE
.PHONY: tb-l2c-bypdepth-$(1)
tb-l2c-bypdepth-$(1): $$(BUILD_DIR)/l2c_byp$(1)/Vtb_l2c
	@echo "Running l2c unit tb (BYPASS_SLOTS=$(1))..."
	$$(BUILD_DIR)/l2c_byp$(1)/Vtb_l2c

$$(BUILD_DIR)/l2c_byp$(1)/Vtb_l2c: $$(L2C_RTL) $$(TB_DIR)/tb_l2c.cpp
	@mkdir -p $$(BUILD_DIR)/l2c_byp$(1)
	$$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-SELRANGE \
		-Irtl/soc \
		-GBYPASS_SLOTS=$(1) \
		-Mdir $$(BUILD_DIR)/l2c_byp$(1) \
		--top-module tb_l2c \
		$$(L2C_RTL) \
		$$(TB_DIR)/tb_l2c.cpp \
		-CFLAGS "-std=c++17 -DL2C_BYPASS_SLOTS_BUILD=$(1)"
endef
$(foreach d,$(L2C_BYP_DEPTHS),$(eval $(call L2C_BYPDEPTH_RULE,$(d))))

.PHONY: tb-l2c-bypass-all
tb-l2c-bypass-all: $(L2C_BYPASS_BUILD)/Vtb_l2c_bypass
	@echo "Running l2c L2_BYPASS_ALL=1 pass-through equivalence tb..."
	$(L2C_BYPASS_BUILD)/Vtb_l2c_bypass

$(L2C_BYPASS_BUILD)/Vtb_l2c_bypass: $(L2C_RTL) $(TB_DIR)/tb_l2c.cpp
	@mkdir -p $(L2C_BYPASS_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-SELRANGE \
		-Irtl/soc \
		-GL2_BYPASS_ALL=1 \
		--prefix Vtb_l2c_bypass \
		-Mdir $(L2C_BYPASS_BUILD) \
		--top-module tb_l2c \
		$(L2C_RTL) \
		$(TB_DIR)/tb_l2c.cpp \
		-CFLAGS "-std=c++17 -DBYPASS_ALL_BUILD"

# ──────────────────────────────────────────────────────────────────────────────
# l2c CHAIN integration testbench (T13) -- the REAL chain:
#   l2c -> axi_async_bridge -> axi_ddr4_mig_bridge -> sim_mig_backend
# across two clock domains (core_clk/mig_clk), via tb/tb_l2c_chain.v.
# This is what actually lands behind fpga_top_ddr.vh's `L2C_ENABLE` gate
# (docs/l2c_spec.md, rtl/soc/fpga_top_ddr.vh) -- unlike tb-l2c (which
# drives l2c's m_axi against a software memory model), this tb proves
# l2c's traffic survives the real CDC bridge + MIG contract shim +
# behavioural memory end to end.  See tb_l2c_chain.cpp's header for the
# per-scenario writeup.
#
# Two builds share tb_l2c_chain.cpp:
#   tb-l2c-chain      -- CHAIN_L2C_ENABLE=1 (l2c really in the chain),
#                        STALL_ENABLE=1 (mig-side command backpressure,
#                        rtl/board/sim_mig_backend.v).  Full scenario set
#                        incl. >=5000-op randomized traffic, concurrent-
#                        fill-overlap timing, and core-side reset-mid-
#                        traffic recovery.
#   tb-l2c-chain-off  -- CHAIN_L2C_ENABLE=0: l2c is NOT ELABORATED AT ALL
#                        (mirrors fpga_top_ddr.vh's L2C_ENABLE-undefined
#                        wiring exactly, not merely L2_BYPASS_ALL) --
#                        proves the harness/chain wiring itself is
#                        correct with l2c entirely absent ("L2C_ENABLE=
#                        off equivalence smoke" in the T13 brief).
# ──────────────────────────────────────────────────────────────────────────────
L2C_CHAIN_RTL := \
	$(RTL_DIR)/soc/l2c_pri8.v \
	$(RTL_DIR)/soc/l2c_reset.v \
	$(RTL_DIR)/soc/l2c_tags.v \
	$(RTL_DIR)/soc/l2c_data.v \
	$(RTL_DIR)/soc/l2c_victim_sel.v \
	$(RTL_DIR)/soc/l2c_mshr.v \
	$(RTL_DIR)/soc/l2c_victim.v \
	$(RTL_DIR)/soc/l2c_bypass.v \
	$(RTL_DIR)/soc/l2c_ctrl.v \
	$(RTL_DIR)/soc/l2c.v \
	$(RTL_DIR)/board/async_fifo.v \
	$(RTL_DIR)/board/pulse_cdc.v \
	$(RTL_DIR)/soc/axi_bridge_stale_sink.v \
	$(RTL_DIR)/soc/axi_bridge_w_pad.v \
	$(RTL_DIR)/soc/axi_async_bridge.v \
	$(RTL_DIR)/board/axi_ddr4_mig_bridge.v \
	$(RTL_DIR)/board/sim_mig_backend.v \
	$(TB_DIR)/tb_l2c_chain.v
L2C_CHAIN_BUILD     := $(BUILD_DIR)/l2c_chain
L2C_CHAIN_OFF_BUILD := $(BUILD_DIR)/l2c_chain_off

.PHONY: tb-l2c-chain
tb-l2c-chain: $(L2C_CHAIN_BUILD)/Vtb_l2c_chain
	@echo "Running l2c chain integration tb (CHAIN_L2C_ENABLE=1)..."
	$(L2C_CHAIN_BUILD)/Vtb_l2c_chain

$(L2C_CHAIN_BUILD)/Vtb_l2c_chain: $(L2C_CHAIN_RTL) $(TB_DIR)/tb_l2c_chain.cpp
	@mkdir -p $(L2C_CHAIN_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-SELRANGE \
		-Irtl/soc -Irtl/board \
		-GSTALL_ENABLE=1 \
		-Mdir $(L2C_CHAIN_BUILD) \
		--top-module tb_l2c_chain \
		$(L2C_CHAIN_RTL) \
		$(TB_DIR)/tb_l2c_chain.cpp \
		-CFLAGS "-std=c++17"

.PHONY: tb-l2c-chain-off
tb-l2c-chain-off: $(L2C_CHAIN_OFF_BUILD)/Vtb_l2c_chain
	@echo "Running l2c chain integration tb (CHAIN_L2C_ENABLE=0, equivalence smoke)..."
	$(L2C_CHAIN_OFF_BUILD)/Vtb_l2c_chain

$(L2C_CHAIN_OFF_BUILD)/Vtb_l2c_chain: $(L2C_CHAIN_RTL) $(TB_DIR)/tb_l2c_chain.cpp
	@mkdir -p $(L2C_CHAIN_OFF_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-SELRANGE \
		-Irtl/soc -Irtl/board \
		-GCHAIN_L2C_ENABLE=0 -GSTALL_ENABLE=1 \
		-Mdir $(L2C_CHAIN_OFF_BUILD) \
		--top-module tb_l2c_chain \
		$(L2C_CHAIN_RTL) \
		$(TB_DIR)/tb_l2c_chain.cpp \
		-CFLAGS "-std=c++17 -DCHAIN_OFF_BUILD"

# ──────────────────────────────────────────────────────────────────────────────
# l2c STREAMING-WRITE measurement harness -- the real boot write path:
#   32-bit master -> axi_narrow_to_wide -> [l2c] -> axi_async_bridge ->
#   axi_ddr4_mig_bridge -> sim_mig_backend
# via tb/tb_l2c_wstream.v, which instantiates tb_l2c_chain verbatim and
# only adds the narrow front end.  Reports cycles per 32-bit word and the
# 256 MiB @ 100 MHz extrapolation for narrow AWLEN 0/15/63/255.
#
# This is the harness that produced the read-allocate-on-write-miss
# numbers in commit 2afe6d1 and the full-line-write-no-fill numbers that
# followed; it lives in the tree so those are re-derivable rather than
# quoted from a commit message.  It is a MEASUREMENT target, not a
# correctness gate (it only asserts OKAY responses and no wedge) --
# tb-l2c / tb-l2c-chain own cache correctness.
#
#   make tb-l2c-wstream      -- l2c in the chain
#   make tb-l2c-wstream-off  -- l2c not elaborated at all (the ceiling)
#   WSTREAM_WORDS=<n>        -- 32-bit words per data point (default 16384)
# ──────────────────────────────────────────────────────────────────────────────
L2C_WSTREAM_RTL := \
	$(L2C_CHAIN_RTL) \
	$(RTL_DIR)/soc/axi_narrow_to_wide.v \
	$(TB_DIR)/tb_l2c_wstream.v
L2C_WSTREAM_BUILD     := $(BUILD_DIR)/l2c_wstream
L2C_WSTREAM_OFF_BUILD := $(BUILD_DIR)/l2c_wstream_off

define L2C_WSTREAM_BUILD_RULE
	@mkdir -p $(1)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-SELRANGE \
		-Irtl/soc -Irtl/board \
		$(2) \
		-Mdir $(1) \
		--top-module tb_l2c_wstream \
		$(L2C_WSTREAM_RTL) \
		$(TB_DIR)/tb_l2c_wstream.cpp \
		-CFLAGS "-std=c++17 $(3)"
endef

.PHONY: tb-l2c-wstream
tb-l2c-wstream: $(L2C_WSTREAM_BUILD)/Vtb_l2c_wstream
	@echo "Measuring l2c streaming-write throughput (l2c IN the chain)..."
	$(L2C_WSTREAM_BUILD)/Vtb_l2c_wstream

$(L2C_WSTREAM_BUILD)/Vtb_l2c_wstream: $(L2C_WSTREAM_RTL) $(TB_DIR)/tb_l2c_wstream.cpp
	$(call L2C_WSTREAM_BUILD_RULE,$(L2C_WSTREAM_BUILD),-GWS_L2C_ENABLE=1,)

.PHONY: tb-l2c-wstream-off
tb-l2c-wstream-off: $(L2C_WSTREAM_OFF_BUILD)/Vtb_l2c_wstream
	@echo "Measuring l2c streaming-write throughput (l2c REMOVED from the chain)..."
	$(L2C_WSTREAM_OFF_BUILD)/Vtb_l2c_wstream

$(L2C_WSTREAM_OFF_BUILD)/Vtb_l2c_wstream: $(L2C_WSTREAM_RTL) $(TB_DIR)/tb_l2c_wstream.cpp
	$(call L2C_WSTREAM_BUILD_RULE,$(L2C_WSTREAM_OFF_BUILD),-GWS_L2C_ENABLE=0,-DWSTREAM_L2C_OFF)

# ──────────────────────────────────────────────────────────────────────────────
# l2c SCATTERED / PARTIAL-LINE write measurement harness -- same chain and
# same tb/tb_l2c_wstream.v top as tb-l2c-wstream, different BFM
# (tb/tb_l2c_sctr.cpp).  tb-l2c-wstream measures the case commit 00f079c
# optimised (writes covering whole 64 B lines); this measures the case it
# does not catch -- a write touching only part of a line, which is what a
# SECTORED dirty/valid scheme would change.
#
# Reports cycles per useful 32-bit word AND, more to the point, DDR read
# and write BEATS per touched line, counted on l2c's own 128-bit master
# port (one beat = one 16 B quadrant = one 68040 L1D line).  The working
# set is shaped so that in steady state every line both misses and evicts
# a dirty victim, so both the fill and the writeback show up.
#
#   make tb-l2c-sctr       -- l2c in the chain
#   make tb-l2c-sctr-off   -- l2c not elaborated at all (the reference)
#   SCTR_SETS=<n>          -- distinct sets per data point (default 128)
# ──────────────────────────────────────────────────────────────────────────────
L2C_SCTR_BUILD     := $(BUILD_DIR)/l2c_sctr
L2C_SCTR_OFF_BUILD := $(BUILD_DIR)/l2c_sctr_off

define L2C_SCTR_BUILD_RULE
	@mkdir -p $(1)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-SELRANGE \
		-Irtl/soc -Irtl/board \
		$(2) \
		-Mdir $(1) \
		--top-module tb_l2c_wstream \
		$(L2C_WSTREAM_RTL) \
		$(TB_DIR)/tb_l2c_sctr.cpp \
		-CFLAGS "-std=c++17 $(3)"
endef

.PHONY: tb-l2c-sctr
tb-l2c-sctr: $(L2C_SCTR_BUILD)/Vtb_l2c_wstream
	@echo "Measuring l2c scattered/partial-line write traffic (l2c IN the chain)..."
	$(L2C_SCTR_BUILD)/Vtb_l2c_wstream

$(L2C_SCTR_BUILD)/Vtb_l2c_wstream: $(L2C_WSTREAM_RTL) $(TB_DIR)/tb_l2c_sctr.cpp
	$(call L2C_SCTR_BUILD_RULE,$(L2C_SCTR_BUILD),-GWS_L2C_ENABLE=1,)

.PHONY: tb-l2c-sctr-off
tb-l2c-sctr-off: $(L2C_SCTR_OFF_BUILD)/Vtb_l2c_wstream
	@echo "Measuring scattered/partial-line write traffic (l2c REMOVED from the chain)..."
	$(L2C_SCTR_OFF_BUILD)/Vtb_l2c_wstream

$(L2C_SCTR_OFF_BUILD)/Vtb_l2c_wstream: $(L2C_WSTREAM_RTL) $(TB_DIR)/tb_l2c_sctr.cpp
	$(call L2C_SCTR_BUILD_RULE,$(L2C_SCTR_OFF_BUILD),-GWS_L2C_ENABLE=0,-DSCTR_L2C_OFF)

# ──────────────────────────────────────────────────────────────────────────────
# VRAM-in-DDR integration testbench (T14, reshaped by T16 "decode-level
# VRAM lane") -- one seam further out than tb-l2c-chain: axi_xbar
# (VRAM_IN_DDR; S3 is a genuine slave again, T16 revert of T14's S3->S0
# decode fold) -> S3 -> [address-translate] -> axi_vram_priority_mux3
# (3-way: l2c-path [when CHAIN_L2C_ENABLE], S3-lane, scanout) ->
# axi_async_bridge -> axi_ddr4_mig_bridge -> sim_mig_backend, via
# tb/tb_vram_ddr_chain.v.  See tb_vram_ddr_chain.cpp's header for the
# per-scenario writeup.  Both builds compile the RTL with -DVRAM_IN_DDR
# (required for axi_xbar.v's S3 handling + scanout_ddr_reader/mux to be
# elaborated at all); the OFF-path equivalence itself is proven by
# preprocessor-diff, not by a build variant here (see docs/l2c_spec.md-
# style precedent: L2C_ENABLE off-equivalence has its OWN dedicated
# tb-l2c-chain-off; VRAM_IN_DDR's off-equivalence is instead proven
# structurally -- every touched file's `ifndef VRAM_IN_DDR` branch is a
# byte-for-byte-identical superset of the pre-T14 file, verified during
# development; the existing VRAM_IN_DDR-undefined video tb suite --
# tb-framebuffer-pixel, tb-dafb-scanout, tb-vram-scaler-firstlight,
# tb-vram, tb-vram-cpu-write -- re-run unmodified is the regression gate
# for that path).
#
# tb-vram-ddr-chain          -- CHAIN_L2C_ENABLE=1 (l2c genuinely in the
#                                RAM/ROM/FB path; VRAM-aperture CPU
#                                traffic never reaches it -- S3 lane).
# tb-vram-ddr-chain-nol2c    -- CHAIN_L2C_ENABLE=0 (xbar S0 -> mux
#                                directly, matching VRAM_IN_DDR without
#                                L2C_ENABLE).
# ──────────────────────────────────────────────────────────────────────────────
VRAM_DDR_CHAIN_RTL := \
	$(RTL_DIR)/soc/l2c_pri8.v \
	$(RTL_DIR)/soc/l2c_reset.v \
	$(RTL_DIR)/soc/l2c_tags.v \
	$(RTL_DIR)/soc/l2c_data.v \
	$(RTL_DIR)/soc/l2c_victim_sel.v \
	$(RTL_DIR)/soc/l2c_mshr.v \
	$(RTL_DIR)/soc/l2c_victim.v \
	$(RTL_DIR)/soc/l2c_bypass.v \
	$(RTL_DIR)/soc/l2c_ctrl.v \
	$(RTL_DIR)/soc/l2c.v \
	$(RTL_DIR)/board/async_fifo.v \
	$(RTL_DIR)/board/pulse_cdc.v \
	$(RTL_DIR)/soc/axi_bridge_stale_sink.v \
	$(RTL_DIR)/soc/axi_bridge_w_pad.v \
	$(RTL_DIR)/soc/axi_async_bridge.v \
	$(RTL_DIR)/board/axi_ddr4_mig_bridge.v \
	$(RTL_DIR)/board/sim_mig_backend.v \
	$(RTL_DIR)/soc/axi_xbar.v \
	$(RTL_DIR)/soc/scanout_ddr_reader.v \
	$(RTL_DIR)/soc/scanout_line_fetch.v \
	$(RTL_DIR)/soc/axi_vram_priority_mux3.v \
	$(RTL_DIR)/soc/axi_vram_smoke_mux.v \
	$(RTL_DIR)/board/video_phy/vram_smoke.v \
	$(TB_DIR)/tb_vram_ddr_chain.v
VRAM_DDR_CHAIN_BUILD       := $(BUILD_DIR)/vram_ddr_chain
VRAM_DDR_CHAIN_NOL2C_BUILD := $(BUILD_DIR)/vram_ddr_chain_nol2c
VRAM_MUX3_DIRECT_BUILD     := $(BUILD_DIR)/vram_mux3_direct
VIDEO_SMOKE_DDR_BUILD      := $(BUILD_DIR)/video_smoke_ddr
VIDEO_SMOKE_DDR_NEG_BUILD  := $(BUILD_DIR)/video_smoke_ddr_negctl
# Must stay in sync with tb_video_smoke_ddr.cpp's SMOKE_* constants.
VIDEO_SMOKE_DDR_GEOM := -GCHAIN_VIDEO_SMOKE=1 -GCHAIN_SMOKE_W=16 \
	-GCHAIN_SMOKE_H=64 -GCHAIN_SMOKE_ROW_BAND_LOG2=3

.PHONY: tb-axi-vram-mux3
tb-axi-vram-mux3: $(VRAM_MUX3_DIRECT_BUILD)/Vtb_axi_vram_priority_mux3
	@echo "Running direct VRAM/MIG arbitration and ownership tb..."
	$(VRAM_MUX3_DIRECT_BUILD)/Vtb_axi_vram_priority_mux3

$(VRAM_MUX3_DIRECT_BUILD)/Vtb_axi_vram_priority_mux3: \
		$(RTL_DIR)/soc/axi_vram_priority_mux3.v $(TB_DIR)/tb_vram_ddr_chain.v
	@mkdir -p $(VRAM_MUX3_DIRECT_BUILD)
	$(VERILATOR) --binary --timing --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-SELRANGE -Wno-BLKSEQ \
		-Irtl/soc -DAXI_VRAM_MUX3_DIRECT_TB \
		-Mdir $(VRAM_MUX3_DIRECT_BUILD) \
		--top-module tb_axi_vram_priority_mux3 \
		$(RTL_DIR)/soc/axi_vram_priority_mux3.v $(TB_DIR)/tb_vram_ddr_chain.v

.PHONY: tb-vram-ddr-chain
tb-vram-ddr-chain: $(VRAM_DDR_CHAIN_BUILD)/Vtb_vram_ddr_chain
	@echo "Running T14 VRAM-in-DDR chain integration tb (CHAIN_L2C_ENABLE=1)..."
	$(VRAM_DDR_CHAIN_BUILD)/Vtb_vram_ddr_chain

$(VRAM_DDR_CHAIN_BUILD)/Vtb_vram_ddr_chain: $(VRAM_DDR_CHAIN_RTL) $(TB_DIR)/tb_vram_ddr_chain.cpp
	@mkdir -p $(VRAM_DDR_CHAIN_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-SELRANGE \
		-Irtl/soc -Irtl/board \
		-DVRAM_IN_DDR \
		-GCHAIN_L2C_ENABLE=1 -GSTALL_ENABLE=1 \
		-Mdir $(VRAM_DDR_CHAIN_BUILD) \
		--top-module tb_vram_ddr_chain \
		$(VRAM_DDR_CHAIN_RTL) \
		$(TB_DIR)/tb_vram_ddr_chain.cpp \
		-CFLAGS "-std=c++17"

.PHONY: tb-vram-ddr-chain-nol2c
tb-vram-ddr-chain-nol2c: $(VRAM_DDR_CHAIN_NOL2C_BUILD)/Vtb_vram_ddr_chain
	@echo "Running T14 VRAM-in-DDR chain integration tb (CHAIN_L2C_ENABLE=0)..."
	$(VRAM_DDR_CHAIN_NOL2C_BUILD)/Vtb_vram_ddr_chain

$(VRAM_DDR_CHAIN_NOL2C_BUILD)/Vtb_vram_ddr_chain: $(VRAM_DDR_CHAIN_RTL) $(TB_DIR)/tb_vram_ddr_chain.cpp
	@mkdir -p $(VRAM_DDR_CHAIN_NOL2C_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-SELRANGE \
		-Irtl/soc -Irtl/board \
		-DVRAM_IN_DDR \
		-GCHAIN_L2C_ENABLE=0 -GSTALL_ENABLE=1 \
		-Mdir $(VRAM_DDR_CHAIN_NOL2C_BUILD) \
		--top-module tb_vram_ddr_chain \
		$(VRAM_DDR_CHAIN_RTL) \
		$(TB_DIR)/tb_vram_ddr_chain.cpp \
		-CFLAGS "-std=c++17 -DCHAIN_OFF_BUILD"

# ──────────────────────────────────────────────────────────────────────────────
# tb-video-smoke-ddr -- the CPU-less video-test rig.  Same tb_vram_ddr_chain
# wrapper, built with CHAIN_VIDEO_SMOKE=1 so a real vram_smoke instance is
# muxed onto the S3 lane by the production axi_vram_smoke_mux, exactly as
# fpga_top_ddr.vh does it.  Proves smoke writes -> DDR carveout -> scanout
# reads produce the expected pixels AND that smoke sits on the correct side
# of axi_xbar.v's S3 byte-swap.  See tb_video_smoke_ddr.cpp's header.
#
# tb-video-smoke-ddr-negctl is the NEGATIVE CONTROL: the identical test
# built with CHAIN_SMOKE_BROKEN_SWAP=1, which deliberately puts smoke on
# the wrong side of the swap.  The checker must FAIL there; the recipe
# passes only when it does.  Without this, "the pattern matched" would only
# prove the checker can see a MISSING pattern, not a WRONG one.
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: tb-video-smoke-ddr
tb-video-smoke-ddr: $(VIDEO_SMOKE_DDR_BUILD)/Vtb_vram_ddr_chain
	@echo "Running CPU-less video-smoke rig tb (VIDEO_SMOKE=1 + VRAM_IN_DDR)..."
	$(VIDEO_SMOKE_DDR_BUILD)/Vtb_vram_ddr_chain

$(VIDEO_SMOKE_DDR_BUILD)/Vtb_vram_ddr_chain: $(VRAM_DDR_CHAIN_RTL) $(TB_DIR)/tb_video_smoke_ddr.cpp
	@mkdir -p $(VIDEO_SMOKE_DDR_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-SELRANGE -Wno-BLKSEQ -Wno-CMPCONST -Wno-UNSIGNED \
		-Irtl/soc -Irtl/board -Irtl/board/video_phy \
		-DVRAM_IN_DDR \
		-GCHAIN_L2C_ENABLE=1 -GSTALL_ENABLE=1 $(VIDEO_SMOKE_DDR_GEOM) \
		-Mdir $(VIDEO_SMOKE_DDR_BUILD) \
		--top-module tb_vram_ddr_chain \
		$(VRAM_DDR_CHAIN_RTL) \
		$(TB_DIR)/tb_video_smoke_ddr.cpp \
		-CFLAGS "-std=c++17"

.PHONY: tb-video-smoke-ddr-negctl
tb-video-smoke-ddr-negctl: $(VIDEO_SMOKE_DDR_NEG_BUILD)/Vtb_vram_ddr_chain
	@echo "Running CPU-less video-smoke rig NEGATIVE CONTROL (wrong byte-swap side)..."
	@SMOKE_DDR_EXPECT_FAIL=1 $(VIDEO_SMOKE_DDR_NEG_BUILD)/Vtb_vram_ddr_chain

$(VIDEO_SMOKE_DDR_NEG_BUILD)/Vtb_vram_ddr_chain: $(VRAM_DDR_CHAIN_RTL) $(TB_DIR)/tb_video_smoke_ddr.cpp
	@mkdir -p $(VIDEO_SMOKE_DDR_NEG_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-SELRANGE -Wno-BLKSEQ -Wno-CMPCONST -Wno-UNSIGNED \
		-Irtl/soc -Irtl/board -Irtl/board/video_phy \
		-DVRAM_IN_DDR \
		-GCHAIN_L2C_ENABLE=1 -GSTALL_ENABLE=1 $(VIDEO_SMOKE_DDR_GEOM) \
		-GCHAIN_SMOKE_BROKEN_SWAP=1 \
		-Mdir $(VIDEO_SMOKE_DDR_NEG_BUILD) \
		--top-module tb_vram_ddr_chain \
		$(VRAM_DDR_CHAIN_RTL) \
		$(TB_DIR)/tb_video_smoke_ddr.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# T14 follow-up: the REAL fb_reader.v (not just a C++ BFM) driven through
# the VRAM-in-DDR chain -- tb-vram-ddr-chain above only drives
# scanout_ddr_reader.v's port directly; this proves fb_reader.v's own
# pclk<->core_clk CDC + credit machinery (video_top.v's actual consumer)
# works against the DDR-backed reader. See tb_fb_reader_ddr_chain.v's
# header for the deliberately-minimal scope (skips scanout_placement_
# sync/linebuf_scanout -- scan-timing generation, not this seam).
# ──────────────────────────────────────────────────────────────────────────────
FB_READER_DDR_CHAIN_RTL := \
	$(RTL_DIR)/board/video_phy/fb_reader.v \
	$(RTL_DIR)/board/async_fifo.v \
	$(RTL_DIR)/board/pulse_cdc.v \
	$(RTL_DIR)/soc/axi_bridge_stale_sink.v \
	$(RTL_DIR)/soc/axi_bridge_w_pad.v \
	$(RTL_DIR)/soc/axi_async_bridge.v \
	$(RTL_DIR)/board/axi_ddr4_mig_bridge.v \
	$(RTL_DIR)/board/sim_mig_backend.v \
	$(RTL_DIR)/soc/scanout_ddr_reader.v \
	$(RTL_DIR)/soc/scanout_line_fetch.v \
	$(RTL_DIR)/soc/axi_vram_priority_mux3.v \
	$(TB_DIR)/tb_fb_reader_ddr_chain.v
FB_READER_DDR_CHAIN_BUILD := $(BUILD_DIR)/fb_reader_ddr_chain

.PHONY: tb-fb-reader-ddr-chain
tb-fb-reader-ddr-chain: $(FB_READER_DDR_CHAIN_BUILD)/Vtb_fb_reader_ddr_chain
	@echo "Running T14 fb_reader-through-DDR-chain integration tb..."
	$(FB_READER_DDR_CHAIN_BUILD)/Vtb_fb_reader_ddr_chain

$(FB_READER_DDR_CHAIN_BUILD)/Vtb_fb_reader_ddr_chain: $(FB_READER_DDR_CHAIN_RTL) $(TB_DIR)/tb_fb_reader_ddr_chain.cpp
	@mkdir -p $(FB_READER_DDR_CHAIN_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-SELRANGE \
		-Irtl/soc -Irtl/board -Irtl/board/video_phy \
		-Mdir $(FB_READER_DDR_CHAIN_BUILD) \
		--top-module tb_fb_reader_ddr_chain \
		$(FB_READER_DDR_CHAIN_RTL) \
		$(TB_DIR)/tb_fb_reader_ddr_chain.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# tb-scanout-ddr-frames -- MULTI-FRAME scanout over the REAL fetch chain.
#
# Closes the seam no other target covers.  tb-scanout-frames runs the scanner
# against a C++ fetch-port model; tb-fb-reader-ddr-chain runs fb_reader +
# scanout_ddr_reader against a C++ request stream and explicitly "skips
# scanout_placement_sync/linebuf_scanout"; tb-video-smoke-ddr reads the DDR
# port back directly with no scanner at all.  Nothing put the scanner's
# per-frame resync walk in front of the line-granular DDR ring -- which is
# exactly where a whole-frame source-origin slip lives.  See
# tb/tb_scanout_ddr_frames.v's header.
#
# Runs several 1920x1080 frames at production clock RATIOS, so it is slow
# (minutes, not seconds) and deliberately not in the fast inner loop.
# SCAN_FRAMES=<n> and SCAN_ONLY=<scenario> narrow it for bisection.
# ──────────────────────────────────────────────────────────────────────────────
SCANOUT_DDR_FRAMES_RTL := \
	$(RTL_DIR)/board/video_phy/vtg.v \
	$(RTL_DIR)/board/video_phy/mode_admit.v \
	$(RTL_DIR)/board/video_phy/scanout_fetch.v \
	$(RTL_DIR)/board/video_phy/place_plan.v \
	$(RTL_DIR)/board/video_phy/pixel_unpack.v \
	$(RTL_DIR)/board/video_phy/clut.v \
	$(RTL_DIR)/board/video_phy/upscale.v \
	$(RTL_DIR)/board/video_phy/compositor.v \
	$(RTL_DIR)/board/video_phy/scanout_display.v \
	$(RTL_DIR)/board/video_phy/linebuf_scanout.v \
	$(RTL_DIR)/board/video_phy/fb_reader.v \
	$(RTL_DIR)/board/async_fifo.v \
	$(RTL_DIR)/board/pulse_cdc.v \
	$(RTL_DIR)/soc/axi_bridge_stale_sink.v \
	$(RTL_DIR)/soc/axi_bridge_w_pad.v \
	$(RTL_DIR)/soc/axi_async_bridge.v \
	$(RTL_DIR)/board/axi_ddr4_mig_bridge.v \
	$(RTL_DIR)/board/sim_mig_backend.v \
	$(RTL_DIR)/soc/scanout_ddr_reader.v \
	$(RTL_DIR)/soc/scanout_line_fetch.v \
	$(RTL_DIR)/soc/axi_vram_priority_mux3.v \
	$(TB_DIR)/tb_scanout_ddr_frames.v
# SCANOUT_FRAMES_TAG / SCANOUT_FRAMES_G let the same source be elaborated at
# several arbitration + DDR-latency points WITHOUT clobbering each other's
# build dir (a -G change alone does not re-trigger a rebuild, so the tag is
# what keeps the sweep honest).  Example:
#   make tb-scanout-ddr-frames SCANOUT_FRAMES_TAG=_b6_lat200 \
#        SCANOUT_FRAMES_G="-GMUX_MAX_BULK_AHEAD=6 -GDDR_READ_LATENCY_BASE=200 -GDDR_READ_LATENCY_JITTER=64"
SCANOUT_FRAMES_TAG ?=
SCANOUT_FRAMES_G   ?=
SCANOUT_DDR_FRAMES_BUILD := $(BUILD_DIR)/scanout_ddr_frames$(SCANOUT_FRAMES_TAG)

.PHONY: tb-scanout-ddr-frames
tb-scanout-ddr-frames: $(SCANOUT_DDR_FRAMES_BUILD)/Vtb_scanout_ddr_frames
	@echo "Running full-chain multi-frame scanout tb (scanner -> fb_reader -> DDR ring)..."
	$(SCANOUT_DDR_FRAMES_BUILD)/Vtb_scanout_ddr_frames

# ──────────────────────────────────────────────────────────────────────────────
# tb-scanout-first-pixel -- the LINE-START gate.
#
# Same rig as tb-scanout-ddr-frames, but elaborated at the PRODUCTION scanner
# bound (SRC_W=1152 / SRC_H=1024 -- fpga_top_video.vh's FB_W/FB_H, which the
# default 1024x768 elaboration does NOT match) and run at the worst case the
# hardware can present: a saturating CPU miss storm on the shared DDR read path
# (BULK=1) and a DDR round trip of 200 +/- 64 core clocks, 5x the figure every
# scanout margin in this tree is derived from.
#
# WHY IT EXISTS.  A scanout tb that counts lines and checks frame timing cannot
# see a LINE-START FETCH UNDERRUN: the frame still has the right number of
# lines, and only the first few pixels after the left border are wrong.  The
# checker in tb_scanout_ddr_frames.cpp already compares EVERY active pixel
# against an independent re-derivation of the source and reports `first_bad=
# (line, col)`, so it does see it -- but nothing ran it at the production bound
# under load, which is where the underrun hypothesis says it lives.
#
# tb-scanout-first-pixel-negctl PROVES that sensitivity instead of assuming it:
# SCANOUT_INJECT_LINESTART_UNDERRUN makes scanout_line_fetch.v serve the ring
# while the line is still in flight -- literally "the line-start fetch has not
# returned and the scanner consumes stale ring contents" -- and the gate must
# FAIL, naming (line 0, col 0).  Keep both halves.
# ──────────────────────────────────────────────────────────────────────────────
SCANOUT_FIRSTPX_BUILD := $(BUILD_DIR)/scanout_ddr_frames_first_pixel
SCANOUT_FIRSTPX_NEG_BUILD := $(BUILD_DIR)/scanout_ddr_frames_first_pixel_neg
SCANOUT_FIRSTPX_G := -GSRC_W=1152 -GSRC_H=1024 \
	-GDDR_READ_LATENCY_BASE=200 -GDDR_READ_LATENCY_JITTER=64

.PHONY: tb-scanout-first-pixel
tb-scanout-first-pixel: $(SCANOUT_FIRSTPX_BUILD)/Vtb_scanout_ddr_frames
	@echo "Running line-start (first-pixel) scanout gate: production scanner bound, CPU miss storm, 200+/-64 cycle DDR round trip..."
	BULK=1 $(SCANOUT_FIRSTPX_BUILD)/Vtb_scanout_ddr_frames

$(SCANOUT_FIRSTPX_BUILD)/Vtb_scanout_ddr_frames: $(SCANOUT_DDR_FRAMES_RTL) $(TB_DIR)/tb_scanout_ddr_frames.cpp
	@$(MAKE) $(BUILD_DIR)/scanout_ddr_frames_first_pixel/Vtb_scanout_ddr_frames \
		SCANOUT_FRAMES_TAG=_first_pixel \
		SCANOUT_FRAMES_G="$(SCANOUT_FIRSTPX_G)"

# NEGATIVE CONTROL: the same gate against an RTL fault injection that IS a
# line-start underrun.  Expected to FAIL; the target inverts the exit code so a
# GREEN run here is the failure ("the gate cannot see the thing it gates").
.PHONY: tb-scanout-first-pixel-negctl
tb-scanout-first-pixel-negctl: $(SCANOUT_FIRSTPX_NEG_BUILD)/Vtb_scanout_ddr_frames
	@echo "Running line-start scanout NEGATIVE CONTROL (injected line-start underrun)..."
	@if SCAN_FRAMES=2 BULK=0 SCAN_ONLY=24bpp-832x624-3_2 \
	      $(SCANOUT_FIRSTPX_NEG_BUILD)/Vtb_scanout_ddr_frames > /dev/null 2>&1; then \
	    echo "NEGATIVE CONTROL FAILED: the injected line-start underrun was NOT detected."; \
	    exit 1; \
	else \
	    echo "NEGATIVE CONTROL OK: the injected line-start underrun was detected."; \
	fi

$(SCANOUT_FIRSTPX_NEG_BUILD)/Vtb_scanout_ddr_frames: $(SCANOUT_DDR_FRAMES_RTL) $(TB_DIR)/tb_scanout_ddr_frames.cpp
	@$(MAKE) $(BUILD_DIR)/scanout_ddr_frames_first_pixel_neg/Vtb_scanout_ddr_frames \
		SCANOUT_FRAMES_TAG=_first_pixel_neg \
		SCANOUT_FRAMES_G="-GSRC_W=1152 -GSRC_H=1024 -DSCANOUT_INJECT_LINESTART_UNDERRUN"

$(SCANOUT_DDR_FRAMES_BUILD)/Vtb_scanout_ddr_frames: $(SCANOUT_DDR_FRAMES_RTL) $(TB_DIR)/tb_scanout_ddr_frames.cpp
	@mkdir -p $(SCANOUT_DDR_FRAMES_BUILD)
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY -Wno-PINMISSING \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND -Wno-CMPCONST -Wno-UNSIGNED -Wno-SELRANGE \
		-Wno-UNOPTTHREADS -Wno-BLKSEQ \
		-Irtl/soc -Irtl/board -Irtl/board/video_phy \
		-Mdir $(SCANOUT_DDR_FRAMES_BUILD) \
		--top-module tb_scanout_ddr_frames \
		$(SCANOUT_FRAMES_G) \
		$(SCANOUT_DDR_FRAMES_RTL) \
		$(TB_DIR)/tb_scanout_ddr_frames.cpp \
		-CFLAGS "-std=c++17"

# ──────────────────────────────────────────────────────────────────────────────
# Musashi reference ISS build
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: musashi musashi-ref musashi-smoke musashi-bus-smoke rom-boot-bus-smoke musashi-rom-boot
musashi:
	$(MAKE) -C $(TB_DIR)/models musashi

musashi-ref:
	$(MAKE) -C $(TB_DIR)/models musashi-ref

musashi-smoke:
	$(MAKE) -C $(TB_DIR)/models smoke

musashi-bus-smoke:
	$(MAKE) -C $(TB_DIR)/models bus-smoke

rom-boot-bus-smoke:
	$(MAKE) -C $(TB_DIR)/models rom-bus-smoke

musashi-rom-boot:
	$(MAKE) -C $(TB_DIR)/models musashi-rom-boot
	@mkdir -p $(ROMBOOT_OUTPUT_ROOT)
	$(TB_DIR)/models/musashi_rom_boot \
		--rom $(ROM) \
		--trace $(ROMBOOT_OUTPUT_ROOT)/musashi_rom_boot_trace.log \
		--periph-log $(ROMBOOT_OUTPUT_ROOT)/musashi_rom_boot_periph.log \
		--max-insts $(MUSASHI_ROM_BOOT_MAX) \
		$(MUSASHI_ROM_BOOT_ARGS)

MUSASHI_ROM_BOOT_MAX ?= 5000
MUSASHI_ROM_BOOT_ARGS ?=

ROM_FRONTIER_DIFF_ROOT ?= /dev/shm/m68k/rom_frontier_diff
ROM_FRONTIER_DIFF_STOP_PC ?= 0x408005b0
ROM_FRONTIER_DIFF_MAX_INSTS ?= 2048
ROM_FRONTIER_DIFF_LASTN_CYCLES ?= 256
ROM_FRONTIER_DIFF_PERIPH_LIMIT ?= 128
ROM_FRONTIER_DIFF_PERIPH_FILTER ?= VIA1,VIA2,ADB,VBL,RTC,PRAM,ASC,SCSI,SCC,DAFB,VRAM
ROM_FRONTIER_DIFF_RTL_STOP_ON_EXC ?= 2,4,11
ROM_FRONTIER_DIFF_RUN_NAME ?=
ROM_FRONTIER_DIFF_SAMPLE_EVERY ?= 0
ROM_FRONTIER_DIFF_MAME_TRACE ?=
ROM_FRONTIER_DIFF_MAME_PC_ONLY ?= 0

.PHONY: rom-frontier-diff
rom-frontier-diff:
	@mkdir -p $(ROM_FRONTIER_DIFF_ROOT)
	VERILATOR_THREADS=4 VERILATOR_JOBS=4 MAKEFLAGS='-j1' \
		python3 $(TOOLS_DIR)/rom_frontier_diff.py \
		--repo-root $(PROJ_ROOT) \
		--root $(ROM_FRONTIER_DIFF_ROOT) \
		$(if $(ROM_FRONTIER_DIFF_RUN_NAME),--run-name $(ROM_FRONTIER_DIFF_RUN_NAME),) \
		--stop-pc $(ROM_FRONTIER_DIFF_STOP_PC) \
		--max-insts $(ROM_FRONTIER_DIFF_MAX_INSTS) \
		--lastn-cycles $(ROM_FRONTIER_DIFF_LASTN_CYCLES) \
		--periph-limit $(ROM_FRONTIER_DIFF_PERIPH_LIMIT) \
		--periph-filter $(ROM_FRONTIER_DIFF_PERIPH_FILTER) \
		--rtl-stop-on-exc $(ROM_FRONTIER_DIFF_RTL_STOP_ON_EXC) \
		--sample-every $(ROM_FRONTIER_DIFF_SAMPLE_EVERY) \
		$(if $(ROM_FRONTIER_DIFF_MAME_TRACE),--mame-trace $(ROM_FRONTIER_DIFF_MAME_TRACE),) \
		$(if $(filter 1 true yes,$(ROM_FRONTIER_DIFF_MAME_PC_ONLY)),--mame-pc-only,)

# ──────────────────────────────────────────────────────────────────────────────
# Fuzzer (m68k-ooo vs Musashi co-simulation)
# ──────────────────────────────────────────────────────────────────────────────
# make fuzz N=100        — run 100 random seeds
# make fuzz-replay FILE=tb/fuzz_fails/seed_12345.bin
#                        — replay a saved failing case
# make fuzz-replay-seeds SEEDS_FILE=build/fuzz/repro_seeds.txt
#                        — replay a deterministic seed list
# N, FUZZ_N and the legacy spelling are all accepted.
N ?= $(FUZZ_N)
FUZZ_N ?= 500
FUZZ_DIR := $(TOOLS_DIR)/fuzz
FUZZ_WRITE_LOG_POLICY ?= auto
FUZZ_REPRO_SEEDS ?= $(BUILD_DIR)/fuzz/repro_seeds.txt

.PHONY: fuzz fuzz-deep fuzz-replay fuzz-replay-seeds
fuzz: sim musashi-ref
	$(MAKE) -C $(TB_DIR)/models musashi-run
	@mkdir -p $(BUILD_DIR)/fuzz
	python3 $(FUZZ_DIR)/fuzz.py \
		--n $(N) \
		--sim $(BUILD_DIR)/sim/Vmac_top \
		--musashi $(TB_DIR)/models/libmusashi_ref.a \
		--work $(BUILD_DIR)/fuzz \
		--as $(M68K_AS) --ld $(M68K_LD) --objcopy $(M68K_OBJCOPY) \
		--write-log-policy $(FUZZ_WRITE_LOG_POLICY) \
		--save-seed-file $(FUZZ_REPRO_SEEDS) \
		--save-fails $(TB_DIR)/fuzz_fails

# ──────────────────────────────────────────────────────────────────────
# Deep-fuzz pre-synth gate — exhaustive ~5min Musashi co-sim sweep
# ──────────────────────────────────────────────────────────────────────
# Designed as a pre-impl gate.  Distinct from `make fuzz` (the legacy
# nightly N=200 baseline) — this target widens every dimension the
# baseline keeps narrow, runs in parallel, and fails on ANY non-PASS
# class.  See docs/fuzz_deep.md.
#
# Knobs (override on the command line):
#   FUZZ_DEEP_N         — # seeds (default 1500)
#   FUZZ_DEEP_JOBS      — parallel workers (default 12)
#   FUZZ_DEEP_BASE_SEED — pinned base seed for reproducibility (default 100)
#   FUZZ_DEEP_TIMEOUT   — RTL cycle cap (default 400000 — instr-mix N=1000
#                         seeds are heavier than the 40-instr baseline)
#   FUZZ_DEEP_EXTRAS    — extra fuzz.py flags (default empty; use
#                         --memind-heavy here once BUG_alu_memind_*
#                         is closed)
FUZZ_DEEP_N         ?= 1500
FUZZ_DEEP_JOBS      ?= 12
FUZZ_DEEP_BASE_SEED ?= 100
FUZZ_DEEP_TIMEOUT   ?= 400000
FUZZ_DEEP_EXTRAS    ?=
# Set FUZZ_DEEP_RANDOM_DRAM=1 to seed each A0..A3 data pool's first
# 128 bytes with deterministic random bytes (Phase B3 / audit P2 §13).
# Catches data-dependent read bugs that the default 0xFF mem floor
# can't surface.  Adds ~120 instructions of preamble per seed; default
# stays off so the budget at FUZZ_DEEP_N=1500 is ~2:30 wall.  At
# FUZZ_DEEP_RANDOM_DRAM=1 the gate auto-drops N=1000 to fit budget.
FUZZ_DEEP_RANDOM_DRAM ?= 0
ifeq ($(FUZZ_DEEP_RANDOM_DRAM),1)
FUZZ_DEEP_N := 1000
FUZZ_DEEP_EXTRAS += --random-dram
endif
FUZZ_DEEP_KNOWN_FAILS := $(TB_DIR)/fuzz_fails/known_failures.txt
FUZZ_DEEP_REPRO_SEEDS := $(BUILD_DIR)/fuzz/deep_repro_seeds.txt

fuzz-deep: sim musashi-ref
	$(MAKE) -C $(TB_DIR)/models musashi-run
	@mkdir -p $(BUILD_DIR)/fuzz
	@echo "===================================================================="
	@echo " fuzz-deep: pre-synth gate"
	@echo "  N=$(FUZZ_DEEP_N) jobs=$(FUZZ_DEEP_JOBS) base_seed=$(FUZZ_DEEP_BASE_SEED)"
	@echo "  policy=strict  instr_mix=on  fail_on=MISMATCH,ERROR,TIMEOUT,WRITELOG"
	@echo "  budget: 5 min wall-clock"
	@echo "===================================================================="
	@start=$$(date +%s); \
	python3 $(FUZZ_DIR)/fuzz.py \
		--n $(FUZZ_DEEP_N) \
		--jobs $(FUZZ_DEEP_JOBS) \
		--base-seed $(FUZZ_DEEP_BASE_SEED) \
		--instr-mix \
		--write-log-policy strict \
		--fail-on MISMATCH,ERROR,TIMEOUT,WRITELOG \
		--timeout $(FUZZ_DEEP_TIMEOUT) \
		--musashi-max $(FUZZ_DEEP_TIMEOUT) \
		--exclude-seeds $(FUZZ_DEEP_KNOWN_FAILS) \
		--sim $(BUILD_DIR)/sim/Vmac_top \
		--musashi $(TB_DIR)/models/libmusashi_ref.a \
		--work $(BUILD_DIR)/fuzz \
		--as $(M68K_AS) --ld $(M68K_LD) --objcopy $(M68K_OBJCOPY) \
		--save-seed-file $(FUZZ_DEEP_REPRO_SEEDS) \
		--save-fails $(TB_DIR)/fuzz_fails \
		$(FUZZ_DEEP_EXTRAS); \
	rc=$$?; \
	end=$$(date +%s); \
	dt=$$((end - start)); \
	echo "===================================================================="; \
	if [ $$rc -eq 0 ]; then \
	    echo " fuzz-deep: PASS  N=$(FUZZ_DEEP_N) wall=$${dt}s"; \
	else \
	    echo " fuzz-deep: FAIL  N=$(FUZZ_DEEP_N) wall=$${dt}s"; \
	    echo "   repro seeds:  $(FUZZ_DEEP_REPRO_SEEDS)"; \
	    echo "   saved fails:  $(TB_DIR)/fuzz_fails/seed_*.bin"; \
	    echo "   replay one:   make fuzz-replay FILE=tb/fuzz_fails/seed_<N>.bin"; \
	fi; \
	if [ $$dt -gt 300 ]; then \
	    echo "   WARNING: wall-clock $${dt}s exceeded 5min budget"; \
	fi; \
	echo "===================================================================="; \
	exit $$rc

# ──────────────────────────────────────────────────────────────────────────────
# SCSI 53C96 differential fuzz — MAME ncr53c90 as golden reference
#
#   make fuzz-scsi                 — 50 seeds from 0
#   make fuzz-scsi N=200 START=0   — 200 seeds
#   make fuzz-scsi-replay SEED=17  — replay one seed
#
# Requires: mame (>=0.264) on PATH, chdman, and the Q700 ROM set
# (default ~/mame_q700_good/roms; override MAME_Q700_ROMS).
# Contract + triage runbook: docs/scsi_fuzz.md
# ──────────────────────────────────────────────────────────────────────────────
FUZZ_SCSI_N     ?= $(if $(N),$(N),50)
FUZZ_SCSI_START ?= $(if $(START),$(START),0)

#   make fuzz-scsi-direct          — same seeds against scsi.v ALONE
#                                    (no peripheral_bus in the DUT); use it
#                                    to attribute a divergence to the
#                                    fabric vs the chip model.
.PHONY: fuzz-scsi fuzz-scsi-replay fuzz-scsi-direct fuzz-scsi-direct-replay
fuzz-scsi: tb-scsi-fuzz-harness
	python3 $(FUZZ_DIR)/scsi_fuzz.py \
		--n $(FUZZ_SCSI_N) --start $(FUZZ_SCSI_START) \
		--rtl $(SCSI_FUZZ_BUILD)/Vtb_pb_scsi \
		--work $(BUILD_DIR)/fuzz_scsi \
		$(if $(DISK_IMAGE),--disk-image $(DISK_IMAGE))

fuzz-scsi-replay: tb-scsi-fuzz-harness
ifndef SEED
	@echo "error: set SEED=<n>"; exit 1
else
	python3 $(FUZZ_DIR)/scsi_fuzz.py \
		--seed $(SEED) \
		--rtl $(SCSI_FUZZ_BUILD)/Vtb_pb_scsi \
		--work $(BUILD_DIR)/fuzz_scsi \
		$(if $(DISK_IMAGE),--disk-image $(DISK_IMAGE))
endif

fuzz-scsi-direct: tb-scsi-fuzz-harness-direct
	python3 $(FUZZ_DIR)/scsi_fuzz.py \
		--n $(FUZZ_SCSI_N) --start $(FUZZ_SCSI_START) \
		--rtl $(SCSI_FUZZ_DIR_BUILD)/Vtb_scsi_vhdd_sd \
		--work $(BUILD_DIR)/fuzz_scsi_direct \
		$(if $(DISK_IMAGE),--disk-image $(DISK_IMAGE))

fuzz-scsi-direct-replay: tb-scsi-fuzz-harness-direct
ifndef SEED
	@echo "error: set SEED=<n>"; exit 1
else
	python3 $(FUZZ_DIR)/scsi_fuzz.py \
		--seed $(SEED) \
		--rtl $(SCSI_FUZZ_DIR_BUILD)/Vtb_scsi_vhdd_sd \
		--work $(BUILD_DIR)/fuzz_scsi_direct \
		$(if $(DISK_IMAGE),--disk-image $(DISK_IMAGE))
endif

fuzz-replay:
ifndef FILE
	@echo "error: set FILE=<path-to-failing-.bin>"; exit 1
else
	python3 $(FUZZ_DIR)/fuzz.py \
		--replay $(FILE) \
		--sim $(BUILD_DIR)/sim/Vmac_top \
		--musashi $(TB_DIR)/models/libmusashi_ref.a \
		--work $(BUILD_DIR)/fuzz
endif

fuzz-replay-seeds:
ifndef SEEDS_FILE
	@echo "error: set SEEDS_FILE=<path-to-seed-list>"; exit 1
else
	python3 $(FUZZ_DIR)/fuzz.py \
		--seed-file $(SEEDS_FILE) \
		--sim $(BUILD_DIR)/sim/Vmac_top \
		--musashi $(TB_DIR)/models/libmusashi_ref.a \
		--work $(BUILD_DIR)/fuzz \
		--as $(M68K_AS) --ld $(M68K_LD) --objcopy $(M68K_OBJCOPY) \
		--write-log-policy $(FUZZ_WRITE_LOG_POLICY)
endif

# ──────────────────────────────────────────────────────────────────────────────
# Deterministic final-register comparison against Musashi
# ──────────────────────────────────────────────────────────────────────────────
REGSTATE_WORK ?= $(BUILD_DIR)/regstate-sweep
REGSTATE_ARGS ?= \
	--case andi_b_imm_d0_b00000000_i00 \
	--case ori_b_imm_d0_b00000000_i00 \
	--case move_w_imm_d0_bffffffff_i0000 \
	--case move_w_imm_d0_b12345678_i8000 \
	--case movea_w_imm_a4_b00000000_w8001 \
	--case movea_postinc_a4_b00104000_a0 \
	--case movea_postinc_a5_b00105000_a1 \
	--case move_w_postinc_d0_bcafebabe_w8001 \
	--case move_b_postinc_d1_b13572468_b80 \
	--case swap_d2_b11223344_u \
	--case extb_d3_b12345680_u \
	--case move_reg_partial_d0_b12345678_b_saa55cc33 \
	--case move_reg_partial_d1_ba5a500ff_w_s13572468 \
	--case alu_reg_bw_supported_d2_b12345678_andb_saa55cc33 \
	--case alu_reg_bw_supported_d3_ba5a500ff_andw_s13572468 \
	--case alu_reg_bw_supported_d4_b80007f80_orb_s13572468 \
	--case alu_reg_bw_supported_d5_b5a5aff00_orw_saa55cc33 \
	--case alu_reg_bw_supported_d6_b80007f80_addb_sffffffff \
	--case alu_reg_bw_supported_d7_b5a5aff00_addw_saa55cc33 \
	--case quick_reg_supported_d5_b123456fc_addqb_q4 \
	--case quick_reg_supported_d1_b0badbe00_subqb_q1 \
	--case quick_reg_supported_d3_b007c00fc_addqw_q1 \
	--case quick_reg_supported_d2_babcd0003_subqw_q4 \
	--case quick_reg_supported_d0_b00000000_addql_q8 \
	--case quick_reg_supported_d4_b00000008_subql_q8 \
	--case quick_reg_supported_a1_b00102000_addql_q4 \
	--case quick_reg_supported_a2_b00103008_subql_q8 \
	--case quick_mem_supported_d0_b00000000_addqb_q4 \
	--case quick_mem_supported_d1_b00000000_subqb_q1 \
	--case quick_mem_supported_d2_b00000000_addqw_q1 \
	--case quick_mem_supported_d3_b00000000_subqw_q4 \
	--case quick_mem_supported_d4_b00000000_addql_q8 \
	--case quick_mem_supported_d5_b00000000_subql_q8 \
	--case rom_frontier_movew_areg_d0_b12345678_a7w8000 \
	--case rom_frontier_movel_areg_postinc_d1_b00000000_a0a7post \
	--case rom_frontier_movem_pc_disp_d0_b00000000_d0_d5 \
	--case rom_frontier_lea_pc_indexed_a0_b00000020_a0_alias \
	--case rom_frontier_lea_scaled_alias_a5_b00100000_d1w4 \
	--case rom_frontier_suba_mem_a1_b08000000_a5_ind \
	--case rom_frontier_scc_indexed_d2_b00000000_seq_d1l \
	--case rom_frontier_cmpb_indexed_d2_b00000000_d2w \
	--case rom_frontier_notb_indexed_d2_b00000000_d2w \
	--case rom_frontier_jmp_pc_indexed_d0_b00000000_d3w \
	--case rom_frontier_moveb_indexed_dst_d1_b00000000_d3l \
	--case rom_frontier_movel_full_memind_d0_b00000000_a4bd_od \
	--case rom_frontier_movel_full_memind_no_outer_d1_b00000000_a4bd_null \
	--case rom_frontier_subq_mem_disp_d0_b00000000_a4m16 \
	--case rom_frontier_movew_imm_disp_d2_b00000000_a4m154 \
	--case rom_frontier_movel_indexed_mem_to_indexed_d0_b00000000_a1d2w_a0d2w

.PHONY: regstate-compare
regstate-compare: sim
	$(MAKE) -C $(TB_DIR)/models musashi-run
	python3 $(TOOLS_DIR)/regstate/regstate_compare.py \
		--sim $(BUILD_DIR)/sim/Vmac_top \
		--musashi $(TB_DIR)/models/musashi_run \
		--work $(REGSTATE_WORK) \
		--as $(M68K_AS) --ld $(M68K_LD) --objcopy $(M68K_OBJCOPY) \
		$(REGSTATE_ARGS)

MUSASHI_PARITY_WORK ?= $(BUILD_DIR)/musashi-parity
MUSASHI_PARITY_ARGS ?= \
	--completion stop-pc \
	--timeout 50000 --musashi-max 50000 \
	--compare-pc --compare-sr --compare-ccr --compare-a7 \
	--case parity_trap_rte_ccr_restore_d4_b00000000_trap0 \
	--case parity_user_stack_switch_a7_b00000000_trap0 \
	--case parity_vector_frame_regs_d1_b00000000_fmt0 \
	--case parity_rom_frontier_d2_b00000000_probe

.PHONY: musashi-parity
musashi-parity: sim
	$(MAKE) -C $(TB_DIR)/models musashi-run
	python3 $(TOOLS_DIR)/regstate/regstate_compare.py \
		--sim $(BUILD_DIR)/sim/Vmac_top \
		--musashi $(TB_DIR)/models/musashi_run \
		--work $(MUSASHI_PARITY_WORK) \
		--as $(M68K_AS) --ld $(M68K_LD) --objcopy $(M68K_OBJCOPY) \
		$(MUSASHI_PARITY_ARGS)

MUSASHI_ADVERSARIAL_WORK ?= $(BUILD_DIR)/musashi-adversarial
MUSASHI_ADVERSARIAL_ARGS ?= \
	--include-memory \
	--timeout 50000 --musashi-max 50000 \
	--compare-a7 \
	--case parity_word_odd3_d1_b00000000_odd3 \
	--case parity_store_load_same_addr_d2_b00000000_repeat \
	--case parity_movem_an_in_list_a7_b00000000_a7list

.PHONY: musashi-adversarial
musashi-adversarial: sim
	$(MAKE) -C $(TB_DIR)/models musashi-run
	python3 $(TOOLS_DIR)/regstate/regstate_compare.py \
		--sim $(BUILD_DIR)/sim/Vmac_top \
		--musashi $(TB_DIR)/models/musashi_run \
		--work $(MUSASHI_ADVERSARIAL_WORK) \
		--as $(M68K_AS) --ld $(M68K_LD) --objcopy $(M68K_OBJCOPY) \
		$(MUSASHI_ADVERSARIAL_ARGS)

# ──────────────────────────────────────────────────────────────────────────────
# Host-side Python tooling (m68kctl) unit tests
#
# Runs tb/tests/host/ against the MockDevice — no FPGA required.  Uses
# unittest to avoid a pytest dependency; pytest works too if present.
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: tb-host
tb-host:
	@echo "Running m68kctl host-side tests..."
	@if command -v python3 >/dev/null; then \
		PYTHONPATH=$(TOOLS_DIR) python3 -m unittest discover -s $(TB_DIR)/tests/host -v; \
	else \
		echo "error: python3 not found"; exit 1; \
	fi

# ──────────────────────────────────────────────────────────────────────────────
# jtag_repl.tcl host-side helper tests — no FPGA, no Vivado.
#
# Guards the correctness-critical numeric parsing and read wrappers in
# tools/jtag_repl.tcl (hex-by-default operands, alignment rejection, the
# rdx integer accessor, capability-bit lookup).  These are the paths that
# silently fabricated plausible-but-wrong values before 2026-07-26.
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: tb-jtag-repl-host
tb-jtag-repl-host:
	@echo "Running jtag_repl.tcl host-side helper tests..."
	@if command -v tclsh >/dev/null; then \
		tclsh $(TB_DIR)/tests/host/test_jtag_repl_helpers.tcl < /dev/null; \
	else \
		echo "error: tclsh not found"; exit 1; \
	fi

# ──────────────────────────────────────────────────────────────────────────────
# tb-gdbstub-host — host-side GDB stub tests.  No FPGA, no Vivado, no Tcl.
#
# Drives tools/gdbstub.py + tools/gdbdbg.py + tools/macsym.py against
# tb/tests/host/fake_jtag_repl.py, a software model of the debug register file
# that reproduces the hardware's awkward semantics on purpose (arch-apply
# auto-resume, the break-PC hit latch read-bit-15/write-bit-14 asymmetry,
# poisoned snap-chain reads while running, D-cache bypass).  Passing here
# proves the HOST is right; it says nothing about the board.
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: tb-gdbstub-host
tb-gdbstub-host:
	@echo "Running gdbstub host-side tests..."
	@if command -v python3 >/dev/null; then \
		PYTHONPATH=$(TOOLS_DIR):$(TB_DIR)/tests/host python3 -m unittest discover \
			-s $(TB_DIR)/tests/host \
			-p 'test_gdbstub_host.py' -v; \
	else \
		echo "error: python3 not found"; exit 1; \
	fi

.PHONY: tb-macsym-host
tb-macsym-host:
	@echo "Running macsym symbol-file tests..."
	@if command -v python3 >/dev/null; then \
		PYTHONPATH=$(TOOLS_DIR) python3 -m unittest discover \
			-s $(TB_DIR)/tests/host \
			-p 'test_macsym.py' -v; \
	else \
		echo "error: python3 not found"; exit 1; \
	fi

.PHONY: tb-asc-wav-export-smoke
tb-asc-wav-export-smoke:
	@echo "Running ASC WAV exporter smoke test..."
	@if command -v python3 >/dev/null; then \
		PYTHONPATH=$(TOOLS_DIR) python3 -m unittest discover \
			-s $(TB_DIR)/tests/host \
			-p 'test_asc_wav_from_periph_log.py' -v; \
	else \
		echo "error: python3 not found"; exit 1; \
	fi

# ──────────────────────────────────────────────────────────────────────────────
# Run every unit testbench in sequence and summarise pass/fail counts.
#
# This is the SIM-FIRST gate: each tb builds + runs in its own directory
# (see the targets above), so a clean tree yields the full directed matrix.
# A failure in any tb is surfaced in the summary; the harness exits
# non-zero only if some tb FAILED (not if it was merely skipped).
#
# TB_KNOWN_BROKEN lists tbs that are expected to fail with a PRE-EXISTING
# issue (not introduced by whatever changes are in flight).  They run and
# their outputs are shown, but they don't gate the tb-all exit code.  As
# soon as the underlying bug is fixed, remove the name from this list to
# re-gate it.
# ──────────────────────────────────────────────────────────────────────────────
# Platform + peripheral unit-tb suite — this is the repo-root `make test`
# gate (see `test: tb-all` above).  Core-track unit tbs (tb-lsu, tb-alu,
# tb-rat, tb-mmu, ras_test, ...) are NOT here: they verilate rtl/core/*,
# which moved to the cpu/ submodule in the SoC split and is exercised by
# cpu/Makefile's own `make test` / `make tb-all`, not this one.  tb-debug
# (rtl/core/debug/debug_ctrl.v), tb-hw-checkerboard-path and tb-cold-boot
# (both wanted a real CPU via the retired flat RTL_SRCS list) and tb-scaler
# (rtl/board/video_phy/scaler.v, deleted) are gone for the same reason —
# see the retirement stubs / comments left in their place above.
#
# ORPHAN AUDIT (2026-08-08, measured — re-measure, don't trust these numbers
# once the lists move).  130 real `tb-*` targets are declared in this
# Makefile (plus tb-all itself); 80 are listed below and 8 in
# TB_KNOWN_BROKEN.  That leaves 49 declared targets in NEITHER list: they
# are never built by any aggregate target, so nothing notices when a port
# reshape stops them elaborating.  tb-dma-integration was one of them and
# had been dead through TWO axi_xbar master-port changes before anyone
# looked (task #249).
# Later retirement removed tb-dma-integration and its axi_n64_to_wide
# wrapper. It must not be reintroduced into ALL_TBS without a real target;
# the current DMA engine is covered by tb-dma-engine and tb-dma-l2c.
#
# The disk/SCSI/SD path was swept in that task and is now accounted for:
# tb-scsi-c96-mame-chunk (3/3), tb-scsi-c96-sm43-chunk (291/291),
# tb-scsi-dual (6/6), tb-scsi-trace-ring (16/16), tb-sd-provision (5/5),
# tb-vhdd-ctrl (42 checks) and tb-dma-integration (23/23) are all listed
# below.  tb-sd-jtag-writer is listed below AND in
# TB_KNOWN_BROKEN — note that membership of TB_KNOWN_BROKEN alone does
# NOTHING: tb-all iterates ALL_TBS and only consults TB_KNOWN_BROKEN to
# decide whether a failure gates the exit code.  A target that is only in
# TB_KNOWN_BROKEN is still never built, i.e. still exactly as invisible as
# an unlisted orphan.  Every existing TB_KNOWN_BROKEN entry is in both
# lists; keep it that way.
# 2026-08-19: tb-scsi-sd-e2e-c96 and tb-scsi-sd-e2e-c96-core100 promoted
# into the list below.  They were the sharpest instance of this gap: the
# gated tb-scsi-sd-e2e compiles 11 scenarios against the BARE 5380 front
# end, while the two c96 variants compile 18 against the 53C96 + DAFB
# pseudo-DMA front end — and the Quadra 700 ships the 53C96.  The gated
# suite therefore covered a front end this machine does not use, and the
# one it does use went red under 7c348be (whose own gate list does not
# name them) without anyone noticing.
# Two disk-path targets were deliberately NOT added, with reasons:
#   tb-sd-bridge-pulse    — a MEASUREMENT harness, not a gate: its main()
#                           ends `return 0` unconditionally ("the printed
#                           table is the product") and it currently prints
#                           **LOSS** for SP=4 and for the adjacent-pair
#                           shape while still exiting 0.  Listing it would
#                           add a permanently-[OK] row that reports data
#                           loss in its own output.  Give it a real exit
#                           status first, then list it.
#   tb-rom-boot-scsi-smoke — a retirement tombstone: the recipe just prints
#                           "tb-rom-boot-* were removed: rtl/core/* moved to
#                           the cpu/ submodule" and exits 2.
# Of the 49 remaining unlisted targets, those two are the only ones anyone
# has looked at; the other 47 (video/DAFB/ADB/ASC/rom-boot/lockstep
# families) are UNAUDITED.  Sweeping them is a bigger judgement call than
# task #249 took on; this comment exists so the next person sees the gap
# instead of rediscovering it one rotted target at a time.
# RE-GATED 2026-09-03 (orphan audit): these were declared but in NEITHER
# ALL_TBS nor TB_KNOWN_BROKEN, so nothing built them.  The 2026-08-08 audit
# note below counted 49 such targets; a re-measure on 2026-09-03 found 68 --
# the rot is getting worse, not better.  These seven are the highest-value
# ones and all were verified passing before being added:
#   tb-framebuffer-pixel  the ONLY pixel-exact CPU->VRAM->scanout coverage
#   tb-sd-boot-mirror     gates MIRROR_LOW_RAM=1, which cpu040 structurally
#                         depends on (fpga_top_boot_master.vh)
#   tb-l2c-wstream(-off)  the write-stream throughput harness that proves
#   tb-l2c-sctr(-off)     the sectored/full-line-write path -- these are the
#                         measurements docs/fabric_concurrency_contract.md
#                         clause C9 cites, and they were themselves ungated
#   tb-l2c-bypass-window  l2c_bypass window decode, both mask senses
ALL_TBS := \
	tb-dafb-via-irq \
	tb-framebuffer-pixel tb-sd-boot-mirror tb-l2c-wstream tb-l2c-wstream-off \
	tb-l2c-sctr tb-l2c-sctr-off tb-l2c-bypass-window \
	tb-axi-xbar tb-axi-w-skid tb-vram tb-vram-cpu-write tb-vram-xbar-e2e tb-sd-boot tb-sd-boot-zero tb-sd-ctrl \
	tb-mode-admit \
	tb-scanout-placement-sync tb-scanout-frames tb-scanout-frames-negctl \
	tb-scanout-1bpp tb-scanout-1bpp-1080p tb-scanout-1bpp-negctl tb-video tb-video-pattern tb-video-checkerboard tb-video-smoke tb-vram-scaler-firstlight tb-dafb-scanout tb-dafb-mode-matrix tb-dafb-mode-matrix-negctl tb-mode-decode tb-dafb-24bpp-capacity tb-ddr-model tb-axi-ddr-contract tb-axi-ddr4-mig-bridge tb-via1 tb-via2 tb-scc tb-scsi tb-sd-scsi-lba-mapper tb-sd-scsi-bridge tb-scsi-sd-e2e tb-scsi-sd-e2e-dual tb-scsi-sd-e2e-dual-noselect \
	tb-scsi-sd-e2e-c96 tb-scsi-sd-e2e-c96-core100 \
	tb-scsi-sd-e2e-ra tb-scsi-sd-e2e-c96-ra \
	tb-vhdd-readahead tb-vhdd-readahead-off \
	tb-irq-agg tb-peripheral-bus \
	tb-rtc tb-q700-eth-sonic tb-q700-eth-sonic-engine tb-q700-sonic-tx tb-q700-sonic-rx tb-q700-eth-stream-share tb-q700-toggle-rx tb-q700-toggle-rx-mut tb-asc tb-audio-i2s tb-audio-pwm tb-adb tb-adb-inject \
	tb-iwm tb-scsi-c96-probe tb-scsi-c96-register tb-scsi-c96-inquiry tb-scsi-c96-read6 tb-scsi-c96-cmdout-drain tb-scsi-c96-nondma-trailing tb-turboscsi tb-pb-scsi tb-async-fifo tb-axi-async-bridge tb-glue \
	tb-scsi-c96-mame-chunk tb-scsi-c96-sm43-chunk tb-scsi-dual tb-scsi-trace-ring tb-sonic-trace-ring tb-scsi-trace-pb \
	tb-sd-provision tb-sd-jtag-writer tb-vhdd-ctrl \
	tb-dma-ctrl tb-dma-engine tb-dma-l2c tb-axil-null-slave tb-axil-split2 \
	tb-pram-sd tb-pram-sd-populated tb-pram-sd-autoload \
	tb-l2c tb-l2c-stress tb-l2c-collision tb-l2c-bypass-all tb-l2c-chain tb-l2c-chain-off \
	tb-axi-vram-mux3 tb-vram-ddr-chain tb-vram-ddr-chain-nol2c tb-fb-reader-ddr-chain \
	tb-scanout-ddr-frames \
	tb-video-smoke-ddr tb-video-smoke-ddr-negctl \
	tb-host tb-periph-event-log tb-mem-model \
	tb-gdbstub-host tb-macsym-host \
	tb-n2w-vram-byte tb-n2w-watchdog

# Pre-existing failures (verified against the fix/build-truth-hygiene
# branch point, 4db20c6, via `git stash` A/B — none of these are caused
# by this task's changes):
#   (tb-debug-stop was removed from this list 2026-08-19: it now builds
#    cpu/rtl/core/debug/debug_stop_manager.v against cpu/tb's testbench
#    and passes 10/10.  Its old failures were the SoC's stale inline
#    duplicate of the module measured against a stale local testbench.)
#   tb-vram-xbar-e2e      — tb_vram_xbar_e2e.v still instantiates axi_xbar
#                           with the pre-2026-07-16 5-master port list
#                           (m4_*); PINNOTFOUND/PINMISSING against the
#                           current 3M×6S xbar.  Testing-track fix.
#   tb-axi-ddr-contract   — tb_axi_ddr_contract.v missing axi_narrow_to_
#                           wide's n_arsize pin; same stale-wrapper class
#                           as tb-vram-xbar-e2e.
#   tb-video / tb-video-smoke — real pixel-pipeline mismatches
#                           (SMPTE-bar / scanout content diffs), unrelated
#                           to the scaler.v deletion in this task (video_
#                           top.v never instantiated scaler; verified by
#                           reproducing the same failure on unmodified
#                           HEAD).
#   tb-host               — tools/m68kctl/regs.py is missing SDP_VERSION_
#                           MAGIC / SDP_BLOCK_SIZE that device.py/cli.py
#                           reference; a host-tooling drift bug, no RTL/
#                           Makefile involvement.
#   tb-mem-model          — tb/models/mem_model.cpp does not exist in
#                           this repo (tb/tb_mem_model.cpp's counterpart
#                           was never added here).
#   tb-sd-jtag-writer     — PORT-LIST DRIFT, not a logic failure (added to
#                           this list 2026-08-08, task #249, having been in
#                           neither list before).  rtl/board/sd_jtag_writer.v
#                           :251 instantiates sd_ctrl without 8 of its pins:
#                           the input `crc_check_en` and the outputs
#                           dbg_rd_crc_calc, dbg_rd_crc_recv,
#                           dbg_last_real_r1, dbg_cur_cmd, dbg_lba_lat,
#                           dbg_last_crc7_sent, dbg_last_poll_cnt.
#                           `-Wall` promotes the 8 PINMISSING warnings to
#                           "%Error: Exiting due to 8 warning(s)", so the
#                           target never builds.  The 7 dbg_* are outputs and
#                           safe to leave unconnected explicitly; the fix is
#                           NOT purely mechanical because `crc_check_en` is a
#                           functional INPUT left dangling — unobservable
#                           today only because this writer hardwires
#                           cmd_type to CT_CMD24 and crc_check_en gates the
#                           READ-data CRC path (sd_ctrl.v:1058), so it must
#                           be tied deliberately rather than to whatever
#                           silences the warning.  That is an rtl/board
#                           decision, and rtl/board/sd_ctrl.v is owned by
#                           another agent right now.
# tb-dafb-via-irq added 2026-09-03, in BOTH lists (membership here alone
# does nothing -- see the note above: tb-all iterates ALL_TBS).  It was in
# NEITHER list, and the 2026-08-27 throttle commit d47c5c4 added a via1.v
# port (t2_armed_out) that this tb's own via1 instantiation never
# connected -- so it stopped BUILDING (Verilator PINMISSING) and nothing
# noticed for a week.  The throttle is gone now and the build is restored;
# the 6 remaining failures are older still (16 PASS / 6 FAIL at fd33c2c,
# the throttle commit's own parent), so they are genuinely pre-existing
# logic bugs, not fallout.  Gated here so it at least RUNS -- which is
# exactly what would have caught the port break.  Remove when the 6 are
# fixed.
TB_KNOWN_BROKEN := tb-vram-xbar-e2e tb-axi-ddr-contract tb-video tb-video-smoke tb-host tb-mem-model tb-sd-jtag-writer \
                   tb-dafb-via-irq

.PHONY: tb-all
tb-all:
	@total=0; pass=0; fail=0; xfail=0; fails=""; \
	for tb in $(ALL_TBS); do \
		total=$$((total+1)); \
		echo ""; \
		echo "════════════════════════════════════════════════════════════"; \
		echo "  $$tb"; \
		echo "════════════════════════════════════════════════════════════"; \
		if $(MAKE) -s $$tb; then \
			pass=$$((pass+1)); \
			echo "  [OK] $$tb"; \
		else \
			if echo " $(TB_KNOWN_BROKEN) " | grep -qw "$$tb"; then \
				xfail=$$((xfail+1)); \
				echo "  [XFAIL] $$tb (pre-existing, tracked in TB_KNOWN_BROKEN)"; \
			else \
				fail=$$((fail+1)); \
				fails="$$fails $$tb"; \
				echo "  [FAIL] $$tb"; \
			fi; \
		fi; \
	done; \
	echo ""; \
	echo "════════════════════════════════════════════════════════════"; \
	echo "  tb-all: PASS=$$pass XFAIL=$$xfail FAIL=$$fail (of $$total)"; \
	if [ $$fail -gt 0 ]; then \
		echo "  FAILED:$$fails"; \
		exit 1; \
	fi

# ──────────────────────────────────────────────────────────────────────────────
# Clean
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)
	rm -f sim.fst sim.vcd

.PHONY: clean-synth
clean-synth:
	rm -rf $(VIVADO_IMPL_DIR)

# ──────────────────────────────────────────────────────────────────────────────
# PM workflow helpers (tools/pm/)
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: pm-status
pm-status:
	@tools/pm/status

# ──────────────────────────────────────────────────────────────────────────────
# FPGA HDMI capture helpers
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: fpga-video-list fpga-video-view fpga-video-still
fpga-video-list:
	@$(FPGA_VIDEO_TOOL) list

fpga-video-view:
	@$(FPGA_VIDEO_TOOL) view

fpga-video-still:
	@$(FPGA_VIDEO_TOOL) still

# ──────────────────────────────────────────────────────────────────────────────
# Help
# ──────────────────────────────────────────────────────────────────────────────
.PHONY: help
help:
	@echo "m68k-ooo build system"
	@echo ""
	@echo "Simulation:"
	@echo "  make sim              Build Verilator simulation"
	@echo "  make test             Run all directed tests"
	@echo "  make test TEST=foo    Run specific test"
	@echo "  make test TEST_GROUP=exception-frontier"
	@echo "  make test STOP_ON_FAIL=1   Stop directed suite at first regression"
	@echo "  make test TEST=foo WAVES=1   Run with FST waveform dump"
	@echo "  make decode-check     Cross-check decode vs Musashi ISS"
	@echo "  make regstate-compare Deterministic final-register sweep vs Musashi"
	@echo "  make musashi-parity   Stop-PC D/A/PC/SR/CCR parity slice"
	@echo "  make tb-fpga-top-rom  Run ROM execution through full fpga_top RTL"
	@echo "  make coverage         ISA opcode coverage report"
	@echo "  make tb-via           Run VIA1 + VIA2 unit tbs"
	@echo "  make tb-axi-ddr-contract  Run 32-bit AXI to DDR contract tb"
	@echo "  make tb-axi-ddr4-mig-bridge  Run repo-DDR to pcie_test-MIG bridge contract tb"
	@echo "  make ddr-reference-check  Compare DDR pins/MIG facts against pcie_test + factory image_ku5p"
	@echo "  make ddr4-mig-cache-status  Fast cache probe for repo DDR4 MIG artifacts"
	@echo "  make ddr4-mig         Generate repo-owned DDR4 MIG XCI/DCP + manifest (cached)"
	@echo "  make pcie-xdma-validate  Generate/check repo-owned XDMA XCI + manifest"
	@echo "  make pcie-xdma       Generate repo-owned XDMA XCI/DCP/stub + manifest"
	@echo "  make fpga-first-hw-offline-preflight  DDR/MIG contract + FPGA-top lint + clk_rst, no Vivado"
	@echo "  make fpga-first-hw-preflight  Canonical first-hardware 100 MHz preflight"
	@echo ""
	@echo "Synthesis (Vivado):"
	@echo "  make synth            Generic synth-only flow; not the canonical board bring-up target"
	@echo "  make impl             Generic full place+route; REAL_FPGA_BUILD=1 now fails unless debug-capable or explicitly overridden"
	@echo "  make verify-fpga-debug-artifacts  Check that the selected build emitted bit/ltx/manifest for debug"
	@echo "  make fpga-100mhz-jtag-bitstream  Alias for the canonical 100 MHz debug-enabled real-MIG bitstream"
	@echo "  make fpga-100mhz-jtag-bitstream-dram  Canonical 100 MHz debug-enabled real-MIG bitstream"
	@echo "  make fpga-50mhz-jtag-bitstream  Alias for the DRAM-backed 50 MHz first-light impl"
	@echo "  make fpga-50mhz-jtag-bitstream-dram  50 MHz first-light impl using real DDR4"
	@echo "  make pcie-xdma-bitstream  100 MHz real-MIG PCIe/XDMA bitstream with VIO probes"
	@echo "  make jtag-discover    List Vivado hw_server JTAG targets; no programming"
	@echo "  make jtag-status      Decoded one-shot VIO/JTAG status snapshot"
	@echo "  make jtag-tui         Refresh-loop JTAG bring-up dashboard"
	@echo ""
	@echo "Debugging the Mac with GDB (see docs/debugging_the_mac_on_fpga.md):"
	@echo "  tools/macsym.py build -o build/macos.elf   Build the GDB symbol file"
	@echo "  tools/gdbstub.py --port 1234              Attach GDB to the board"
	@echo "  tools/gdbstub.py --fake --port 1234       Same, simulated, no board"
	@echo "  make tb-gdbstub-host  GDB stub host tests (no FPGA, no Vivado)"
	@echo "  make tb-macsym-host   Symbol-map/ELF tests (no FPGA)"
	@echo "  make tb-jtag-repl-host  jtag_repl.tcl helper tests (no FPGA)"
	@echo "  make fpga-100mhz-preflight  100 MHz lint/elab/pin preflight, no bitstream"
	@echo "  make fpga-50mhz-preflight  50 MHz lint/elab/pin preflight, no bitstream"
	@echo "  make vivado-dry-run   Parse-only Vivado RTL/XDC elaboration"
	@echo "  make pcie-xdma-dry-run  Parse-only real-MIG + PCIe/XDMA host shell"
	@echo "  make pcie-xdma-pincheck  Parse-only DDR + PCIe pin/constraint shell"
	@echo "  make clock-report     Parse-only clock audit"
	@echo "  make ddr-pincheck     DDR4 real-pin RTL/XDC smoke check"
	@echo "  make ddr-pcie-test-check  Compare DDR shell against ~/FPGA/pcie_test"
	@echo "  make timing-audit     Summarize Vivado timing/check_timing blockers"
	@echo "  make tb-clk-rst       clk_rst divide/reset unit tb"
	@echo "  make timing           Print WNS from latest impl"
	@echo "  make gui [STEP=route] Open Vivado GUI on checkpoint"
	@echo ""
	@echo "Misc:"
	@echo "  make fpga-video-list  List V4L2 HDMI capture devices/modes"
	@echo "  make fpga-video-view  Open live FPGA HDMI capture preview"
	@echo "  make fpga-video-still Save one FPGA HDMI capture still under captures/"
	@echo "  make ras_test         Build + run RAS unit tests (standalone)"
	@echo "  make lint [MODULE=x]  Lint RTL (all, or specific module)"
	@echo "  make musashi          Build Musashi reference ISS"
	@echo "  make clean            Remove build artefacts"
.PHONY: tb-peripheral-reset-sequencer
tb-peripheral-reset-sequencer: $(BUILD_DIR)/peripheral_reset_sequencer/Vperipheral_reset_sequencer
	@echo "Running storage-safe peripheral reset sequencer tb..."
	$(BUILD_DIR)/peripheral_reset_sequencer/Vperipheral_reset_sequencer

$(BUILD_DIR)/peripheral_reset_sequencer/Vperipheral_reset_sequencer: \
		$(RTL_DIR)/soc/peripheral_reset_sequencer.v \
		$(TB_DIR)/tb_peripheral_reset_sequencer.cpp
	@mkdir -p $(BUILD_DIR)/peripheral_reset_sequencer
	$(VERILATOR) --cc --exe --build --assert \
		--x-assign fast --x-initial fast -O3 \
		-Wall -Wno-UNUSED -Wno-UNDRIVEN -Wno-PINCONNECTEMPTY \
		-Wno-DECLFILENAME -Wno-CASEINCOMPLETE -Wno-WIDTHTRUNC \
		-Wno-WIDTHEXPAND \
		-GPULSE_CYCLES=8 \
		-Mdir $(BUILD_DIR)/peripheral_reset_sequencer \
		--top-module peripheral_reset_sequencer \
		$(RTL_DIR)/soc/peripheral_reset_sequencer.v \
		$(TB_DIR)/tb_peripheral_reset_sequencer.cpp \
		-CFLAGS "-std=c++17"
