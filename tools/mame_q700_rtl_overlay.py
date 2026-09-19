#!/usr/bin/env python3
# Includes/adapts MAME macquadra700.cpp fragments, Copyright R. Belmont.
# Those portions retain BSD-3-Clause; original contributions are MIT except
# the generated BSD-3-Clause helper. See THIRD_PARTY_NOTICES.md and
# LICENSES/MAME-BSD-3-Clause.txt.
"""Install a Q700 MAME overlay for the RTL bridge protocol.

This patches a pinned MAME checkout in place.  The patched Quadra 700 map can
forward the main MMIO peripheral windows to a blocking RTL bridge socket.  Use
tools/mame_axi_periph_bridge.cpp for the broad AXI peripheral_bus endpoint.
"""

from __future__ import annotations

import argparse
import tempfile
from pathlib import Path


HEADER = r'''// license:BSD-3-Clause
// copyright-holders:m68k-ooo contributors
// Header-only UNIX socket client for m68k-ooo's blocking MAME RTL bridge.

#ifndef MAME_APPLE_RTL_BRIDGE_SOCKET_H
#define MAME_APPLE_RTL_BRIDGE_SOCKET_H

#include <cerrno>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

#if defined(__unix__) || defined(__APPLE__)
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif

class rtl_mmio_trace
{
public:
	rtl_mmio_trace(const char *env_name)
	{
		const char *path = std::getenv(env_name);
		if (path && path[0])
			m_file = std::fopen(path, "w");
	}

	~rtl_mmio_trace()
	{
		if (m_file)
			std::fclose(m_file);
	}

	bool enabled() const { return m_file != nullptr; }

	void log(const char *mode, const char *op, const char *label, uint32_t addr,
		uint8_t size, uint32_t data, uint32_t mem_mask, uint32_t pc, uint32_t cpu_cycles)
	{
		if (!m_file)
			return;
		std::fprintf(m_file,
			"mame-mmio mode=%s op=%s label=%s size=%u addr=0x%08x data=0x%08x mem_mask=0x%08x pc=0x%08x cycles=%u\n",
			mode, op, label, unsigned(size), addr, data, mem_mask, pc, cpu_cycles);
		std::fflush(m_file);
	}

	void divergence(const char *label, uint32_t addr, uint8_t size, uint32_t mame_data,
		uint32_t rtl_data, uint32_t mem_mask, uint32_t pc, uint32_t cpu_cycles)
	{
		if (!m_file)
			return;
		std::fprintf(m_file,
			"mame-mmio-divergence label=%s size=%u addr=0x%08x mame=0x%08x rtl=0x%08x mem_mask=0x%08x pc=0x%08x cycles=%u\n",
			label, unsigned(size), addr, mame_data, rtl_data, mem_mask, pc, cpu_cycles);
		std::fflush(m_file);
	}

	void accepted_divergence(const char *reason, const char *label, uint32_t addr, uint8_t size, uint32_t mame_data,
		uint32_t rtl_data, uint32_t mem_mask, uint32_t pc, uint32_t cpu_cycles)
	{
		if (!m_file)
			return;
		std::fprintf(m_file,
			"mame-mmio-accepted-divergence reason=%s label=%s size=%u addr=0x%08x mame=0x%08x rtl=0x%08x mem_mask=0x%08x pc=0x%08x cycles=%u\n",
			reason, label, unsigned(size), addr, mame_data, rtl_data, mem_mask, pc, cpu_cycles);
		std::fflush(m_file);
	}

	void missing(const char *reason, const char *op, const char *label, uint32_t addr,
		uint8_t size, uint32_t data, uint32_t mem_mask, uint32_t pc, uint32_t cpu_cycles)
	{
		if (!m_file)
			return;
		std::fprintf(m_file,
			"mame-mmio-missing reason=%s op=%s label=%s size=%u addr=0x%08x data=0x%08x mem_mask=0x%08x pc=0x%08x cycles=%u\n",
			reason, op, label, unsigned(size), addr, data, mem_mask, pc, cpu_cycles);
		std::fflush(m_file);
	}

	void unmodeled(const char *op, const char *label, uint32_t addr,
		uint8_t size, uint32_t data, uint32_t mem_mask, uint32_t pc, uint32_t cpu_cycles)
	{
		if (!m_file)
			return;
		std::fprintf(m_file,
			"mame-mmio-unmodeled op=%s label=%s size=%u addr=0x%08x data=0x%08x mem_mask=0x%08x pc=0x%08x cycles=%u\n",
			op, label, unsigned(size), addr, data, mem_mask, pc, cpu_cycles);
		std::fflush(m_file);
	}

	void irq_snapshot(const char *label, uint32_t addr, uint8_t mame_irq, uint8_t rtl_irq,
		uint32_t pc, uint32_t cpu_cycles)
	{
		if (!m_file || !std::getenv("MAME_RTL_IRQ_TRACE"))
			return;
		std::fprintf(m_file,
			"mame-irq label=%s addr=0x%08x mame=0x%02x rtl=0x%02x pc=0x%08x cycles=%u\n",
			label, addr, unsigned(mame_irq), unsigned(rtl_irq), pc, cpu_cycles);
		std::fflush(m_file);
	}

	void pc_trap(const char *op, const char *label, uint32_t addr, uint8_t size,
		uint32_t data, uint32_t mem_mask, uint32_t pc, uint32_t cpu_cycles, uint32_t hits)
	{
		if (!m_file)
			return;
		std::fprintf(m_file,
			"mame-pc-trap reason=hot-pc op=%s label=%s size=%u addr=0x%08x data=0x%08x mem_mask=0x%08x pc=0x%08x cycles=%u hits=%u\n",
			op, label, unsigned(size), addr, data, mem_mask, pc, cpu_cycles, hits);
		std::fflush(m_file);
	}

private:
	std::FILE *m_file = nullptr;
};

class rtl_bridge_socket
{
public:
	rtl_bridge_socket(const char *env_name)
	{
		const char *path = std::getenv(env_name);
		if (path && path[0])
			m_path = path;
	}

	~rtl_bridge_socket()
	{
		close_fd();
	}

	bool enabled() const { return !m_path.empty(); }
	const std::string &last_error() const { return m_error; }
	uint8_t last_irq_bitmap() const { return m_last_irq_bitmap; }

	bool read_byte(uint32_t addr, uint32_t pc, uint32_t cpu_cycles, uint8_t &data, uint32_t &cycles)
	{
		uint32_t value = 0;
		if (!transaction(OP_READ, 1, addr, 0, lane_strobe(addr), pc, cpu_cycles, value, cycles))
			return false;
		data = uint8_t(value >> ((3U - (addr & 3U)) * 8U));
		return true;
	}

	bool read_word(uint32_t addr, uint32_t pc, uint32_t cpu_cycles, uint16_t &data, uint32_t &cycles)
	{
		uint32_t value = 0;
		if (!transaction(OP_READ, 2, addr, 0, word_strobe(addr), pc, cpu_cycles, value, cycles))
			return false;
		data = uint16_t(value);
		return true;
	}

	bool read_dword(uint32_t addr, uint32_t pc, uint32_t cpu_cycles, uint32_t &data, uint32_t &cycles)
	{
		return transaction(OP_READ, 4, addr, 0, 0x0f, pc, cpu_cycles, data, cycles);
	}

	bool write_byte(uint32_t addr, uint8_t data, uint32_t pc, uint32_t cpu_cycles, uint32_t &cycles)
	{
		uint32_t value = uint32_t(data) << ((3U - (addr & 3U)) * 8U);
		uint32_t ignored = 0;
		return transaction(OP_WRITE, 1, addr, value, lane_strobe(addr), pc, cpu_cycles, ignored, cycles);
	}

	bool write_word(uint32_t addr, uint16_t data, uint32_t pc, uint32_t cpu_cycles, uint32_t &cycles)
	{
		uint32_t value = (addr & 2U) ? uint32_t(data) : (uint32_t(data) << 16);
		uint32_t ignored = 0;
		return transaction(OP_WRITE, 2, addr, value, word_strobe(addr), pc, cpu_cycles, ignored, cycles);
	}

	bool write_dword(uint32_t addr, uint32_t data, uint32_t pc, uint32_t cpu_cycles, uint32_t &cycles)
	{
		uint32_t ignored = 0;
		return transaction(OP_WRITE, 4, addr, data, 0x0f, pc, cpu_cycles, ignored, cycles);
	}

private:
	static constexpr uint8_t OP_READ = 1;
	static constexpr uint8_t OP_WRITE = 2;
	static constexpr uint8_t RESP_OKAY = 0;

	static uint8_t lane_strobe(uint32_t addr)
	{
		return uint8_t(1U << (3U - (addr & 3U)));
	}

	static uint8_t word_strobe(uint32_t addr)
	{
		return (addr & 2U) ? 0x03 : 0x0c;
	}

	static void put_be32(uint8_t *p, uint32_t v)
	{
		p[0] = uint8_t(v >> 24);
		p[1] = uint8_t(v >> 16);
		p[2] = uint8_t(v >> 8);
		p[3] = uint8_t(v);
	}

	static uint32_t get_be32(const uint8_t *p)
	{
		return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
			(uint32_t(p[2]) << 8) | uint32_t(p[3]);
	}

	bool transaction(uint8_t op, uint8_t size, uint32_t addr, uint32_t wdata,
		uint8_t wstrb, uint32_t pc, uint32_t cpu_cycles, uint32_t &rdata, uint32_t &cycles)
	{
		if (m_path.empty())
		{
			m_error = "bridge socket is disabled";
			return false;
		}

#if defined(__unix__) || defined(__APPLE__)
		if (!ensure_connected())
			return false;

		uint8_t req[24] = {'M', 'R', 'T', 'B', 2, op, size, wstrb};
		put_be32(req + 8, addr);
		put_be32(req + 12, wdata);
		put_be32(req + 16, pc);
		put_be32(req + 20, cpu_cycles);
		if (!write_exact(m_fd, req, sizeof(req)))
		{
			close_fd();
			return false;
		}

		uint8_t resp[16];
		if (!read_exact(m_fd, resp, sizeof(resp)))
		{
			close_fd();
			return false;
		}

		if (std::memcmp(resp, "MRTB", 4) != 0 || (resp[4] != 1 && resp[4] != 2))
		{
			m_error = "bad bridge response header";
			return false;
		}
		if (resp[5] != RESP_OKAY)
		{
			m_error = "bridge response code " + std::to_string(resp[5]);
			return false;
		}
		m_last_irq_bitmap = resp[6];
		rdata = get_be32(resp + 8);
		cycles = get_be32(resp + 12);
		return true;
#else
		m_error = "UNIX domain sockets are not available in this build";
		return false;
#endif
	}

#if defined(__unix__) || defined(__APPLE__)
	bool ensure_connected()
	{
		if (m_fd >= 0)
			return true;

		int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
		if (fd < 0)
		{
			m_error = std::string("socket: ") + std::strerror(errno);
			return false;
		}

		sockaddr_un sa{};
		sa.sun_family = AF_UNIX;
		if (m_path.size() >= sizeof(sa.sun_path))
		{
			m_error = "socket path too long";
			::close(fd);
			return false;
		}
		std::strncpy(sa.sun_path, m_path.c_str(), sizeof(sa.sun_path) - 1);

		if (::connect(fd, reinterpret_cast<sockaddr *>(&sa), sizeof(sa)) < 0)
		{
			m_error = std::string("connect: ") + std::strerror(errno);
			::close(fd);
			return false;
		}

		m_fd = fd;
		return true;
	}

	void close_fd()
	{
		if (m_fd >= 0)
		{
			::close(m_fd);
			m_fd = -1;
		}
	}

	bool read_exact(int fd, uint8_t *buf, size_t len)
	{
		size_t got = 0;
		while (got < len)
		{
			ssize_t n = ::read(fd, buf + got, len - got);
			if (n == 0)
			{
				m_error = "short read from bridge";
				return false;
			}
			if (n < 0)
			{
				if (errno == EINTR)
					continue;
				m_error = std::string("read: ") + std::strerror(errno);
				return false;
			}
			got += size_t(n);
		}
		return true;
	}

	bool write_exact(int fd, const uint8_t *buf, size_t len)
	{
		size_t sent = 0;
		while (sent < len)
		{
			ssize_t n = ::write(fd, buf + sent, len - sent);
			if (n < 0)
			{
				if (errno == EINTR)
					continue;
				m_error = std::string("write: ") + std::strerror(errno);
				return false;
			}
			sent += size_t(n);
		}
		return true;
	}
#endif

	std::string m_path;
	std::string m_error;
	int m_fd = -1;
	uint8_t m_last_irq_bitmap = 0;
};

#endif // MAME_APPLE_RTL_BRIDGE_SOCKET_H
'''


IMPL = r'''
u8 spike_state::rtl_read8(u32 addr, const char *label)
{
	if (!rtl_bridge_label_enabled(label))
		return 0;
	const u32 pc = u32(m_maincpu->pc());
	const u32 cpu_cycles = u32(m_maincpu->total_cycles());
	u8 data = 0;
	u32 cycles = 0;
	if (!m_rtl_bridge.read_byte(addr, pc, cpu_cycles, data, cycles))
	{
		m_rtl_trace.missing("rtl-bridge-error", "r", label, addr, 1, 0, 0, pc, cpu_cycles);
		fatalerror("MAME RTL %s read failed addr=%08x pc=%08x: %s\n", label, addr, pc, m_rtl_bridge.last_error().c_str());
	}
	rtl_trace_read("rtl", addr, 1, data, 0, label);
	rtl_trace_irq_snapshot(label, addr);
	return data;
}

void spike_state::rtl_write8(u32 addr, u8 data, const char *label)
{
	if (!rtl_bridge_label_enabled(label))
		return;
	const u32 pc = u32(m_maincpu->pc());
	const u32 cpu_cycles = u32(m_maincpu->total_cycles());
	u32 cycles = 0;
	if (!m_rtl_bridge.write_byte(addr, data, pc, cpu_cycles, cycles))
	{
		m_rtl_trace.missing("rtl-bridge-error", "w", label, addr, 1, data, 0, pc, cpu_cycles);
		fatalerror("MAME RTL %s write failed addr=%08x pc=%08x data=%02x: %s\n", label, addr, pc, data, m_rtl_bridge.last_error().c_str());
	}
	rtl_trace_write("rtl", addr, 1, data, 0, label);
	rtl_trace_irq_snapshot(label, addr);
}

u16 spike_state::rtl_read16(u32 addr, const char *label)
{
	if (!rtl_bridge_label_enabled(label))
		return 0;
	const u32 pc = u32(m_maincpu->pc());
	const u32 cpu_cycles = u32(m_maincpu->total_cycles());
	u16 data = 0;
	u32 cycles = 0;
	if (!m_rtl_bridge.read_word(addr, pc, cpu_cycles, data, cycles))
	{
		m_rtl_trace.missing("rtl-bridge-error", "r", label, addr, 2, 0, 0, pc, cpu_cycles);
		fatalerror("MAME RTL %s read16 failed addr=%08x pc=%08x: %s\n", label, addr, pc, m_rtl_bridge.last_error().c_str());
	}
	rtl_trace_read("rtl", addr, 2, data, 0, label);
	rtl_trace_irq_snapshot(label, addr);
	return data;
}

void spike_state::rtl_write16(u32 addr, u16 data, u16 mem_mask, const char *label)
{
	if (!rtl_bridge_label_enabled(label))
		return;
	const u32 pc = u32(m_maincpu->pc());
	const u32 cpu_cycles = u32(m_maincpu->total_cycles());
	u32 cycles = 0;
	if (mem_mask == 0 || mem_mask == 0xffffU)
	{
		if (!m_rtl_bridge.write_word(addr, data, pc, cpu_cycles, cycles))
		{
			m_rtl_trace.missing("rtl-bridge-error", "w", label, addr, 2, data, mem_mask, pc, cpu_cycles);
			fatalerror("MAME RTL %s write16 failed addr=%08x pc=%08x data=%04x: %s\n", label, addr, pc, data, m_rtl_bridge.last_error().c_str());
		}
		rtl_trace_write("rtl", addr, 2, data, mem_mask, label);
		rtl_trace_irq_snapshot(label, addr);
		return;
	}
	if (ACCESSING_BITS_8_15)
		rtl_write8(addr, u8(data >> 8), label);
	if (ACCESSING_BITS_0_7)
		rtl_write8(addr + 1, u8(data), label);
}

u32 spike_state::rtl_read32(u32 addr, const char *label)
{
	if (!rtl_bridge_label_enabled(label))
		return 0;
	const u32 pc = u32(m_maincpu->pc());
	const u32 cpu_cycles = u32(m_maincpu->total_cycles());
	u32 data = 0;
	u32 cycles = 0;
	if (!m_rtl_bridge.read_dword(addr, pc, cpu_cycles, data, cycles))
	{
		m_rtl_trace.missing("rtl-bridge-error", "r", label, addr, 4, 0, 0, pc, cpu_cycles);
		fatalerror("MAME RTL %s read32 failed addr=%08x pc=%08x: %s\n", label, addr, pc, m_rtl_bridge.last_error().c_str());
	}
	rtl_trace_read("rtl", addr, 4, data, 0, label);
	rtl_trace_irq_snapshot(label, addr);
	return data;
}

void spike_state::rtl_write32(u32 addr, u32 data, u32 mem_mask, const char *label)
{
	if (!rtl_bridge_label_enabled(label))
		return;
	const u32 pc = u32(m_maincpu->pc());
	const u32 cpu_cycles = u32(m_maincpu->total_cycles());
	u32 cycles = 0;
	if (mem_mask == 0 || mem_mask == 0xffffffffU)
	{
		if (!m_rtl_bridge.write_dword(addr, data, pc, cpu_cycles, cycles))
		{
			m_rtl_trace.missing("rtl-bridge-error", "w", label, addr, 4, data, mem_mask, pc, cpu_cycles);
			fatalerror("MAME RTL %s write32 failed addr=%08x pc=%08x data=%08x: %s\n", label, addr, pc, data, m_rtl_bridge.last_error().c_str());
		}
		rtl_trace_write("rtl", addr, 4, data, mem_mask, label);
		rtl_trace_irq_snapshot(label, addr);
		return;
	}
	if (ACCESSING_BITS_24_31)
		rtl_write8(addr, u8(data >> 24), label);
	if (ACCESSING_BITS_16_23)
		rtl_write8(addr + 1, u8(data >> 16), label);
	if (ACCESSING_BITS_8_15)
		rtl_write8(addr + 2, u8(data >> 8), label);
	if (ACCESSING_BITS_0_7)
		rtl_write8(addr + 3, u8(data), label);
}

void spike_state::rtl_trace_read(const char *mode, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label)
{
	if (std::strcmp(mode, "mame") == 0 || !rtl_lockstep_enabled())
		rtl_check_pc_trap("r", label, addr, size, data, mem_mask);
	if (!rtl_trace_label_enabled(label))
	{
		if (std::strcmp(mode, "mame") == 0)
			rtl_trace_missing("not-bridged", "r", addr, size, data, mem_mask, label);
		return;
	}
	m_rtl_trace.log(mode, "r", label, addr, size, data, mem_mask,
		u32(m_maincpu->pc()), u32(m_maincpu->total_cycles()));
	if (std::strcmp(mode, "mame") == 0)
		rtl_trace_missing("not-bridged", "r", addr, size, data, mem_mask, label);
}

void spike_state::rtl_trace_write(const char *mode, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label)
{
	if (std::strcmp(mode, "mame") == 0 || !rtl_lockstep_enabled())
		rtl_check_pc_trap("w", label, addr, size, data, mem_mask);
	if (!rtl_trace_label_enabled(label))
	{
		if (std::strcmp(mode, "mame") == 0)
			rtl_trace_missing("not-bridged", "w", addr, size, data, mem_mask, label);
		return;
	}
	m_rtl_trace.log(mode, "w", label, addr, size, data, mem_mask,
		u32(m_maincpu->pc()), u32(m_maincpu->total_cycles()));
	if (std::strcmp(mode, "mame") == 0)
		rtl_trace_missing("not-bridged", "w", addr, size, data, mem_mask, label);
}

void spike_state::rtl_trace_missing(const char *reason, const char *op, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label)
{
	if (rtl_bridge_label_enabled(label))
		return;
	if (!rtl_require_label_enabled(label))
		return;
	const u32 pc = u32(m_maincpu->pc());
	const u32 cpu_cycles = u32(m_maincpu->total_cycles());
	m_rtl_trace.missing(reason, op, label, addr, size, data, mem_mask, pc, cpu_cycles);
	if (std::getenv("MAME_RTL_FAIL_ON_MISSING"))
		fatalerror("MAME RTL required MMIO missing %s %s addr=%08x size=%u pc=%08x\n",
			label, op, addr, unsigned(size), pc);
}

void spike_state::rtl_trace_irq_snapshot(const char *label, u32 addr)
{
	u8 mame_irq = 0;
	if (m_maincpu->input_line_state(1) == ASSERT_LINE)
		mame_irq |= 0x01;
	if (m_maincpu->input_line_state(2) == ASSERT_LINE)
		mame_irq |= 0x02;
	if (m_maincpu->input_line_state(4) == ASSERT_LINE)
		mame_irq |= 0x04;
	m_rtl_trace.irq_snapshot(label, addr, mame_irq, m_rtl_bridge.last_irq_bitmap(),
		u32(m_maincpu->pc()), u32(m_maincpu->total_cycles()));
}

bool spike_state::rtl_lockstep_enabled() const
{
	const char *value = std::getenv("MAME_RTL_LOCKSTEP");
	return value && value[0] && std::strcmp(value, "0") != 0;
}

bool spike_state::rtl_lockstep_return_mame() const
{
	const char *value = std::getenv("MAME_RTL_READ_SOURCE");
	return value && std::strcmp(value, "mame") == 0;
}

bool spike_state::rtl_lockstep_return_mame(const char *label) const
{
	const char *rtl_labels = std::getenv("MAME_RTL_READ_SOURCE_RTL_LABELS");
	if (rtl_labels && rtl_labels[0] && rtl_label_filter_enabled(rtl_labels, label))
		return false;
	const char *mame_labels = std::getenv("MAME_RTL_READ_SOURCE_MAME_LABELS");
	if (mame_labels && mame_labels[0] && rtl_label_filter_enabled(mame_labels, label))
		return true;
	return rtl_lockstep_return_mame();
}

bool spike_state::rtl_bridge_label_enabled(const char *label) const
{
	if (!m_rtl_bridge.enabled())
		return false;
	return rtl_label_filter_enabled(std::getenv("MAME_RTL_BRIDGE_LABELS"), label);
}

bool spike_state::rtl_trace_label_enabled(const char *label) const
{
	return rtl_label_filter_enabled(std::getenv("MAME_RTL_TRACE_LABELS"), label);
}

void spike_state::rtl_trace_unmodeled(const char *op, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label)
{
	rtl_check_pc_trap(op, label, addr, size, data, mem_mask);
	if (!rtl_trace_label_enabled(label))
		return;
	const u32 pc = u32(m_maincpu->pc());
	const u32 cpu_cycles = u32(m_maincpu->total_cycles());
	m_rtl_trace.unmodeled(op, label, addr, size, data, mem_mask, pc, cpu_cycles);
	if (std::getenv("MAME_RTL_FAIL_ON_UNMODELED"))
		fatalerror("MAME unmodeled MMIO %s %s addr=%08x size=%u pc=%08x\n",
			label, op, addr, unsigned(size), pc);
}

bool spike_state::rtl_require_label_enabled(const char *label) const
{
	const char *filter = std::getenv("MAME_RTL_REQUIRE_LABELS");
	return filter && filter[0] && rtl_label_filter_enabled(filter, label);
}

bool spike_state::rtl_label_filter_enabled(const char *filter, const char *label) const
{
	if (!filter || !filter[0] || std::strcmp(filter, "all") == 0)
		return true;
	if (std::strcmp(filter, "display") == 0)
		return std::strcmp(label, "DAFB") == 0 || std::strcmp(label, "VRAM") == 0;
	if (std::strcmp(filter, "platform") == 0)
		return std::strcmp(label, "VRAM") != 0;

	const size_t label_len = std::strlen(label);
	const char *p = filter;
	while (*p)
	{
		while (*p == ',' || *p == ' ' || *p == '\t')
			p++;
		const char *start = p;
		while (*p && *p != ',')
			p++;
		const char *end = p;
		while (end > start && (end[-1] == ' ' || end[-1] == '\t'))
			end--;
		if (size_t(end - start) == label_len && std::strncmp(start, label, label_len) == 0)
			return true;
	}
	return false;
}

bool spike_state::rtl_pc_trap_enabled(u32 pc) const
{
	const char *filter = std::getenv("MAME_RTL_PC_TRAPS");
	if (!filter || !filter[0])
		return false;
	const char *p = filter;
	while (*p)
	{
		while (*p == ',' || *p == ' ' || *p == '\t')
			p++;
		if (!*p)
			break;
		char *end = nullptr;
		const unsigned long value = std::strtoul(p, &end, 0);
		if (end != p && u32(value) == pc)
			return true;
		p = (end && end != p) ? end : (p + 1);
		while (*p && *p != ',')
			p++;
	}
	return false;
}

u32 spike_state::rtl_pc_trap_hot_count() const
{
	const char *value = std::getenv("MAME_RTL_PC_TRAP_HOT_COUNT");
	if (!value || !value[0])
		return 1;
	char *end = nullptr;
	const unsigned long count = std::strtoul(value, &end, 0);
	return (end != value && count > 0) ? u32(count) : 1;
}

void spike_state::rtl_check_pc_trap(const char *op, const char *label, u32 addr, u8 size, u32 data, u32 mem_mask)
{
	const u32 pc = u32(m_maincpu->pc());
	if (!rtl_pc_trap_enabled(pc))
		return;
	u32 &hits = m_rtl_pc_trap_hits[pc];
	hits++;
	const u32 hot_count = rtl_pc_trap_hot_count();
	if (hits != hot_count)
		return;
	const u32 cpu_cycles = u32(m_maincpu->total_cycles());
	m_rtl_trace.pc_trap(op, label, addr, size, data, mem_mask, pc, cpu_cycles, hits);
	if (std::getenv("MAME_RTL_FAIL_ON_PC_TRAP"))
		fatalerror("MAME RTL PC trap pc=%08x label=%s op=%s addr=%08x hits=%u\n",
			pc, label, op, addr, unsigned(hits));
}

u8 spike_state::rtl_mem_mask_size(u32 mem_mask) const
{
	if (mem_mask == 0 || mem_mask == 0xffffffffU)
		return 4;
	if (mem_mask == 0xff000000U || mem_mask == 0x00ff0000U ||
		mem_mask == 0x0000ff00U || mem_mask == 0x000000ffU)
		return 1;
	if (mem_mask == 0xffff0000U || mem_mask == 0x0000ffffU ||
		mem_mask == 0x00ffff00U)
		return 2;
	return 4;
}

u32 spike_state::rtl_unmodeled_io_r(offs_t offset, u32 mem_mask)
{
	const u32 addr = 0x50000000U + (u32(offset) << 2);
	rtl_trace_unmodeled("r", addr, rtl_mem_mask_size(mem_mask), 0xffffffffU, mem_mask, "UNMODELED_IO");
	return 0xffffffffU;
}

void spike_state::rtl_unmodeled_io_w(offs_t offset, u32 data, u32 mem_mask)
{
	const u32 addr = 0x50000000U + (u32(offset) << 2);
	rtl_trace_unmodeled("w", addr, rtl_mem_mask_size(mem_mask), data, mem_mask, "UNMODELED_IO");
}

u32 spike_state::rtl_unmodeled_video_r(offs_t offset, u32 mem_mask)
{
	const u32 addr = 0xf9000000U + (u32(offset) << 2);
	rtl_trace_unmodeled("r", addr, rtl_mem_mask_size(mem_mask), 0xffffffffU, mem_mask, "UNMODELED_VIDEO");
	return 0xffffffffU;
}

void spike_state::rtl_unmodeled_video_w(offs_t offset, u32 data, u32 mem_mask)
{
	const u32 addr = 0xf9000000U + (u32(offset) << 2);
	rtl_trace_unmodeled("w", addr, rtl_mem_mask_size(mem_mask), data, mem_mask, "UNMODELED_VIDEO");
}

u32 spike_state::rtl_normalize_read(u32 data, u8 size, u32 mem_mask) const
{
	if (size == 1)
		return data & 0xff;
	if (size == 2 && mem_mask == 0x0000ff00)
		return (data >> 8) & 0xff;
	if (size == 2 && mem_mask == 0x000000ff)
		return data & 0xff;
	if (size == 2 && ((data >> 8) & 0xff) == (data & 0xff))
		return data & 0xff;
	if (size == 2)
		return data & 0xffff;
	return data;
}

bool spike_state::rtl_accept_validated_timing_divergences() const
{
	const char *value = std::getenv("MAME_RTL_ACCEPT_VALIDATED_TIMING");
	return value && value[0] && std::strcmp(value, "0") != 0;
}

bool spike_state::rtl_validated_timing_divergence(u32 addr, u8 size, u32 mame_norm, u32 rtl_norm, const char *label, const char **reason) const
{
	if (!rtl_accept_validated_timing_divergences())
		return false;
	if (std::strcmp(label, "SCC") == 0)
	{
		const u32 scc_offset = addr - 0x5000c000U;
		// Channel-A RR1 bit 0 is "all sent".  The RTL SCC is intended to
		// run from real FPGA clocks, while MAME advances serial completion
		// from its emulated timer queue.  Treat only this single status-bit
		// phase difference as timing, and keep all other SCC bits fatal.
		if (scc_offset == 0x22U && size == 1 && ((mame_norm ^ rtl_norm) == 0x01))
		{
			*reason = "scc_rr1_all_sent_phase_only";
			return true;
		}
		// RR0 bit 4 is SYNC/HUNT, which reflects the external /SYNC pin
		// when the crystal oscillator is not selected.  The FPGA top ties
		// the serial SYNC inputs inactive unless real board wiring provides
		// them; MAME's RS232 environment can leave this line at the opposite
		// idle level.  Accept only that one external-line bit on SCC control
		// port reads and continue returning RTL data to the emulated CPU.
		if ((scc_offset == 0x20U || scc_offset == 0x22U) &&
			size == 1 && ((mame_norm ^ rtl_norm) == 0x10))
		{
			*reason = "scc_rr0_sync_hunt_external_line_only";
			return true;
		}
		return false;
	}
	if (std::strcmp(label, "SCSI") == 0)
	{
		const u32 scsi_offset = addr - 0x5000f000U;
		// The FPGA Q700 path enables the DAFB TurboSCSI/NCR53C96 front
		// door and reports the idle C96 status phase bit as 0x04.  MAME's
		// NCR53C96 model reports 0x00 in the same idle polls.  Accept only
		// that single status bit on register 4; ROM delay loops at this PC
		// test bit 0, and later feature probing should consume RTL data.
		if (scsi_offset == 0x40U && size == 1 && ((mame_norm ^ rtl_norm) == 0x04))
		{
			*reason = "scsi_c96_idle_status_phase_bit_rtl_only";
			return true;
		}
		return false;
	}
	if (std::strcmp(label, "VIA1") == 0)
	{
		const u32 via1_offset = addr - 0x50000000U;
		const u32 via1_diff = mame_norm ^ rtl_norm;
		// The ROM clocks RTC reads with a read-modify-write BSET on VIA1
		// ORB bit 1.  The read half can observe a one-cycle PB0 phase
		// difference between MAME's RTC callback scheduling and the RTL
		// GPIO/RTC path.  That read value is only used to form the ORB
		// write while DDRB[0] is input; the following ORB read is the
		// architecturally visible RTC data sample and remains compared.
		// Keep returning RTL data so the RMW writeback follows real 6522
		// pin-read semantics.
		if (via1_offset == 0x0000U && size == 2 && via1_diff == 0x01U)
		{
			*reason = "via1_rtc_orb_rmw_data_bit_phase_only";
			return true;
		}
		// VIA1 IFR bit 0 is CA2 (RTC CKO) and bit 1 is CA1 (Q700 chains
		// VIA2 PB7's 60.15 Hz tick into VIA1).  Their absolute phases are
		// not architecturally meaningful for MMIO lockstep.  Bits 5/6 are
		// VIA1 Timer 2/1; under the blocking RTL bridge their absolute phase
		// can also differ from MAME's timer queue even when both models are
		// behaving.
		// Bit 4 is CB1, the ADB modem shift clock.  RTL now drives a shaped
		// idle clock source, but its phase is not expected to match MAME's
		// separate PIC execution exactly.  Bit 7 is only the derived 6522
		// summary bit, so allow it to differ when the only underlying flag
		// differences are CA1/CA2/T1/T2/ADB-CB1.  CB2 data, shift-register,
		// and other bits remain fatal.
		if (via1_offset == 0x1a00U && via1_diff != 0 && ((via1_diff & ~0xf3U) == 0))
		{
			*reason = "via1_ifr_ca1_ca2_timers_adb_clock_phase_only";
			return true;
		}
		if (size == 2 &&
			(via1_offset == 0x0800U || via1_offset == 0x0a00U ||
			 via1_offset == 0x1000U || via1_offset == 0x1200U))
		{
			*reason = "via1_timer_counter_phase_only";
			return true;
		}
		return false;
	}
	if (std::strcmp(label, "VIA2") != 0)
		return false;
	const u32 via2_offset = addr - 0x50002000U;
	const u8 via2_reg = u8((via2_offset >> 9) & 0x0f);
	if (via2_reg != 0 || size != 2)
		return false;

	// VIA2 ACR bit 7 routes free-running Timer 1 onto PB7.  Accept only
	// when ACR enables that route and PB7 is the sole differing bit.
	if ((m_rtl_via2_acr & 0x80) != 0 && ((mame_norm ^ rtl_norm) == 0x80))
	{
		*reason = "via2_t1_pb7_phase_only";
		return true;
	}
	return false;
}

void spike_state::rtl_compare_read(u32 addr, u8 size, u32 mame_data, u32 rtl_data, u32 mem_mask, const char *label)
{
	if (!rtl_bridge_label_enabled(label))
		return;
	const u32 mame_norm = rtl_normalize_read(mame_data, size, mem_mask);
	const u32 rtl_norm = rtl_normalize_read(rtl_data, size, 0);
	if (mame_norm == rtl_norm)
		return;
	const char *accepted_reason = nullptr;
	if (rtl_validated_timing_divergence(addr, size, mame_norm, rtl_norm, label, &accepted_reason))
	{
		m_rtl_trace.accepted_divergence(accepted_reason, label, addr, size, mame_data, rtl_data, mem_mask,
			u32(m_maincpu->pc()), u32(m_maincpu->total_cycles()));
		return;
	}
	m_rtl_trace.divergence(label, addr, size, mame_data, rtl_data, mem_mask,
		u32(m_maincpu->pc()), u32(m_maincpu->total_cycles()));
	const bool fatal = std::getenv("MAME_RTL_LOCKSTEP_FATAL") != nullptr;
	if (fatal || std::getenv("MAME_RTL_LOCKSTEP_VERBOSE"))
		std::fprintf(stderr,
			"MAME RTL lockstep read divergence %s addr=%08x size=%u mame=%08x rtl=%08x pc=%08x\n",
			label, addr, unsigned(size), mame_data, rtl_data, u32(m_maincpu->pc()));
	if (fatal)
		fatalerror("MAME RTL lockstep read divergence %s addr=%08x mame=%08x rtl=%08x\n",
			label, addr, mame_data, rtl_data);
}

void spike_state::rtl_maybe_pin_rtc()
{
	if (m_rtl_rtc_date_forced)
		return;
	m_rtl_rtc_date_forced = true;
	const char *value = std::getenv("MAME_RTL_RTC_DATE");
	if (!value || !value[0])
		return;

	int year = 0;
	int month = 0;
	int day = 0;
	int hour = 0;
	int minute = 0;
	int second = 0;
	const int fields = std::sscanf(value, "%d-%d-%dT%d:%d:%d",
		&year, &month, &day, &hour, &minute, &second);
	if (fields < 3 || year < 2000 || month < 1 || month > 12 ||
		day < 1 || day > 31 || hour < 0 || hour > 23 ||
		minute < 0 || minute > 59 || second < 0 || second > 59)
	{
		fatalerror("MAME_RTL_RTC_DATE must be YYYY-MM-DD or YYYY-MM-DDTHH:MM:SS, year >= 2000\n");
	}

	// Pinning MAME here lets lockstep runs compare against an RTL
	// +rtc_init_seconds value instead of host wall time.  MAME's Mac RTC
	// interface reduces the year to two digits internally, matching the
	// macseconds conversion path used by set_current_time().
	m_rtc->set_time(true, year, month, day, 1, hour, minute, second);
}

u16 spike_state::rtl_via1_r(offs_t offset, u16 mem_mask)
{
	rtl_maybe_pin_rtc();
	const u32 addr = 0x50000000U + (u32(offset) << 1);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		const u16 mame_data = quadrax00_state::via_r(offset);
		rtl_trace_read("mame", addr, 2, mame_data, mem_mask, "VIA1");
		if (!m_rtl_bridge.enabled())
			return mame_data;
		const u8 rtl_data = rtl_read8(addr, "VIA1");
		rtl_compare_read(addr, 2, mame_data, rtl_data, mem_mask, "VIA1");
		const char *accepted_reason = nullptr;
		const bool accepted_via1_ifr_timing = rtl_validated_timing_divergence(addr, 2,
			rtl_normalize_read(mame_data, 2, mem_mask), rtl_normalize_read(rtl_data, 1, 0),
			"VIA1", &accepted_reason);
		// MAME schedules the interrupt entry; the blocking RTL bridge only
		// advances RTL time on MMIO.  For validated VIA1 IFR timing reads,
		// compare/log RTL but let the MAME-timed interrupt handler consume
		// the MAME cause byte so ROM timer calibration makes forward progress.
		if (accepted_via1_ifr_timing && accepted_reason &&
			(std::strcmp(accepted_reason, "via1_ifr_ca1_ca2_timers_adb_clock_phase_only") == 0 ||
			 std::strcmp(accepted_reason, "via1_timer_counter_phase_only") == 0))
			return mame_data;
		return rtl_lockstep_return_mame("VIA1") ? mame_data : (u16(rtl_data) | (u16(rtl_data) << 8));
	}
	const u8 data = rtl_read8(addr, "VIA1");
	return u16(data) | (u16(data) << 8);
}

void spike_state::rtl_via1_w(offs_t offset, u16 data, u16 mem_mask)
{
	rtl_maybe_pin_rtc();
	const u32 addr = 0x50000000U + (u32(offset) << 1);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		quadrax00_state::via_w(offset, data, mem_mask);
		rtl_trace_write("mame", addr, 2, data, mem_mask, "VIA1");
		if (!m_rtl_bridge.enabled())
			return;
	}
	rtl_write16(addr, data, mem_mask, "VIA1");
}

u16 spike_state::rtl_via2_r(offs_t offset, u16 mem_mask)
{
	const u32 addr = 0x50002000U + (u32(offset) << 1);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		const u16 mame_data = quadrax00_state::via2_r(offset);
		rtl_trace_read("mame", addr, 2, mame_data, mem_mask, "VIA2");
		if (!m_rtl_bridge.enabled())
			return mame_data;
		const u8 rtl_data = rtl_read8(addr, "VIA2");
		rtl_compare_read(addr, 2, mame_data, rtl_data, mem_mask, "VIA2");
		return rtl_lockstep_return_mame("VIA2") ? mame_data : (u16(rtl_data) | (u16(rtl_data) << 8));
	}
	const u8 data = rtl_read8(addr, "VIA2");
	return u16(data) | (u16(data) << 8);
}

void spike_state::rtl_via2_w(offs_t offset, u16 data, u16 mem_mask)
{
	const u32 addr = 0x50002000U + (u32(offset) << 1);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		quadrax00_state::via2_w(offset, data, mem_mask);
		rtl_trace_write("mame", addr, 2, data, mem_mask, "VIA2");
		if ((((addr - 0x50002000U) >> 9) & 0x0f) == 0x0b)
			m_rtl_via2_acr = ACCESSING_BITS_8_15 ? u8(data >> 8) : u8(data);
		if (!m_rtl_bridge.enabled())
			return;
	}
	rtl_write16(addr, data, mem_mask, "VIA2");
}

u8 spike_state::rtl_enet_r(offs_t offset)
{
	const u32 addr = 0x50008000U + u32(offset);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		const u8 mame_data = ethernet_mac_r(offset);
		rtl_trace_read("mame", addr, 1, mame_data, 0, "ENET");
		if (!m_rtl_bridge.enabled())
			return mame_data;
		const u8 rtl_data = rtl_read8(addr, "ENET");
		rtl_compare_read(addr, 1, mame_data, rtl_data, 0, "ENET");
		return rtl_lockstep_return_mame("ENET") ? mame_data : rtl_data;
	}
	return rtl_read8(addr, "ENET");
}

u16 spike_state::rtl_sonic_r(offs_t offset, u16 mem_mask)
{
	const u32 addr = 0x5000a000U + (u32(offset) << 1);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		const u16 mame_data = m_sonic->reg_r(offset & 0x7f);
		rtl_trace_read("mame", addr, 2, mame_data, mem_mask, "SONIC");
		if (!m_rtl_bridge.enabled())
			return mame_data;
		const u16 rtl_data = rtl_read16(addr, "SONIC");
		rtl_compare_read(addr, 2, mame_data, rtl_data, mem_mask, "SONIC");
		return rtl_lockstep_return_mame("SONIC") ? mame_data : rtl_data;
	}
	return rtl_read16(addr, "SONIC");
}

void spike_state::rtl_sonic_w(offs_t offset, u16 data, u16 mem_mask)
{
	const u32 addr = 0x5000a000U + (u32(offset) << 1);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		m_sonic->reg_w(offset & 0x7f, data);
		rtl_trace_write("mame", addr, 2, data, mem_mask, "SONIC");
		if (!m_rtl_bridge.enabled())
			return;
	}
	rtl_write16(addr, data, mem_mask, "SONIC");
}

u16 spike_state::rtl_scc_r(offs_t offset, u16 mem_mask)
{
	const u32 addr = 0x5000c000U + (u32(offset) << 1);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		const u16 mame_data = quadrax00_state::scc_r(offset);
		rtl_trace_read("mame", addr, 1, mame_data >> 8, mem_mask, "SCC");
		if (!m_rtl_bridge.enabled())
			return mame_data;
		const u8 rtl_data = rtl_read8(addr, "SCC");
		rtl_compare_read(addr, 1, mame_data >> 8, rtl_data, mem_mask, "SCC");
		return rtl_lockstep_return_mame("SCC") ? mame_data : (u16(rtl_data) << 8);
	}
	return u16(rtl_read8(addr, "SCC")) << 8;
}

void spike_state::rtl_scc_w(offs_t offset, u16 data, u16 mem_mask)
{
	const u32 addr = 0x5000c000U + (u32(offset) << 1);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		if (ACCESSING_BITS_8_15)
		{
			quadrax00_state::scc_w(offset, data);
			rtl_trace_write("mame", addr, 1, data >> 8, mem_mask, "SCC");
		}
		if (!m_rtl_bridge.enabled())
			return;
	}
	if (ACCESSING_BITS_8_15)
		rtl_write8(addr, u8(data >> 8), "SCC");
}

u16 spike_state::rtl_orwell_r(offs_t offset, u16 mem_mask)
{
	const u32 addr = 0x5000e000U + (u32(offset) << 1);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		const u16 mame_data = 0;
		rtl_trace_read("mame", addr, 2, mame_data, mem_mask, "ORWELL");
		if (!m_rtl_bridge.enabled())
			return mame_data;
		const u16 rtl_data = rtl_read16(addr, "ORWELL");
		rtl_compare_read(addr, 2, mame_data, rtl_data, mem_mask, "ORWELL");
		return rtl_lockstep_return_mame("ORWELL") ? mame_data : rtl_data;
	}
	return rtl_read16(addr, "ORWELL");
}

void spike_state::rtl_orwell_w(offs_t offset, u16 data, u16 mem_mask)
{
	const u32 addr = 0x5000e000U + (u32(offset) << 1);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		rtl_trace_write("mame", addr, 2, data, mem_mask, "ORWELL");
		if (!m_rtl_bridge.enabled())
			return;
	}
	rtl_write16(addr, data, mem_mask, "ORWELL");
}

u8 spike_state::rtl_scsi_r(offs_t offset)
{
	const u32 addr = 0x5000f000U + u32(offset);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		const u8 mame_data = m_dafb->turboscsi_r<0>(offset);
		rtl_trace_read("mame", addr, 1, mame_data, 0, "SCSI");
		if (!m_rtl_bridge.enabled())
			return mame_data;
		const u8 rtl_data = rtl_read8(addr, "SCSI");
		rtl_compare_read(addr, 1, mame_data, rtl_data, 0, "SCSI");
		return rtl_lockstep_return_mame("SCSI") ? mame_data : rtl_data;
	}
	return rtl_read8(addr, "SCSI");
}

void spike_state::rtl_scsi_w(offs_t offset, u8 data)
{
	const u32 addr = 0x5000f000U + u32(offset);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		m_dafb->turboscsi_w<0>(offset, data);
		rtl_trace_write("mame", addr, 1, data, 0, "SCSI");
		if (!m_rtl_bridge.enabled())
			return;
	}
	rtl_write8(addr, data, "SCSI");
}

u16 spike_state::rtl_scsi_dma_r(offs_t offset, u16 mem_mask)
{
	const u32 addr = 0x5000f100U + (u32(offset) << 1);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		const u16 mame_data = m_dafb->turboscsi_dma_r<0>(offset, mem_mask);
		rtl_trace_read("mame", addr, 2, mame_data, mem_mask, "SCSI DMA");
		if (!m_rtl_bridge.enabled())
			return mame_data;
		const u16 rtl_data = rtl_read16(addr, "SCSI DMA");
		rtl_compare_read(addr, 2, mame_data, rtl_data, mem_mask, "SCSI DMA");
		return rtl_lockstep_return_mame("SCSI DMA") ? mame_data : rtl_data;
	}
	return rtl_read16(addr, "SCSI DMA");
}

void spike_state::rtl_scsi_dma_w(offs_t offset, u16 data, u16 mem_mask)
{
	const u32 addr = 0x5000f100U + (u32(offset) << 1);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		m_dafb->turboscsi_dma_w<0>(offset, data, mem_mask);
		rtl_trace_write("mame", addr, 2, data, mem_mask, "SCSI DMA");
		if (!m_rtl_bridge.enabled())
			return;
	}
	rtl_write16(addr, data, mem_mask, "SCSI DMA");
}

u8 spike_state::rtl_asc_r(offs_t offset)
{
	const u32 addr = 0x50014000U + u32(offset);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		const u8 mame_data = m_easc->read(offset);
		rtl_trace_read("mame", addr, 1, mame_data, 0, "ASC");
		if (!m_rtl_bridge.enabled())
			return mame_data;
		const u8 rtl_data = rtl_read8(addr, "ASC");
		rtl_compare_read(addr, 1, mame_data, rtl_data, 0, "ASC");
		return rtl_lockstep_return_mame("ASC") ? mame_data : rtl_data;
	}
	return rtl_read8(addr, "ASC");
}

void spike_state::rtl_asc_w(offs_t offset, u8 data)
{
	const u32 addr = 0x50014000U + u32(offset);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		m_easc->write(offset, data);
		rtl_trace_write("mame", addr, 1, data, 0, "ASC");
		if (!m_rtl_bridge.enabled())
			return;
	}
	rtl_write8(addr, data, "ASC");
}

u16 spike_state::rtl_swim_r(offs_t offset, u16 mem_mask)
{
	const u32 addr = 0x5001e000U + (u32(offset) << 1);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		const u16 mame_data = quadrax00_state::swim_r(offset, mem_mask);
		rtl_trace_read("mame", addr, 1, mame_data >> 8, mem_mask, "SWIM");
		if (!m_rtl_bridge.enabled())
			return mame_data;
		const u8 rtl_data = rtl_read8(addr, "SWIM");
		rtl_compare_read(addr, 1, mame_data >> 8, rtl_data, mem_mask, "SWIM");
		return rtl_lockstep_return_mame("SWIM") ? mame_data : (u16(rtl_data) << 8);
	}
	return u16(rtl_read8(addr, "SWIM")) << 8;
}

void spike_state::rtl_swim_w(offs_t offset, u16 data, u16 mem_mask)
{
	const u32 addr = 0x5001e000U + (u32(offset) << 1);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		quadrax00_state::swim_w(offset, data, mem_mask);
		if (ACCESSING_BITS_8_15)
			rtl_trace_write("mame", addr, 1, data >> 8, mem_mask, "SWIM");
		if (!m_rtl_bridge.enabled())
			return;
	}
	if (ACCESSING_BITS_8_15)
		rtl_write8(addr, u8(data >> 8), "SWIM");
}

u32 spike_state::rtl_vram_r(offs_t offset, u32 mem_mask)
{
	const u32 addr = 0xf9000000U + (u32(offset) << 2);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		const u32 mame_data = m_dafb->vram_r(offset);
		rtl_trace_read("mame", addr, 4, mame_data, mem_mask, "VRAM");
		if (!m_rtl_bridge.enabled())
			return mame_data;
		const u32 rtl_data = rtl_read32(addr, "VRAM");
		rtl_compare_read(addr, 4, mame_data, rtl_data, mem_mask, "VRAM");
		return rtl_lockstep_return_mame("VRAM") ? mame_data : rtl_data;
	}
	return rtl_read32(addr, "VRAM");
}

void spike_state::rtl_vram_w(offs_t offset, u32 data, u32 mem_mask)
{
	const u32 addr = 0xf9000000U + (u32(offset) << 2);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		m_dafb->vram_w(offset, data, mem_mask);
		rtl_trace_write("mame", addr, 4, data, mem_mask, "VRAM");
		if (!m_rtl_bridge.enabled())
			return;
	}
	rtl_write32(addr, data, mem_mask, "VRAM");
}

u32 spike_state::rtl_dafb_r(offs_t offset, u32 mem_mask)
{
	const u32 addr = 0xf9800000U + (u32(offset) << 2);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		const u32 byte_offset = u32(offset) << 2;
		u32 mame_data = 0;
		if (byte_offset < 0x100)
			mame_data = m_dafb->dafb_r(offset);
		else if (byte_offset < 0x200)
			mame_data = m_dafb->swatch_r(offset - 0x40);
		else if (byte_offset < 0x300)
			mame_data = m_dafb->ramdac_r(offset - 0x80);
		else
			mame_data = m_dafb->clockgen_r(byte_offset - 0x300);
		rtl_trace_read("mame", addr, 4, mame_data, mem_mask, "DAFB");
		if (!m_rtl_bridge.enabled())
			return mame_data;
		const u32 rtl_data = rtl_read32(addr, "DAFB");
		rtl_compare_read(addr, 4, mame_data, rtl_data, mem_mask, "DAFB");
		return rtl_lockstep_return_mame("DAFB") ? mame_data : rtl_data;
	}
	return rtl_read32(addr, "DAFB");
}

void spike_state::rtl_dafb_w(offs_t offset, u32 data, u32 mem_mask)
{
	const u32 addr = 0xf9800000U + (u32(offset) << 2);
	if (!m_rtl_bridge.enabled() || rtl_lockstep_enabled())
	{
		const u32 byte_offset = u32(offset) << 2;
		if (byte_offset < 0x100)
			m_dafb->dafb_w(offset, data);
		else if (byte_offset < 0x200)
			m_dafb->swatch_w(offset - 0x40, data);
		else if (byte_offset < 0x300)
			m_dafb->ramdac_w(offset - 0x80, data);
		else
			m_dafb->clockgen_w(byte_offset - 0x300, data);
		rtl_trace_write("mame", addr, 4, data, mem_mask, "DAFB");
		if (!m_rtl_bridge.enabled())
			return;
	}
	rtl_write32(addr, data, mem_mask, "DAFB");
}
'''


def replace_once(text: str, old: str, new: str) -> str:
    if old not in text:
        raise SystemExit(f"expected MAME source fragment not found:\n{old}")
    return text.replace(old, new, 1)


def patch_source(src: str, env_name: str) -> str:
    src = replace_once(
        src,
        '#include "formats/ap_dsk35.h"\n',
        '#include "formats/ap_dsk35.h"\n\n#include "rtl_bridge_socket.h"\n#include <cstdio>\n#include <unordered_map>\n',
    )
    src = replace_once(
        src,
        '\t\tm_adbmodem(*this, "adbmodem"),\n\t\tm_rtc(*this,"rtc")\n',
        f'\t\tm_adbmodem(*this, "adbmodem"),\n\t\tm_rtc(*this,"rtc"),\n\t\tm_rtl_bridge("{env_name}"),\n\t\tm_rtl_trace("MAME_RTL_MMIO_TRACE")\n',
    )
    src = replace_once(
        src,
        "\tvoid via_out_a(u8 data);\n\tvoid via_out_b(u8 data);\n\n\t"
        "required_device<adbmodem_device> m_adbmodem;\n\t"
        "required_device<rtc3430042_device> m_rtc;\n",
        "\tvoid via_out_a(u8 data);\n\tvoid via_out_b(u8 data);\n\t"
        "u8 rtl_read8(u32 addr, const char *label);\n\t"
        "void rtl_write8(u32 addr, u8 data, const char *label);\n\t"
        "u16 rtl_read16(u32 addr, const char *label);\n\t"
        "void rtl_write16(u32 addr, u16 data, u16 mem_mask, const char *label);\n\t"
        "u32 rtl_read32(u32 addr, const char *label);\n\t"
        "void rtl_write32(u32 addr, u32 data, u32 mem_mask, const char *label);\n\t"
        "void rtl_trace_read(const char *mode, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label);\n\t"
        "void rtl_trace_write(const char *mode, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label);\n\t"
        "void rtl_trace_missing(const char *reason, const char *op, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label);\n\t"
        "void rtl_trace_irq_snapshot(const char *label, u32 addr);\n\t"
        "void rtl_trace_unmodeled(const char *op, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label);\n\t"
        "bool rtl_lockstep_enabled() const;\n\t"
        "bool rtl_lockstep_return_mame() const;\n\t"
        "bool rtl_lockstep_return_mame(const char *label) const;\n\t"
        "bool rtl_bridge_label_enabled(const char *label) const;\n\t"
        "bool rtl_trace_label_enabled(const char *label) const;\n\t"
        "bool rtl_require_label_enabled(const char *label) const;\n\t"
        "bool rtl_label_filter_enabled(const char *filter, const char *label) const;\n\t"
        "bool rtl_pc_trap_enabled(u32 pc) const;\n\t"
        "u32 rtl_pc_trap_hot_count() const;\n\t"
        "void rtl_check_pc_trap(const char *op, const char *label, u32 addr, u8 size, u32 data, u32 mem_mask);\n\t"
        "u8 rtl_mem_mask_size(u32 mem_mask) const;\n\t"
        "u32 rtl_unmodeled_io_r(offs_t offset, u32 mem_mask);\n\t"
        "void rtl_unmodeled_io_w(offs_t offset, u32 data, u32 mem_mask);\n\t"
        "u32 rtl_unmodeled_video_r(offs_t offset, u32 mem_mask);\n\t"
        "void rtl_unmodeled_video_w(offs_t offset, u32 data, u32 mem_mask);\n\t"
        "u32 rtl_normalize_read(u32 data, u8 size, u32 mem_mask) const;\n\t"
        "bool rtl_accept_validated_timing_divergences() const;\n\t"
        "bool rtl_validated_timing_divergence(u32 addr, u8 size, u32 mame_norm, u32 rtl_norm, const char *label, const char **reason) const;\n\t"
        "void rtl_compare_read(u32 addr, u8 size, u32 mame_data, u32 rtl_data, u32 mem_mask, const char *label);\n\t"
        "void rtl_maybe_pin_rtc();\n\t"
        "u16 rtl_via1_r(offs_t offset, u16 mem_mask);\n\t"
        "void rtl_via1_w(offs_t offset, u16 data, u16 mem_mask);\n\t"
        "u16 rtl_via2_r(offs_t offset, u16 mem_mask);\n\t"
        "void rtl_via2_w(offs_t offset, u16 data, u16 mem_mask);\n\t"
        "u8 rtl_enet_r(offs_t offset);\n\t"
        "u16 rtl_sonic_r(offs_t offset, u16 mem_mask);\n\t"
        "void rtl_sonic_w(offs_t offset, u16 data, u16 mem_mask);\n\t"
        "u16 rtl_scc_r(offs_t offset, u16 mem_mask);\n\t"
        "void rtl_scc_w(offs_t offset, u16 data, u16 mem_mask);\n\t"
        "u16 rtl_orwell_r(offs_t offset, u16 mem_mask);\n\t"
        "void rtl_orwell_w(offs_t offset, u16 data, u16 mem_mask);\n\t"
        "u8 rtl_scsi_r(offs_t offset);\n\t"
        "void rtl_scsi_w(offs_t offset, u8 data);\n\t"
        "u16 rtl_scsi_dma_r(offs_t offset, u16 mem_mask);\n\t"
        "void rtl_scsi_dma_w(offs_t offset, u16 data, u16 mem_mask);\n\t"
        "u8 rtl_asc_r(offs_t offset);\n\t"
        "void rtl_asc_w(offs_t offset, u8 data);\n\t"
        "u16 rtl_swim_r(offs_t offset, u16 mem_mask);\n\t"
        "void rtl_swim_w(offs_t offset, u16 data, u16 mem_mask);\n\t"
        "u32 rtl_vram_r(offs_t offset, u32 mem_mask);\n\t"
        "void rtl_vram_w(offs_t offset, u32 data, u32 mem_mask);\n\t"
        "u32 rtl_dafb_r(offs_t offset, u32 mem_mask);\n\t"
        "void rtl_dafb_w(offs_t offset, u32 data, u32 mem_mask);\n\n\t"
        "required_device<adbmodem_device> m_adbmodem;\n\t"
        "required_device<rtc3430042_device> m_rtc;\n\t"
        "rtl_bridge_socket m_rtl_bridge;\n\t"
        "rtl_mmio_trace m_rtl_trace;\n\t"
        "bool m_rtl_rtc_date_forced = false;\n\t"
        "u8 m_rtl_via2_acr = 0;\n\t"
        "std::unordered_map<u32, u32> m_rtl_pc_trap_hits;\n",
    )
    src = replace_once(
        src,
        "void spike_state::quadra700_map(address_map &map)\n",
        IMPL + "\nvoid spike_state::quadra700_map(address_map &map)\n",
    )
    src = replace_once(
        src,
        "\tmap(0x50000000, 0x50001fff).rw(FUNC(spike_state::via_r), FUNC(spike_state::via_w)).mirror(0x00fc0000);\n"
        "\tmap(0x50002000, 0x50003fff).rw(FUNC(spike_state::via2_r), FUNC(spike_state::via2_w)).mirror(0x00fc0000);\n"
        "\tmap(0x50008000, 0x50008007).r(FUNC(spike_state::ethernet_mac_r)).mirror(0x00fc0000);\n"
        "\tmap(0x5000a000, 0x5000b0ff).m(m_sonic, FUNC(dp83932c_device::map)).umask32(0x0000ffff).mirror(0x00fc0000);\n"
        "\t// 5000e000 = Orwell controls\n",
        "\tmap(0x50000000, 0x50ffffff).rw(FUNC(spike_state::rtl_unmodeled_io_r), FUNC(spike_state::rtl_unmodeled_io_w));\n"
        "\tmap(0x50000000, 0x50001fff).rw(FUNC(spike_state::rtl_via1_r), FUNC(spike_state::rtl_via1_w)).mirror(0x00fc0000);\n"
        "\tmap(0x50002000, 0x50003fff).rw(FUNC(spike_state::rtl_via2_r), FUNC(spike_state::rtl_via2_w)).mirror(0x00fc0000);\n"
        "\tmap(0x50008000, 0x50008007).r(FUNC(spike_state::rtl_enet_r)).mirror(0x00fc0000);\n"
        "\tmap(0x5000a000, 0x5000b0ff).rw(FUNC(spike_state::rtl_sonic_r), FUNC(spike_state::rtl_sonic_w)).mirror(0x00fc0000);\n"
        "\tmap(0x5000e000, 0x5000e0ff).rw(FUNC(spike_state::rtl_orwell_r), FUNC(spike_state::rtl_orwell_w)).mirror(0x00fc0000);\n",
    )
    src = replace_once(
        src,
        "\tmap(0x5000f000, 0x5000f0ff).rw(m_dafb, FUNC(dafb_device::turboscsi_r<0>), FUNC(dafb_device::turboscsi_w<0>)).mirror(0x00fc0000);\n"
        "\tmap(0x5000f100, 0x5000f101).rw(m_dafb, FUNC(dafb_device::turboscsi_dma_r<0>), FUNC(dafb_device::turboscsi_dma_w<0>)).select(0x00fc0000);\n",
        "\tmap(0x5000f000, 0x5000f0ff).rw(FUNC(spike_state::rtl_scsi_r), FUNC(spike_state::rtl_scsi_w)).mirror(0x00fc0000);\n"
        "\tmap(0x5000f100, 0x5000f101).rw(FUNC(spike_state::rtl_scsi_dma_r), FUNC(spike_state::rtl_scsi_dma_w)).select(0x00fc0000);\n",
    )
    src = replace_once(
        src,
        "\tmap(0x5000c000, 0x5000dfff).rw(FUNC(spike_state::scc_r), FUNC(spike_state::scc_w)).mirror(0x00fc0000);\n"
        "\tmap(0x50014000, 0x50015fff).rw(m_easc, FUNC(asc_device::read), FUNC(asc_device::write)).mirror(0x00fc0000);\n"
        "\tmap(0x5001e000, 0x5001ffff).rw(FUNC(spike_state::swim_r), FUNC(spike_state::swim_w)).mirror(0x00fc0000);\n",
        "\tmap(0x5000c000, 0x5000dfff).rw(FUNC(spike_state::rtl_scc_r), FUNC(spike_state::rtl_scc_w)).mirror(0x00fc0000);\n"
        "\tmap(0x50014000, 0x50015fff).rw(FUNC(spike_state::rtl_asc_r), FUNC(spike_state::rtl_asc_w)).mirror(0x00fc0000);\n"
        "\tmap(0x5001e000, 0x5001ffff).rw(FUNC(spike_state::rtl_swim_r), FUNC(spike_state::rtl_swim_w)).mirror(0x00fc0000);\n",
    )
    src = replace_once(
        src,
        "\tmap(0xf9000000, 0xf91fffff).rw(m_dafb, FUNC(dafb_device::vram_r), FUNC(dafb_device::vram_w));\n"
        "\tmap(0xf9800000, 0xf98003ff).m(m_dafb, FUNC(dafb_device::map));\n",
        "\tmap(0xf9000000, 0xf9ffffff).rw(FUNC(spike_state::rtl_unmodeled_video_r), FUNC(spike_state::rtl_unmodeled_video_w));\n"
        "\tmap(0xf9000000, 0xf91fffff).rw(FUNC(spike_state::rtl_vram_r), FUNC(spike_state::rtl_vram_w));\n"
        "\tmap(0xf9800000, 0xf98003ff).rw(FUNC(spike_state::rtl_dafb_r), FUNC(spike_state::rtl_dafb_w));\n",
    )
    return src


def refresh_impl(src: str, env_name: str) -> str:
    if '#include "rtl_bridge_socket.h"\n#include <cstdio>\n#include <unordered_map>\n' not in src:
        if '#include "rtl_bridge_socket.h"\n#include <unordered_map>\n' in src:
            src = src.replace(
                '#include "rtl_bridge_socket.h"\n#include <unordered_map>\n',
                '#include "rtl_bridge_socket.h"\n#include <cstdio>\n#include <unordered_map>\n',
                1,
            )
        elif '#include "rtl_bridge_socket.h"\n' in src:
            src = src.replace(
                '#include "rtl_bridge_socket.h"\n',
                '#include "rtl_bridge_socket.h"\n#include <cstdio>\n#include <unordered_map>\n',
                1,
            )
    if '#include "rtl_bridge_socket.h"\n#include <unordered_map>\n' not in src and '#include <unordered_map>\n' not in src:
        src = src.replace(
            '#include "rtl_bridge_socket.h"\n',
            '#include "rtl_bridge_socket.h"\n#include <unordered_map>\n',
            1,
        )
    start = src.find("\nu8 spike_state::rtl_read8(")
    end = src.find("\nvoid spike_state::quadra700_map(address_map &map)", start)
    if start < 0 or end < 0:
        raise SystemExit("already-patched MAME source is missing the RTL helper block")
    src = src[:start + 1] + IMPL.lstrip("\n") + src[end:]

    bridge_ctor = f'm_rtl_bridge("{env_name}")'
    if 'm_rtl_trace("MAME_RTL_MMIO_TRACE")' not in src:
        src = replace_once(src, bridge_ctor, bridge_ctor + ',\n\t\tm_rtl_trace("MAME_RTL_MMIO_TRACE")')
    if "rtl_mmio_trace m_rtl_trace;" not in src:
        src = replace_once(
            src,
            "\trtl_bridge_socket m_rtl_bridge;\n",
            "\trtl_bridge_socket m_rtl_bridge;\n\trtl_mmio_trace m_rtl_trace;\n",
        )
    src = src.replace(
        "\tbool m_rtl_rtc_epoch_forced = false;\n",
        "\tbool m_rtl_rtc_date_forced = false;\n",
    )
    if "bool m_rtl_rtc_date_forced = false;" not in src:
        src = replace_once(
            src,
            "\trtl_mmio_trace m_rtl_trace;\n",
            "\trtl_mmio_trace m_rtl_trace;\n\tbool m_rtl_rtc_date_forced = false;\n",
        )
    if "void rtl_trace_read(const char *mode" not in src:
        src = replace_once(
            src,
            "\tvoid rtl_write32(u32 addr, u32 data, u32 mem_mask, const char *label);\n\t",
            "\tvoid rtl_write32(u32 addr, u32 data, u32 mem_mask, const char *label);\n\t"
            "void rtl_trace_read(const char *mode, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label);\n\t"
            "void rtl_trace_write(const char *mode, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label);\n\t",
        )
    if "void rtl_trace_missing(const char *reason" not in src:
        src = replace_once(
            src,
            "\tvoid rtl_trace_write(const char *mode, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label);\n\t",
            "\tvoid rtl_trace_write(const char *mode, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label);\n\t"
            "void rtl_trace_missing(const char *reason, const char *op, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label);\n\t",
        )
    if "void rtl_trace_irq_snapshot(const char *label, u32 addr);" not in src:
        src = replace_once(
            src,
            "\tvoid rtl_trace_missing(const char *reason, const char *op, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label);\n\t",
            "\tvoid rtl_trace_missing(const char *reason, const char *op, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label);\n\t"
            "void rtl_trace_irq_snapshot(const char *label, u32 addr);\n\t",
        )
    src = src.replace(
        "\tu16 rtl_scsi_r(offs_t offset, u16 mem_mask);\n\t"
        "void rtl_scsi_w(offs_t offset, u16 data, u16 mem_mask);\n\t",
        "\tu8 rtl_scsi_r(offs_t offset);\n\t"
        "void rtl_scsi_w(offs_t offset, u8 data);\n\t",
    )
    if "bool rtl_lockstep_enabled() const;" not in src:
        src = replace_once(
            src,
            "\tvoid rtl_trace_write(const char *mode, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label);\n\t",
            "\tvoid rtl_trace_write(const char *mode, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label);\n\t"
            "bool rtl_lockstep_enabled() const;\n\t"
            "bool rtl_lockstep_return_mame() const;\n\t"
            "bool rtl_lockstep_return_mame(const char *label) const;\n\t"
            "bool rtl_bridge_label_enabled(const char *label) const;\n\t"
            "bool rtl_trace_label_enabled(const char *label) const;\n\t"
            "bool rtl_require_label_enabled(const char *label) const;\n\t"
            "bool rtl_label_filter_enabled(const char *filter, const char *label) const;\n\t"
            "bool rtl_pc_trap_enabled(u32 pc) const;\n\t"
            "u32 rtl_pc_trap_hot_count() const;\n\t"
            "void rtl_check_pc_trap(const char *op, const char *label, u32 addr, u8 size, u32 data, u32 mem_mask);\n\t"
            "void rtl_trace_unmodeled(const char *op, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label);\n\t"
            "u8 rtl_mem_mask_size(u32 mem_mask) const;\n\t"
            "u32 rtl_unmodeled_io_r(offs_t offset, u32 mem_mask);\n\t"
            "void rtl_unmodeled_io_w(offs_t offset, u32 data, u32 mem_mask);\n\t"
            "u32 rtl_unmodeled_video_r(offs_t offset, u32 mem_mask);\n\t"
            "void rtl_unmodeled_video_w(offs_t offset, u32 data, u32 mem_mask);\n\t"
            "u32 rtl_normalize_read(u32 data, u8 size, u32 mem_mask) const;\n\t"
            "bool rtl_accept_validated_timing_divergences() const;\n\t"
            "bool rtl_validated_timing_divergence(u32 addr, u8 size, u32 mame_norm, u32 rtl_norm, const char *label, const char **reason) const;\n\t"
            "void rtl_compare_read(u32 addr, u8 size, u32 mame_data, u32 rtl_data, u32 mem_mask, const char *label);\n\t"
            "void rtl_maybe_pin_rtc();\n\t",
        )
    if "void rtl_trace_unmodeled(const char *op" not in src:
        src = replace_once(
            src,
            "\tvoid rtl_trace_irq_snapshot(const char *label, u32 addr);\n\t",
            "\tvoid rtl_trace_irq_snapshot(const char *label, u32 addr);\n\t"
            "void rtl_trace_unmodeled(const char *op, u32 addr, u8 size, u32 data, u32 mem_mask, const char *label);\n\t",
        )
    if "bool rtl_accept_validated_timing_divergences() const;" not in src:
        src = replace_once(
            src,
            "\tu32 rtl_normalize_read(u32 data, u8 size, u32 mem_mask) const;\n\t",
            "\tu32 rtl_normalize_read(u32 data, u8 size, u32 mem_mask) const;\n\t"
            "bool rtl_accept_validated_timing_divergences() const;\n\t"
            "bool rtl_validated_timing_divergence(u32 addr, u8 size, u32 mame_norm, u32 rtl_norm, const char *label, const char **reason) const;\n\t",
        )
    if "void rtl_maybe_pin_rtc();" not in src:
        src = replace_once(
            src,
            "\tvoid rtl_compare_read(u32 addr, u8 size, u32 mame_data, u32 rtl_data, u32 mem_mask, const char *label);\n\t",
            "\tvoid rtl_compare_read(u32 addr, u8 size, u32 mame_data, u32 rtl_data, u32 mem_mask, const char *label);\n\t"
            "void rtl_maybe_pin_rtc();\n\t",
        )
    if "bool rtl_bridge_label_enabled(const char *label) const;" not in src:
        src = replace_once(
            src,
            "\tbool rtl_lockstep_return_mame() const;\n\t",
            "\tbool rtl_lockstep_return_mame() const;\n\t"
            "bool rtl_lockstep_return_mame(const char *label) const;\n\t"
            "bool rtl_bridge_label_enabled(const char *label) const;\n\t",
        )
    if "bool rtl_lockstep_return_mame(const char *label) const;" not in src:
        src = replace_once(
            src,
            "\tbool rtl_lockstep_return_mame() const;\n\t",
            "\tbool rtl_lockstep_return_mame() const;\n\t"
            "bool rtl_lockstep_return_mame(const char *label) const;\n\t",
        )
    if "bool rtl_trace_label_enabled(const char *label) const;" not in src:
        src = replace_once(
            src,
            "\tbool rtl_bridge_label_enabled(const char *label) const;\n\t",
            "\tbool rtl_bridge_label_enabled(const char *label) const;\n\t"
            "bool rtl_trace_label_enabled(const char *label) const;\n\t"
            "bool rtl_label_filter_enabled(const char *filter, const char *label) const;\n\t",
        )
    if "bool rtl_require_label_enabled(const char *label) const;" not in src:
        src = replace_once(
            src,
            "\tbool rtl_trace_label_enabled(const char *label) const;\n\t",
            "\tbool rtl_trace_label_enabled(const char *label) const;\n\t"
            "bool rtl_require_label_enabled(const char *label) const;\n\t",
        )
    if "bool rtl_pc_trap_enabled(u32 pc) const;" not in src:
        src = replace_once(
            src,
            "\tbool rtl_label_filter_enabled(const char *filter, const char *label) const;\n\t",
            "\tbool rtl_label_filter_enabled(const char *filter, const char *label) const;\n\t"
            "bool rtl_pc_trap_enabled(u32 pc) const;\n\t"
            "u32 rtl_pc_trap_hot_count() const;\n\t"
            "void rtl_check_pc_trap(const char *op, const char *label, u32 addr, u8 size, u32 data, u32 mem_mask);\n\t",
        )
    if "u8 rtl_mem_mask_size(u32 mem_mask) const;" not in src:
        src = replace_once(
            src,
            "\tu32 rtl_normalize_read(u32 data, u8 size, u32 mem_mask) const;\n\t",
            "\tu8 rtl_mem_mask_size(u32 mem_mask) const;\n\t"
            "u32 rtl_unmodeled_io_r(offs_t offset, u32 mem_mask);\n\t"
            "void rtl_unmodeled_io_w(offs_t offset, u32 data, u32 mem_mask);\n\t"
            "u32 rtl_unmodeled_video_r(offs_t offset, u32 mem_mask);\n\t"
            "void rtl_unmodeled_video_w(offs_t offset, u32 data, u32 mem_mask);\n\t"
            "u32 rtl_normalize_read(u32 data, u8 size, u32 mem_mask) const;\n\t",
        )
    if "u32 rtl_vram_r(offs_t offset, u32 mem_mask);" not in src:
        src = replace_once(
            src,
            "\tvoid rtl_swim_w(offs_t offset, u16 data, u16 mem_mask);\n\t",
            "\tvoid rtl_swim_w(offs_t offset, u16 data, u16 mem_mask);\n\t"
            "u32 rtl_vram_r(offs_t offset, u32 mem_mask);\n\t"
            "void rtl_vram_w(offs_t offset, u32 data, u32 mem_mask);\n\t",
        )
    src = src.replace("\tbool rtl_accepted_read_divergence(u32 addr, u8 size, u32 mame_norm, u32 rtl_norm, const char *label) const;\n", "")
    if "u8 m_rtl_via2_acr = 0;" not in src:
        src = replace_once(
            src,
            "\trtl_mmio_trace m_rtl_trace;\n",
            "\trtl_mmio_trace m_rtl_trace;\n\tu8 m_rtl_via2_acr = 0;\n",
        )
    src = src.replace(
        "\tu32 m_rtl_pc_trap_last_pc = 0;\n\t"
        "u32 m_rtl_pc_trap_hits = 0;\n",
        "\tstd::unordered_map<u32, u32> m_rtl_pc_trap_hits;\n",
    )
    if "std::unordered_map<u32, u32> m_rtl_pc_trap_hits;" not in src:
        src = replace_once(
            src,
            "\tu8 m_rtl_via2_acr = 0;\n",
            "\tu8 m_rtl_via2_acr = 0;\n\tstd::unordered_map<u32, u32> m_rtl_pc_trap_hits;\n",
        )
    if "FUNC(spike_state::rtl_vram_r)" not in src:
        src = replace_once(
            src,
            "\tmap(0xf9000000, 0xf91fffff).rw(m_dafb, FUNC(dafb_device::vram_r), FUNC(dafb_device::vram_w));\n",
            "\tmap(0xf9000000, 0xf91fffff).rw(FUNC(spike_state::rtl_vram_r), FUNC(spike_state::rtl_vram_w));\n",
        )
    if "FUNC(spike_state::rtl_unmodeled_io_r)" not in src:
        src = replace_once(
            src,
            "\tmap(0x50000000, 0x50001fff).rw(FUNC(spike_state::rtl_via1_r), FUNC(spike_state::rtl_via1_w)).mirror(0x00fc0000);\n",
            "\tmap(0x50000000, 0x50ffffff).rw(FUNC(spike_state::rtl_unmodeled_io_r), FUNC(spike_state::rtl_unmodeled_io_w));\n"
            "\tmap(0x50000000, 0x50001fff).rw(FUNC(spike_state::rtl_via1_r), FUNC(spike_state::rtl_via1_w)).mirror(0x00fc0000);\n",
        )
    if "FUNC(spike_state::rtl_unmodeled_video_r)" not in src:
        src = replace_once(
            src,
            "\tmap(0xf9000000, 0xf91fffff).rw(FUNC(spike_state::rtl_vram_r), FUNC(spike_state::rtl_vram_w));\n",
            "\tmap(0xf9000000, 0xf9ffffff).rw(FUNC(spike_state::rtl_unmodeled_video_r), FUNC(spike_state::rtl_unmodeled_video_w));\n"
            "\tmap(0xf9000000, 0xf91fffff).rw(FUNC(spike_state::rtl_vram_r), FUNC(spike_state::rtl_vram_w));\n",
        )
    return src


def apply_overlay(mame_root: Path, env_name: str) -> None:
    apple_dir = mame_root / "src" / "mame" / "apple"
    macquadra = apple_dir / "macquadra700.cpp"
    header = apple_dir / "rtl_bridge_socket.h"
    if not macquadra.exists():
        raise SystemExit(f"missing {macquadra}; pass the MAME checkout root")

    original = macquadra.read_text(encoding="utf-8")
    already_patched = (
        '#include "rtl_bridge_socket.h"' in original
        and "rtl_bridge_socket m_rtl_bridge;" in original
        and "FUNC(spike_state::rtl_via1_r)" in original
        and "spike_state::rtl_read8" in original
    )
    patched = refresh_impl(original, env_name) if already_patched else patch_source(original, env_name)
    header.write_text(HEADER, encoding="utf-8")
    if patched != original:
        macquadra.write_text(patched, encoding="utf-8")
    print(f"wrote {header}")
    print(f"{'already patched' if already_patched else 'patched'} {macquadra}")
    print(f"run MAME with {env_name}=/tmp/mame-axi-periph-bridge.sock")


def selftest() -> None:
    fixture = '''#include "formats/ap_dsk35.h"
class spike_state : public quadrax00_state
{
public:
\tspike_state(const machine_config &mconfig, device_type type, const char *tag) :
\t\tquadrax00_state(mconfig, type, tag),
\t\tm_adbmodem(*this, "adbmodem"),
\t\tm_rtc(*this,"rtc")
\t{
\t}
private:
\tu8 via_in_a();
\tu8 via_in_b();
\tvoid via_out_a(u8 data);
\tvoid via_out_b(u8 data);

\trequired_device<adbmodem_device> m_adbmodem;
\trequired_device<rtc3430042_device> m_rtc;
};
void spike_state::quadra700_map(address_map &map)
{
\tmap(0x50000000, 0x50001fff).rw(FUNC(spike_state::via_r), FUNC(spike_state::via_w)).mirror(0x00fc0000);
\tmap(0x50002000, 0x50003fff).rw(FUNC(spike_state::via2_r), FUNC(spike_state::via2_w)).mirror(0x00fc0000);
\tmap(0x50008000, 0x50008007).r(FUNC(spike_state::ethernet_mac_r)).mirror(0x00fc0000);
\tmap(0x5000a000, 0x5000b0ff).m(m_sonic, FUNC(dp83932c_device::map)).umask32(0x0000ffff).mirror(0x00fc0000);
\t// 5000e000 = Orwell controls
\tmap(0x5000f000, 0x5000f0ff).rw(m_dafb, FUNC(dafb_device::turboscsi_r<0>), FUNC(dafb_device::turboscsi_w<0>)).mirror(0x00fc0000);
\tmap(0x5000f100, 0x5000f101).rw(m_dafb, FUNC(dafb_device::turboscsi_dma_r<0>), FUNC(dafb_device::turboscsi_dma_w<0>)).select(0x00fc0000);
\tmap(0x5000c000, 0x5000dfff).rw(FUNC(spike_state::scc_r), FUNC(spike_state::scc_w)).mirror(0x00fc0000);
\tmap(0x50014000, 0x50015fff).rw(m_easc, FUNC(asc_device::read), FUNC(asc_device::write)).mirror(0x00fc0000);
\tmap(0x5001e000, 0x5001ffff).rw(FUNC(spike_state::swim_r), FUNC(spike_state::swim_w)).mirror(0x00fc0000);

\tmap(0xf9000000, 0xf91fffff).rw(m_dafb, FUNC(dafb_device::vram_r), FUNC(dafb_device::vram_w));
\tmap(0xf9800000, 0xf98003ff).m(m_dafb, FUNC(dafb_device::map));
}
'''
    patched = patch_source(fixture, "MAME_RTL_BRIDGE_SOCKET")
    assert '#include "rtl_bridge_socket.h"' in patched
    assert "#include <unordered_map>" in patched
    assert 'm_rtl_bridge("MAME_RTL_BRIDGE_SOCKET")' in patched
    assert 'm_rtl_trace("MAME_RTL_MMIO_TRACE")' in patched
    assert "rtl_mmio_trace m_rtl_trace;" in patched
    assert "std::unordered_map<u32, u32> m_rtl_pc_trap_hits;" in patched
    assert "rtl_scsi_dma_w" in patched
    assert 'rtl_trace_read("mame"' in patched
    assert 'rtl_trace_read("rtl"' in patched
    assert "FUNC(spike_state::rtl_via1_r)" in patched
    assert "FUNC(spike_state::rtl_scc_r)" in patched
    assert "FUNC(spike_state::rtl_asc_r)" in patched
    assert "FUNC(spike_state::rtl_vram_r)" in patched
    assert "FUNC(spike_state::rtl_dafb_r)" in patched
    assert "FUNC(spike_state::rtl_scsi_r)" in patched
    assert 'std::strcmp(filter, "platform") == 0' in patched
    assert "m_dafb, FUNC(dafb_device::turboscsi_r<0>)" not in patched
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        apple = root / "src" / "mame" / "apple"
        apple.mkdir(parents=True)
        (apple / "macquadra700.cpp").write_text(fixture, encoding="utf-8")
        apply_overlay(root, "MAME_RTL_BRIDGE_SOCKET")
        assert (apple / "rtl_bridge_socket.h").read_text(encoding="utf-8").startswith("// license:BSD-3-Clause")
    print("mame_q700_rtl_overlay selftest passed")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("mame_root", nargs="?", type=Path, help="MAME checkout root to patch")
    ap.add_argument("--env-name", default="MAME_RTL_BRIDGE_SOCKET")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return 0
    if not args.mame_root:
        ap.error("mame_root is required unless --selftest is used")
    apply_overlay(args.mame_root, args.env_name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
