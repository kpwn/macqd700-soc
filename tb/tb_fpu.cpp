// tb_fpu.cpp — Verilator unit testbench for rtl/core/execute/fpu/fpu_top.v
//
// Drives fpu_top directly (no core, no ROB, no iq) and validates the
// 80-bit extended-precision result against a golden host `double`
// computation.  Since the RTL does internal math at IEEE 754 double
// precision (per task #81 spec), the comparison is bit-exact against
// `double` arithmetic.
//
// Scope (task #81 revised):
//   - FNOP    — no-op, result don't-care
//   - FMOV    — pass-through 80-bit value
//   - FABS    — clear sign bit
//   - FNEG    — invert sign bit
//   - FADD    — IEEE 754 double add
//   - FSUB    — IEEE 754 double sub
//   - FMUL    — IEEE 754 double multiply
//   - FDIV    — IEEE 754 double divide
//
// The harness packs C `double` values into 80-bit extended format,
// drives fpu_top, waits for valid_out, unpacks the 80-bit result
// back to `double`, and compares with the expected result.
//
// Build:  make tb-fpu

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <limits>
#include <verilated.h>
#include "Vfpu_top.h"

// ── Opcode constants mirrored from uop_pkg.v ───────────────────────
static const uint32_t FPU_FADD = 0;
static const uint32_t FPU_FSUB = 1;
static const uint32_t FPU_FMUL = 2;
static const uint32_t FPU_FDIV = 3;
static const uint32_t FPU_FSQRT = 4;
static const uint32_t FPU_FABS = 5;
static const uint32_t FPU_FNEG = 6;
static const uint32_t FPU_FMOV = 7;
static const uint32_t FPU_FNOP = 63;

// ── DUT and sim state ──────────────────────────────────────────────
static Vfpu_top* dut = nullptr;
static uint64_t  sim_time = 0;
static int       n_pass = 0, n_fail = 0;

// ── Reinterpret helpers ───────────────────────────────────────────
static inline uint64_t dbl_bits(double d) {
    uint64_t u;
    std::memcpy(&u, &d, sizeof(u));
    return u;
}
static inline double bits_dbl(uint64_t u) {
    double d;
    std::memcpy(&d, &u, sizeof(d));
    return d;
}

// Pack a C double → 80-bit extended (Motorola format).  Populate the
// 80-bit operand into two halves that we drive into the DUT:
//   hi = {sign, exp[14:0], J, frac[62:48]}  (top 32 bits)  — drive
//                                                           a_ext[79:48]
//   (we just construct the 80-bit blob as 3 x 32 in the DUT's port)
// Actually Verilator widens 80-bit ports to {b[79:64]=WData[2],
// b[63:32]=WData[1], b[31:0]=WData[0]} — for our port width=80 we
// need a uint32_t[3] with bits [79:0] in little-endian word order.
//
// Returns a 3-word array (WData_t-ish) — caller uses dut->a_ext[idx]=...
struct ExtBlob {
    uint32_t w[3];     // word0=bits[31:0], word1=bits[63:32], word2=bits[79:64]
};

static ExtBlob dbl_to_ext(double d) {
    ExtBlob e{};
    uint64_t ub = dbl_bits(d);
    bool     s  = (ub >> 63) & 1;
    uint32_t de = (uint32_t)((ub >> 52) & 0x7FF);
    uint64_t df = ub & 0xFFFFFFFFFFFFFULL;

    // Convert to extended fields per fpu_pack.v logic (kept in sync)
    if (de == 0 && df == 0) {
        // zero
        e.w[0] = 0;
        e.w[1] = 0;
        e.w[2] = (uint32_t)s << 15;
    } else if (de == 0x7FF && df == 0) {
        // Inf
        e.w[0] = 0;
        e.w[1] = 0x80000000u;                        // J=1 at bit 63
        e.w[2] = ((uint32_t)s << 15) | 0x7FFF;
    } else if (de == 0x7FF && df != 0) {
        // NaN (quiet)
        uint64_t frac63 = (1ULL << 62) | (df << 10);     // J=1 at bit 63 outside this half
        e.w[0] = (uint32_t)(frac63 & 0xFFFFFFFFu);
        e.w[1] = 0x80000000u | (uint32_t)((frac63 >> 32) & 0x7FFFFFFFu);
        e.w[2] = ((uint32_t)s << 15) | 0x7FFF;
    } else {
        // Normal: ext_exp = de + 15360; J=1 at bit 63; frac is df << 11
        uint16_t ext_exp = (uint16_t)(de + 15360);
        uint64_t frac63  = (df << 11);
        // Bit 63 is the J-bit; frac[62:0] = frac63
        // full 64-bit bottom = {J=1, frac63[62:0]}
        uint64_t bot64 = (1ULL << 63) | frac63;
        e.w[0] = (uint32_t)(bot64 & 0xFFFFFFFFu);
        e.w[1] = (uint32_t)((bot64 >> 32) & 0xFFFFFFFFu);
        e.w[2] = ((uint32_t)s << 15) | ext_exp;
    }
    return e;
}

// Extract the 64-bit "bottom" + sign+exp from an 80-bit ext blob and
// reconstruct it back to a double using the inverse of fpu_unpack.v.
// We use this to turn the DUT's 80-bit result back into a host double
// for comparison.
static double ext_to_dbl(const uint32_t w[3]) {
    uint64_t bot64 = ((uint64_t)w[1] << 32) | (uint64_t)w[0];
    bool     j     = (bot64 >> 63) & 1;
    uint64_t f63   = bot64 & 0x7FFFFFFFFFFFFFFFULL;
    uint32_t hi16  = w[2] & 0xFFFF;
    bool     s     = (hi16 >> 15) & 1;
    uint16_t ee    = hi16 & 0x7FFF;

    if (ee == 0 && !j && f63 == 0) {
        return s ? -0.0 : 0.0;
    }
    if (ee == 0x7FFF && f63 == 0) {
        return s ? -std::numeric_limits<double>::infinity()
                  :  std::numeric_limits<double>::infinity();
    }
    if (ee == 0x7FFF && f63 != 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    // Normal: rebias ext_exp - 15360 → dbl_exp; trunc frac63 to 52 bits
    int32_t  ext_exp = (int32_t)ee;
    int32_t  dbl_exp = ext_exp - 15360;
    if (dbl_exp <= 0) return s ? -0.0 : 0.0;
    if (dbl_exp >= 2047)
        return s ? -std::numeric_limits<double>::infinity()
                  :  std::numeric_limits<double>::infinity();
    uint64_t frac52 = (f63 >> 11) & 0xFFFFFFFFFFFFFULL;
    uint64_t ub = ((uint64_t)s << 63)
                | ((uint64_t)dbl_exp << 52)
                | frac52;
    return bits_dbl(ub);
}

// ── Clock ──────────────────────────────────────────────────────────
static void tick() {
    dut->clk = 0; dut->eval(); sim_time++;
    dut->clk = 1; dut->eval(); sim_time++;
}

static void clear_inputs() {
    dut->valid_in = 0;
    dut->fpu_op   = 0;
    dut->tag_in   = 0;
    dut->rob_tag_in = 0;
    dut->has_dst_in = 0;
    dut->a_ext[0] = 0; dut->a_ext[1] = 0; dut->a_ext[2] = 0;
    dut->b_ext[0] = 0; dut->b_ext[1] = 0; dut->b_ext[2] = 0;
}

static void reset() {
    dut->rst = 1;
    dut->flush_en = 0;
    clear_inputs();
    tick(); tick();
    dut->rst = 0;
    tick();
}

// Issue one op, wait up to N cycles for valid_out, return the 80-bit
// result as a 3-word blob (w[0]=low32, w[1]=mid32, w[2]=bits[79:64]).
struct FpuRes {
    bool     valid;
    uint32_t w[3];
};

static void drive_issue(uint32_t op, double a, double b,
                        uint32_t tag = 0x5, uint32_t rob_tag = 0x15,
                        bool has_dst = true) {
    ExtBlob ea = dbl_to_ext(a);
    ExtBlob eb = dbl_to_ext(b);

    dut->valid_in = 1;
    dut->fpu_op = op;
    dut->tag_in = tag;
    dut->rob_tag_in = rob_tag;
    dut->has_dst_in = has_dst ? 1 : 0;
    dut->a_ext[0] = ea.w[0]; dut->a_ext[1] = ea.w[1]; dut->a_ext[2] = ea.w[2];
    dut->b_ext[0] = eb.w[0]; dut->b_ext[1] = eb.w[1]; dut->b_ext[2] = eb.w[2];
}

static FpuRes fpu_issue(uint32_t op, double a, double b, int timeout = 20) {
    drive_issue(op, a, b);
    tick();
    clear_inputs();
    dut->eval();

    for (int i = 0; i < timeout; i++) {
        if (dut->valid_out) {
            FpuRes r;
            r.valid = true;
            r.w[0] = dut->result_ext[0];
            r.w[1] = dut->result_ext[1];
            r.w[2] = dut->result_ext[2];
            tick();
            return r;
        }
        tick();
    }
    FpuRes r;
    r.valid = false;
    return r;
}

// ── Assertion macros ──────────────────────────────────────────────
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("  FAIL: %s\n", msg); return false; } \
} while(0)

// Compare two doubles bit-exact (treats NaN sign-agnostically).
static bool dbl_match(double got, double exp) {
    if (std::isnan(got) && std::isnan(exp)) return true;
    return dbl_bits(got) == dbl_bits(exp);
}

static void print_mismatch(const char* name, double got, double exp) {
    printf("  FAIL %s: got %.17g (0x%016llx), expected %.17g (0x%016llx)\n",
           name, got, (unsigned long long)dbl_bits(got),
           exp, (unsigned long long)dbl_bits(exp));
}

// ── Scenario: FADD basics ─────────────────────────────────────────

static bool test_fadd_simple() {
    reset();
    FpuRes r = fpu_issue(FPU_FADD, 1.0, 2.0);
    CHECK(r.valid, "valid_out");
    double got = ext_to_dbl(r.w);
    double exp = 1.0 + 2.0;
    if (!dbl_match(got, exp)) { print_mismatch("fadd 1+2", got, exp); return false; }
    return true;
}

static bool test_fadd_exact() {
    reset();
    // 0x1.0000000000001p+0 + 1.0 = exact representable
    double a = bits_dbl(0x3FF0000000000001ULL);
    FpuRes r = fpu_issue(FPU_FADD, a, 1.0);
    CHECK(r.valid, "valid_out");
    double got = ext_to_dbl(r.w);
    double exp = a + 1.0;
    if (!dbl_match(got, exp)) { print_mismatch("fadd exact", got, exp); return false; }
    return true;
}

static bool test_fadd_neg() {
    reset();
    FpuRes r = fpu_issue(FPU_FADD, -3.5, 1.25);
    double got = ext_to_dbl(r.w);
    double exp = -3.5 + 1.25;
    if (!dbl_match(got, exp)) { print_mismatch("fadd -3.5+1.25", got, exp); return false; }
    return true;
}

static bool test_fadd_cancel_to_zero() {
    reset();
    FpuRes r = fpu_issue(FPU_FADD, 1.0, -1.0);
    double got = ext_to_dbl(r.w);
    double exp = 0.0;
    if (!dbl_match(got, exp)) { print_mismatch("fadd 1+(-1)", got, exp); return false; }
    return true;
}

static bool test_fadd_pos_zero() {
    reset();
    FpuRes r = fpu_issue(FPU_FADD, 0.0, 0.0);
    double got = ext_to_dbl(r.w);
    if (!dbl_match(got, 0.0)) { print_mismatch("fadd 0+0", got, 0.0); return false; }
    return true;
}

static bool test_fadd_neg_zero() {
    reset();
    // (-0) + (-0) = -0 per IEEE
    FpuRes r = fpu_issue(FPU_FADD, -0.0, -0.0);
    double got = ext_to_dbl(r.w);
    if (!dbl_match(got, -0.0)) { print_mismatch("fadd -0+(-0)", got, -0.0); return false; }
    return true;
}

static bool test_fadd_round_to_nearest_even() {
    reset();
    // Round-to-nearest even tie case: 1 + (2^-53) = 1 (ties to even)
    double a = 1.0;
    double b = std::ldexp(1.0, -53);   // 2^-53
    FpuRes r = fpu_issue(FPU_FADD, a, b);
    double got = ext_to_dbl(r.w);
    double exp = a + b;                // host IEEE 754 RNE
    if (!dbl_match(got, exp)) { print_mismatch("fadd RNE tie", got, exp); return false; }
    return true;
}

static bool test_fadd_inf_plus_finite() {
    reset();
    double inf = std::numeric_limits<double>::infinity();
    FpuRes r = fpu_issue(FPU_FADD, inf, 1.0);
    double got = ext_to_dbl(r.w);
    if (!dbl_match(got, inf)) { print_mismatch("fadd inf+1", got, inf); return false; }
    return true;
}

static bool test_fadd_inf_plus_neg_inf() {
    reset();
    double inf = std::numeric_limits<double>::infinity();
    FpuRes r = fpu_issue(FPU_FADD, inf, -inf);
    double got = ext_to_dbl(r.w);
    // Must be NaN
    if (!std::isnan(got)) { print_mismatch("fadd inf+(-inf)", got, std::nan("")); return false; }
    return true;
}

static bool test_fadd_nan_propagate() {
    reset();
    double nan = std::nan("");
    FpuRes r = fpu_issue(FPU_FADD, nan, 1.0);
    double got = ext_to_dbl(r.w);
    if (!std::isnan(got)) { print_mismatch("fadd nan+1", got, nan); return false; }
    return true;
}

// ── Scenario: FSUB ────────────────────────────────────────────────

static bool test_fsub_simple() {
    reset();
    FpuRes r = fpu_issue(FPU_FSUB, 5.0, 3.0);
    double got = ext_to_dbl(r.w);
    double exp = 5.0 - 3.0;
    if (!dbl_match(got, exp)) { print_mismatch("fsub 5-3", got, exp); return false; }
    return true;
}

static bool test_fsub_cancel() {
    reset();
    // Catastrophic cancellation exercise: (1 + 2^-52) - 1 = 2^-52 exact
    double a = bits_dbl(0x3FF0000000000001ULL);
    FpuRes r = fpu_issue(FPU_FSUB, a, 1.0);
    double got = ext_to_dbl(r.w);
    double exp = a - 1.0;
    if (!dbl_match(got, exp)) { print_mismatch("fsub cancel", got, exp); return false; }
    return true;
}

static bool test_fsub_inf_minus_inf() {
    reset();
    double inf = std::numeric_limits<double>::infinity();
    FpuRes r = fpu_issue(FPU_FSUB, inf, inf);
    double got = ext_to_dbl(r.w);
    if (!std::isnan(got)) { print_mismatch("fsub inf-inf", got, std::nan("")); return false; }
    return true;
}

static bool test_fsub_zero_minus_zero() {
    reset();
    // 0 - 0 = +0 per IEEE RNE
    FpuRes r = fpu_issue(FPU_FSUB, 0.0, 0.0);
    double got = ext_to_dbl(r.w);
    if (!dbl_match(got, 0.0)) { print_mismatch("fsub 0-0", got, 0.0); return false; }
    return true;
}

// ── Scenario: FMUL ────────────────────────────────────────────────

static bool test_fmul_simple() {
    reset();
    FpuRes r = fpu_issue(FPU_FMUL, 3.0, 4.0);
    double got = ext_to_dbl(r.w);
    double exp = 3.0 * 4.0;
    if (!dbl_match(got, exp)) { print_mismatch("fmul 3*4", got, exp); return false; }
    return true;
}

static bool test_fmul_frac() {
    reset();
    FpuRes r = fpu_issue(FPU_FMUL, 1.5, 2.5);
    double got = ext_to_dbl(r.w);
    double exp = 1.5 * 2.5;
    if (!dbl_match(got, exp)) { print_mismatch("fmul 1.5*2.5", got, exp); return false; }
    return true;
}

static bool test_fmul_neg() {
    reset();
    FpuRes r = fpu_issue(FPU_FMUL, -2.0, 3.5);
    double got = ext_to_dbl(r.w);
    double exp = -2.0 * 3.5;
    if (!dbl_match(got, exp)) { print_mismatch("fmul -2*3.5", got, exp); return false; }
    return true;
}

static bool test_fmul_zero() {
    reset();
    FpuRes r = fpu_issue(FPU_FMUL, 0.0, 123.456);
    double got = ext_to_dbl(r.w);
    double exp = 0.0 * 123.456;
    if (!dbl_match(got, exp)) { print_mismatch("fmul 0*x", got, exp); return false; }
    return true;
}

static bool test_fmul_inf_times_zero() {
    reset();
    double inf = std::numeric_limits<double>::infinity();
    FpuRes r = fpu_issue(FPU_FMUL, inf, 0.0);
    double got = ext_to_dbl(r.w);
    if (!std::isnan(got)) { print_mismatch("fmul inf*0", got, std::nan("")); return false; }
    return true;
}

static bool test_fmul_inf_times_finite() {
    reset();
    double inf = std::numeric_limits<double>::infinity();
    FpuRes r = fpu_issue(FPU_FMUL, inf, 2.0);
    double got = ext_to_dbl(r.w);
    double exp = inf * 2.0;
    if (!dbl_match(got, exp)) { print_mismatch("fmul inf*2", got, exp); return false; }
    return true;
}

static bool test_fmul_overflow() {
    reset();
    // 1e300 * 1e300 = +inf
    FpuRes r = fpu_issue(FPU_FMUL, 1e300, 1e300);
    double got = ext_to_dbl(r.w);
    double exp = 1e300 * 1e300;
    if (!dbl_match(got, exp)) { print_mismatch("fmul overflow", got, exp); return false; }
    return true;
}

static bool test_fmul_underflow() {
    reset();
    // 1e-300 * 1e-200 underflows to 0 (or subnormal; we flush to 0)
    FpuRes r = fpu_issue(FPU_FMUL, 1e-300, 1e-200);
    double got = ext_to_dbl(r.w);
    // Our RTL flushes subnormals to 0; host would produce subnormal
    // or zero depending on mode.  Accept 0 OR host result if host also
    // produces zero.
    double exp = 1e-300 * 1e-200;
    if (std::fabs(exp) < std::ldexp(1.0, -1022)) {
        // host produces subnormal → our FTZ result is 0
        if (!dbl_match(got, 0.0) && !dbl_match(got, -0.0)) {
            print_mismatch("fmul underflow ftz", got, 0.0);
            return false;
        }
    } else {
        if (!dbl_match(got, exp)) { print_mismatch("fmul underflow", got, exp); return false; }
    }
    return true;
}

static bool test_fmul_nan() {
    reset();
    double nan = std::nan("");
    FpuRes r = fpu_issue(FPU_FMUL, nan, 1.0);
    double got = ext_to_dbl(r.w);
    if (!std::isnan(got)) { print_mismatch("fmul nan*1", got, nan); return false; }
    return true;
}

// ── Scenario: FDIV ────────────────────────────────────────────────

static bool test_fdiv_simple() {
    reset();
    FpuRes r = fpu_issue(FPU_FDIV, 7.5, 2.5, 80);
    CHECK(r.valid, "FDIV retires");
    double got = ext_to_dbl(r.w);
    double exp = 7.5 / 2.5;
    if (!dbl_match(got, exp)) { print_mismatch("fdiv 7.5/2.5", got, exp); return false; }
    return true;
}

static bool test_fdiv_rounding() {
    reset();
    FpuRes r = fpu_issue(FPU_FDIV, 1.0, 10.0, 80);
    CHECK(r.valid, "FDIV retires");
    double got = ext_to_dbl(r.w);
    double exp = 1.0 / 10.0;
    if (!dbl_match(got, exp)) { print_mismatch("fdiv 1/10", got, exp); return false; }
    return true;
}

static bool test_fdiv_negative() {
    reset();
    FpuRes r = fpu_issue(FPU_FDIV, -9.0, 2.0, 80);
    CHECK(r.valid, "FDIV retires");
    double got = ext_to_dbl(r.w);
    double exp = -9.0 / 2.0;
    if (!dbl_match(got, exp)) { print_mismatch("fdiv -9/2", got, exp); return false; }
    return true;
}

static bool test_fdiv_divide_by_zero() {
    reset();
    FpuRes r = fpu_issue(FPU_FDIV, 3.0, 0.0, 12);
    CHECK(r.valid, "FDIV by zero retires");
    double got = ext_to_dbl(r.w);
    double exp = std::numeric_limits<double>::infinity();
    if (!dbl_match(got, exp)) { print_mismatch("fdiv 3/0", got, exp); return false; }
    return true;
}

static bool test_fdiv_zero_dividend() {
    reset();
    FpuRes r = fpu_issue(FPU_FDIV, -0.0, 2.0, 12);
    CHECK(r.valid, "FDIV zero dividend retires");
    double got = ext_to_dbl(r.w);
    if (!dbl_match(got, -0.0)) { print_mismatch("fdiv -0/2", got, -0.0); return false; }
    return true;
}

static bool test_fdiv_zero_over_zero() {
    reset();
    FpuRes r = fpu_issue(FPU_FDIV, 0.0, 0.0, 12);
    CHECK(r.valid, "FDIV 0/0 retires");
    double got = ext_to_dbl(r.w);
    if (!std::isnan(got)) { print_mismatch("fdiv 0/0", got, std::nan("")); return false; }
    return true;
}

static bool test_fdiv_inf_over_inf() {
    reset();
    double inf = std::numeric_limits<double>::infinity();
    FpuRes r = fpu_issue(FPU_FDIV, inf, inf, 12);
    CHECK(r.valid, "FDIV inf/inf retires");
    double got = ext_to_dbl(r.w);
    if (!std::isnan(got)) { print_mismatch("fdiv inf/inf", got, std::nan("")); return false; }
    return true;
}

static bool test_fdiv_backpressure_and_parallel_add() {
    reset();

    drive_issue(FPU_FDIV, 8.0, 2.0, 0x1, 0x31);
    dut->eval();
    CHECK(dut->issue_ready == 1, "FDIV initially ready");
    tick();
    clear_inputs();
    dut->eval();

    drive_issue(FPU_FDIV, 9.0, 3.0, 0x2, 0x32);
    dut->eval();
    CHECK(dut->issue_ready == 0, "second FDIV blocked while divider busy");
    clear_inputs();
    dut->eval();

    drive_issue(FPU_FADD, 1.0, 2.0, 0x3, 0x33);
    dut->eval();
    CHECK(dut->issue_ready == 1, "FADD issues while divider busy");
    tick();
    clear_inputs();
    dut->eval();

    bool saw_add = false;
    bool saw_div = false;
    int add_cycle = -1;
    int div_cycle = -1;
    for (int i = 0; i < 90; i++) {
        if (dut->valid_out) {
            uint32_t w[3] = {dut->result_ext[0], dut->result_ext[1], dut->result_ext[2]};
            double got = ext_to_dbl(w);
            if (dut->tag_out == 0x3) {
                CHECK(dbl_match(got, 3.0), "parallel FADD result");
                saw_add = true;
                add_cycle = i;
            } else if (dut->tag_out == 0x1) {
                CHECK(dbl_match(got, 4.0), "parallel FDIV result");
                saw_div = true;
                div_cycle = i;
            } else {
                CHECK(false, "unexpected FPU completion tag");
            }
        }
        tick();
    }

    CHECK(saw_add, "parallel FADD completed");
    CHECK(saw_div, "parallel FDIV completed");
    CHECK(add_cycle >= 0 && div_cycle >= 0 && add_cycle < div_cycle,
          "FADD completed before older long-latency FDIV");
    return true;
}

// ── Scenario: FMOV ────────────────────────────────────────────────

static bool test_fmov_pos() {
    reset();
    FpuRes r = fpu_issue(FPU_FMOV, 3.14159265358979, 0.0);
    double got = ext_to_dbl(r.w);
    double exp = 3.14159265358979;
    if (!dbl_match(got, exp)) { print_mismatch("fmov 3.14", got, exp); return false; }
    return true;
}

static bool test_fmov_zero() {
    reset();
    FpuRes r = fpu_issue(FPU_FMOV, 0.0, 0.0);
    double got = ext_to_dbl(r.w);
    if (!dbl_match(got, 0.0)) { print_mismatch("fmov 0", got, 0.0); return false; }
    return true;
}

static bool test_fmov_neg() {
    reset();
    FpuRes r = fpu_issue(FPU_FMOV, -12345.6789, 0.0);
    double got = ext_to_dbl(r.w);
    double exp = -12345.6789;
    if (!dbl_match(got, exp)) { print_mismatch("fmov neg", got, exp); return false; }
    return true;
}

static bool test_fabs_negative() {
    reset();
    FpuRes r = fpu_issue(FPU_FABS, -123.5, 0.0);
    CHECK(r.valid, "FABS retires");
    double got = ext_to_dbl(r.w);
    if (!dbl_match(got, 123.5)) { print_mismatch("fabs -123.5", got, 123.5); return false; }
    return true;
}

static bool test_fabs_positive() {
    reset();
    FpuRes r = fpu_issue(FPU_FABS, 98.25, 0.0);
    CHECK(r.valid, "FABS retires");
    double got = ext_to_dbl(r.w);
    if (!dbl_match(got, 98.25)) { print_mismatch("fabs +98.25", got, 98.25); return false; }
    return true;
}

static bool test_fabs_negative_zero() {
    reset();
    FpuRes r = fpu_issue(FPU_FABS, -0.0, 0.0);
    CHECK(r.valid, "FABS -0 retires");
    double got = ext_to_dbl(r.w);
    if (!dbl_match(got, 0.0)) { print_mismatch("fabs -0", got, 0.0); return false; }
    return true;
}

static bool test_fneg_positive() {
    reset();
    FpuRes r = fpu_issue(FPU_FNEG, 12.25, 0.0);
    CHECK(r.valid, "FNEG retires");
    double got = ext_to_dbl(r.w);
    if (!dbl_match(got, -12.25)) { print_mismatch("fneg +12.25", got, -12.25); return false; }
    return true;
}

static bool test_fneg_negative() {
    reset();
    FpuRes r = fpu_issue(FPU_FNEG, -44.5, 0.0);
    CHECK(r.valid, "FNEG retires");
    double got = ext_to_dbl(r.w);
    if (!dbl_match(got, 44.5)) { print_mismatch("fneg -44.5", got, 44.5); return false; }
    return true;
}

static bool test_fneg_positive_zero() {
    reset();
    FpuRes r = fpu_issue(FPU_FNEG, 0.0, 0.0);
    CHECK(r.valid, "FNEG +0 retires");
    double got = ext_to_dbl(r.w);
    if (!dbl_match(got, -0.0)) { print_mismatch("fneg +0", got, -0.0); return false; }
    return true;
}

static bool test_fnop_retires() {
    reset();
    FpuRes r = fpu_issue(FPU_FNOP, 1.0, 2.0);
    CHECK(r.valid, "FNOP retires");
    CHECK(r.w[0] == 0 && r.w[1] == 0 && r.w[2] == 0, "FNOP result is zero");
    return true;
}

static bool test_unsupported_ops_do_not_retire() {
    reset();
    FpuRes sqrt = fpu_issue(FPU_FSQRT, 4.0, 0.0, 12);
    CHECK(!sqrt.valid, "unsupported FSQRT must not retire");
    return true;
}

// ── Scenario: chain of ops (sanity) ────────────────────────────────

static bool test_chain_fma_sanity() {
    // (1.5 * 2.0) + 3.0 = 6.0 exactly
    reset();
    FpuRes m = fpu_issue(FPU_FMUL, 1.5, 2.0);
    double m_got = ext_to_dbl(m.w);
    CHECK(dbl_match(m_got, 3.0), "mul 1.5*2.0");
    FpuRes a = fpu_issue(FPU_FADD, m_got, 3.0);
    double a_got = ext_to_dbl(a.w);
    CHECK(dbl_match(a_got, 6.0), "add 3+3");
    return true;
}

static bool test_chain_mix() {
    // Four ops in sequence — stress-test the output mux ordering.
    reset();
    struct Step { uint32_t op; double a; double b; double exp; };
    Step steps[] = {
        { FPU_FADD,   1.0,    2.0,   3.0 },
        { FPU_FSUB,  10.0,    7.5,   2.5 },
        { FPU_FMUL,   2.0,    0.25,  0.5 },
        { FPU_FADD, -2.0,    2.0,    0.0 },
    };
    for (int i = 0; i < 4; i++) {
        FpuRes r = fpu_issue(steps[i].op, steps[i].a, steps[i].b);
        double got = ext_to_dbl(r.w);
        if (!dbl_match(got, steps[i].exp)) {
            print_mismatch("chain step", got, steps[i].exp);
            return false;
        }
    }
    return true;
}

// ── Additional corner-case coverage ──────────────────────────────
// (widening per docs/agent_policy.md: corners we almost got wrong)

static bool test_fadd_large_exp_diff() {
    // Exp difference of 60 — all of small's significand shifts out.
    reset();
    double a = std::ldexp(1.0, 60);   // 2^60
    double b = 1.0;
    FpuRes r = fpu_issue(FPU_FADD, a, b);
    double got = ext_to_dbl(r.w);
    double exp = a + b;
    if (!dbl_match(got, exp)) { print_mismatch("fadd big-diff", got, exp); return false; }
    return true;
}

static bool test_fadd_exp_diff_55() {
    // Exp diff exactly at the shift-clamp boundary.
    reset();
    double a = std::ldexp(1.0, 55);
    double b = 1.0;
    FpuRes r = fpu_issue(FPU_FADD, a, b);
    double got = ext_to_dbl(r.w);
    double exp = a + b;
    if (!dbl_match(got, exp)) { print_mismatch("fadd diff-55", got, exp); return false; }
    return true;
}

static bool test_fsub_leading_zero_many() {
    // 1.0 - (1 - 2^-52) = 2^-52 — massive leading-zero normalization.
    reset();
    double a = 1.0;
    double b = 1.0 - std::ldexp(1.0, -52);
    FpuRes r = fpu_issue(FPU_FSUB, a, b);
    double got = ext_to_dbl(r.w);
    double exp = a - b;
    if (!dbl_match(got, exp)) { print_mismatch("fsub lzc-many", got, exp); return false; }
    return true;
}

static bool test_fmul_tiny_by_small() {
    // Tiny normal × small normal — tests exponent subtraction path.
    reset();
    double a = 0.125;     // 2^-3
    double b = 0.25;      // 2^-2
    FpuRes r = fpu_issue(FPU_FMUL, a, b);
    double got = ext_to_dbl(r.w);
    double exp = a * b;   // = 0.03125
    if (!dbl_match(got, exp)) { print_mismatch("fmul tiny", got, exp); return false; }
    return true;
}

static bool test_fmul_round_up_carry() {
    // 1.999... * 1.5 — the round-up from mult ripples into the exponent.
    // Use values that exercise the round_carry path in fpu_mul.
    reset();
    double a = bits_dbl(0x3FFFFFFFFFFFFFFFULL);  // 1.0 - 2^-52 away from 2.0
    double b = 1.5;
    FpuRes r = fpu_issue(FPU_FMUL, a, b);
    double got = ext_to_dbl(r.w);
    double exp = a * b;
    if (!dbl_match(got, exp)) { print_mismatch("fmul round-carry", got, exp); return false; }
    return true;
}

static bool test_back_to_back_issues() {
    // Issue two FADDs back-to-back; verify the pipeline retires both
    // without stalling or collision (iq_fp will enforce this in normal
    // use, but fpu_top must tolerate it for the in-order unit tb path).
    reset();
    ExtBlob ea = dbl_to_ext(1.0), eb = dbl_to_ext(2.0);
    ExtBlob ec = dbl_to_ext(5.0), ed = dbl_to_ext(3.0);

    // Cycle 1: issue 1.0+2.0
    dut->valid_in = 1; dut->fpu_op = FPU_FADD; dut->tag_in = 0x1;
    dut->rob_tag_in = 0x11; dut->has_dst_in = 1;
    dut->a_ext[0]=ea.w[0]; dut->a_ext[1]=ea.w[1]; dut->a_ext[2]=ea.w[2];
    dut->b_ext[0]=eb.w[0]; dut->b_ext[1]=eb.w[1]; dut->b_ext[2]=eb.w[2];
    tick();
    // Cycle 2: issue 5.0+3.0
    dut->valid_in = 1; dut->fpu_op = FPU_FADD; dut->tag_in = 0x2;
    dut->rob_tag_in = 0x12; dut->has_dst_in = 1;
    dut->a_ext[0]=ec.w[0]; dut->a_ext[1]=ec.w[1]; dut->a_ext[2]=ec.w[2];
    dut->b_ext[0]=ed.w[0]; dut->b_ext[1]=ed.w[1]; dut->b_ext[2]=ed.w[2];
    tick();
    clear_inputs();
    dut->eval();

    // Collect two retires
    int collected = 0;
    double got[2] = {0, 0};
    uint32_t got_tag[2] = {0, 0};
    uint32_t got_rob_tag[2] = {0, 0};
    uint32_t got_has_dst[2] = {0, 0};
    for (int i = 0; i < 20 && collected < 2; i++) {
        if (dut->valid_out) {
            uint32_t w[3] = { dut->result_ext[0], dut->result_ext[1], dut->result_ext[2] };
            got[collected] = ext_to_dbl(w);
            got_tag[collected] = dut->tag_out;
            got_rob_tag[collected] = dut->rob_tag_out;
            got_has_dst[collected] = dut->has_dst_out;
            collected++;
        }
        tick();
    }
    CHECK(collected == 2, "two retires");
    CHECK(got_tag[0] == 0x1, "first tag");
    CHECK(got_tag[1] == 0x2, "second tag");
    CHECK(got_rob_tag[0] == 0x11, "first rob tag");
    CHECK(got_rob_tag[1] == 0x12, "second rob tag");
    CHECK(got_has_dst[0] == 1, "first has-dst");
    CHECK(got_has_dst[1] == 1, "second has-dst");
    CHECK(dbl_match(got[0], 3.0), "first sum");
    CHECK(dbl_match(got[1], 8.0), "second sum");
    return true;
}

static bool test_mixed_pipe_completion_collision() {
    reset();
    ExtBlob mul_a = dbl_to_ext(2.0), mul_b = dbl_to_ext(3.0);
    ExtBlob add_a = dbl_to_ext(1.0), add_b = dbl_to_ext(2.0);

    // FMUL at cycle N and FADD at cycle N+1 complete together
    // (mul latency 6, add latency 5). Both completions must survive
    // the single FPU writeback port.
    dut->valid_in = 1; dut->fpu_op = FPU_FMUL; dut->tag_in = 0x3;
    dut->rob_tag_in = 0x21; dut->has_dst_in = 1;
    dut->a_ext[0]=mul_a.w[0]; dut->a_ext[1]=mul_a.w[1]; dut->a_ext[2]=mul_a.w[2];
    dut->b_ext[0]=mul_b.w[0]; dut->b_ext[1]=mul_b.w[1]; dut->b_ext[2]=mul_b.w[2];
    tick();

    dut->valid_in = 1; dut->fpu_op = FPU_FADD; dut->tag_in = 0x4;
    dut->rob_tag_in = 0x22; dut->has_dst_in = 1;
    dut->a_ext[0]=add_a.w[0]; dut->a_ext[1]=add_a.w[1]; dut->a_ext[2]=add_a.w[2];
    dut->b_ext[0]=add_b.w[0]; dut->b_ext[1]=add_b.w[1]; dut->b_ext[2]=add_b.w[2];
    tick();
    clear_inputs();
    dut->eval();

    bool saw_mul = false;
    bool saw_add = false;
    for (int i = 0; i < 24 && (!saw_mul || !saw_add); i++) {
        if (dut->valid_out) {
            uint32_t w[3] = { dut->result_ext[0], dut->result_ext[1], dut->result_ext[2] };
            double got = ext_to_dbl(w);
            if (dut->tag_out == 0x3) {
                CHECK(dut->rob_tag_out == 0x21, "mul rob tag");
                CHECK(dut->has_dst_out == 1, "mul has-dst");
                CHECK(dbl_match(got, 6.0), "mul result");
                saw_mul = true;
            } else if (dut->tag_out == 0x4) {
                CHECK(dut->rob_tag_out == 0x22, "add rob tag");
                CHECK(dut->has_dst_out == 1, "add has-dst");
                CHECK(dbl_match(got, 3.0), "add result");
                saw_add = true;
            } else {
                CHECK(false, "unexpected collision tag");
            }
        }
        tick();
    }

    CHECK(saw_mul, "collision preserved mul");
    CHECK(saw_add, "collision preserved add");
    return true;
}

static bool test_flush_mid_pipe() {
    // Issue an op, then flush mid-pipeline — output must not fire.
    reset();
    ExtBlob ea = dbl_to_ext(1.5), eb = dbl_to_ext(2.5);

    dut->valid_in = 1; dut->fpu_op = FPU_FADD; dut->tag_in = 0x3;
    dut->a_ext[0]=ea.w[0]; dut->a_ext[1]=ea.w[1]; dut->a_ext[2]=ea.w[2];
    dut->b_ext[0]=eb.w[0]; dut->b_ext[1]=eb.w[1]; dut->b_ext[2]=eb.w[2];
    tick();
    clear_inputs();
    // Flush 2 cycles later (still inside the 5-stage pipe)
    tick(); tick();
    dut->flush_en = 1;
    tick();
    dut->flush_en = 0;
    // Observe next ~10 cycles — no retire must fire.
    for (int i = 0; i < 10; i++) {
        if (dut->valid_out) {
            printf("  FAIL: retire fired after flush (i=%d)\n", i);
            return false;
        }
        tick();
    }
    return true;
}

// ── Main ──────────────────────────────────────────────────────────
#define RUN(fn) do { \
    bool ok = fn(); \
    if (ok) { printf("[PASS] " #fn "\n"); n_pass++; } \
    else    { printf("[FAIL] " #fn "\n"); n_fail++; } \
} while(0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vfpu_top;

    // FADD
    RUN(test_fadd_simple);
    RUN(test_fadd_exact);
    RUN(test_fadd_neg);
    RUN(test_fadd_cancel_to_zero);
    RUN(test_fadd_pos_zero);
    RUN(test_fadd_neg_zero);
    RUN(test_fadd_round_to_nearest_even);
    RUN(test_fadd_inf_plus_finite);
    RUN(test_fadd_inf_plus_neg_inf);
    RUN(test_fadd_nan_propagate);

    // FSUB
    RUN(test_fsub_simple);
    RUN(test_fsub_cancel);
    RUN(test_fsub_inf_minus_inf);
    RUN(test_fsub_zero_minus_zero);

    // FMUL
    RUN(test_fmul_simple);
    RUN(test_fmul_frac);
    RUN(test_fmul_neg);
    RUN(test_fmul_zero);
    RUN(test_fmul_inf_times_zero);
    RUN(test_fmul_inf_times_finite);
    RUN(test_fmul_overflow);
    RUN(test_fmul_underflow);
    RUN(test_fmul_nan);

    // FDIV
    RUN(test_fdiv_simple);
    RUN(test_fdiv_rounding);
    RUN(test_fdiv_negative);
    RUN(test_fdiv_divide_by_zero);
    RUN(test_fdiv_zero_dividend);
    RUN(test_fdiv_zero_over_zero);
    RUN(test_fdiv_inf_over_inf);
    RUN(test_fdiv_backpressure_and_parallel_add);

    // FMOV
    RUN(test_fmov_pos);
    RUN(test_fmov_zero);
    RUN(test_fmov_neg);

    // FABS/FNEG
    RUN(test_fabs_negative);
    RUN(test_fabs_positive);
    RUN(test_fabs_negative_zero);
    RUN(test_fneg_positive);
    RUN(test_fneg_negative);
    RUN(test_fneg_positive_zero);

    RUN(test_fnop_retires);
    RUN(test_unsupported_ops_do_not_retire);

    // Chains
    RUN(test_chain_fma_sanity);
    RUN(test_chain_mix);

    // Corner-case widening
    RUN(test_fadd_large_exp_diff);
    RUN(test_fadd_exp_diff_55);
    RUN(test_fsub_leading_zero_many);
    RUN(test_fmul_tiny_by_small);
    RUN(test_fmul_round_up_carry);
    RUN(test_back_to_back_issues);
    RUN(test_mixed_pipe_completion_collision);
    RUN(test_flush_mid_pipe);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);

    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
