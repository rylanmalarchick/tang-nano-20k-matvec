// Verilator testbench for complex_mul (Q1.15 pipelined complex multiplier)
//
// Tests known complex multiplications and checks results against
// the same computation done in C with integer arithmetic.

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "Vcomplex_mul.h"
#include "verilated.h"

static vluint64_t sim_time = 0;

void tick(Vcomplex_mul* dut) {
    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();
    sim_time++;
}

// Convert double [-1.0, 1.0) to Q1.15
int16_t to_q15(double x) {
    int32_t v = (int32_t)round(x * 32768.0);
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

// Convert Q1.15 to double
double from_q15(int16_t x) {
    return (double)x / 32768.0;
}

// Reference complex multiply in Q1.15 (matching the RTL exactly)
void ref_complex_mul(int16_t a_re, int16_t a_im, int16_t b_re, int16_t b_im,
                     int16_t* p_re, int16_t* p_im) {
    int32_t ac = (int32_t)a_re * (int32_t)b_re;
    int32_t bd = (int32_t)a_im * (int32_t)b_im;
    int32_t ad = (int32_t)a_re * (int32_t)b_im;
    int32_t bc = (int32_t)a_im * (int32_t)b_re;
    *p_re = (int16_t)((ac - bd) >> 15);
    *p_im = (int16_t)((ad + bc) >> 15);
}

struct TestCase {
    const char* name;
    double a_re, a_im, b_re, b_im;
};

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vcomplex_mul* dut = new Vcomplex_mul;

    TestCase tests[] = {
        {"1 * 1",            0.5,    0.0,    0.5,    0.0   },
        {"pure imaginary",   0.0,    0.5,    0.0,    0.5   },  // = -0.25
        {"(1+i)*(1-i)/2",    0.5,    0.5,    0.5,   -0.5   },  // = 0.5+0i
        {"i * i = -1",       0.0,    0.5,    0.0,    0.5   },  // = -0.25+0i
        {"conjugate",        0.7,    0.3,    0.7,   -0.3   },  // = 0.58+0i
        {"small values",     0.01,   0.02,   0.03,   0.04  },
        {"near negative",   -0.9,   -0.9,   -0.9,   -0.9  },  // large magnitude
        {"mixed signs",      0.8,   -0.6,   -0.4,    0.3  },
        {"identity",         0.5,    0.0,    0.999,   0.0  },  // ~0.5
        {"zero",             0.0,    0.0,    0.5,     0.5  },
    };
    int n_tests = sizeof(tests) / sizeof(tests[0]);
    int pass = 0;

    printf("complex_mul testbench (Q1.15)\n\n");

    // Flush pipeline with idle cycles
    dut->valid_in = 0;
    dut->a_re = 0; dut->a_im = 0;
    dut->b_re = 0; dut->b_im = 0;
    for (int i = 0; i < 5; i++) tick(dut);

    for (int t = 0; t < n_tests; t++) {
        TestCase& tc = tests[t];
        int16_t ar = to_q15(tc.a_re), ai = to_q15(tc.a_im);
        int16_t br = to_q15(tc.b_re), bi = to_q15(tc.b_im);

        // Drive inputs
        dut->a_re = ar; dut->a_im = ai;
        dut->b_re = br; dut->b_im = bi;
        dut->valid_in = 1;
        tick(dut);
        dut->valid_in = 0;

        // Pipeline: 2 cycles latency (stage1 + stage2)
        tick(dut);

        // Result should be valid now
        if (!dut->valid_out) {
            printf("  [FAIL] %s: valid_out not asserted\n", tc.name);
            continue;
        }

        int16_t got_re = (int16_t)dut->p_re;
        int16_t got_im = (int16_t)dut->p_im;

        // Reference
        int16_t exp_re, exp_im;
        ref_complex_mul(ar, ai, br, bi, &exp_re, &exp_im);

        // Allow +/- 1 LSB tolerance (rounding difference between
        // arithmetic right shift in RTL vs C)
        int err_re = abs(got_re - exp_re);
        int err_im = abs(got_im - exp_im);

        if (err_re <= 1 && err_im <= 1) {
            printf("  [PASS] %-20s  (%7.4f,%7.4f)*(%7.4f,%7.4f) = (%7.4f,%7.4f)\n",
                   tc.name,
                   from_q15(ar), from_q15(ai), from_q15(br), from_q15(bi),
                   from_q15(got_re), from_q15(got_im));
            pass++;
        } else {
            printf("  [FAIL] %-20s  got (%d,%d) expected (%d,%d) err (%d,%d)\n",
                   tc.name, got_re, got_im, exp_re, exp_im, err_re, err_im);
        }

        // Drain one extra cycle so pipeline is clear
        tick(dut);
    }

    printf("\n%d/%d tests passed.\n", pass, n_tests);

    dut->final();
    delete dut;
    return (pass == n_tests) ? 0 : 1;
}
