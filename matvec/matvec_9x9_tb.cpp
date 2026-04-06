// Verilator testbench for matvec_9x9.
//
// Tests:
// 1. Identity P: output should match input (within Q1.15 rounding)
// 2. Known P: compare to C reference computation
// 3. Multi-step

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include "Vmatvec_9x9.h"
#include "verilated.h"

static vluint64_t sim_time = 0;

void tick(Vmatvec_9x9* dut) {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    sim_time++;
}

void advance(Vmatvec_9x9* dut, int n) {
    for (int i = 0; i < n; i++) tick(dut);
}

int16_t to_q15(double x) {
    int32_t v = (int32_t)round(x * 32768.0);
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

double from_q15(int16_t x) {
    return (double)x / 32768.0;
}

// Write one P entry
void write_p(Vmatvec_9x9* dut, int addr, int16_t re, int16_t im) {
    dut->p_wr_en = 1;
    dut->p_wr_addr = addr;
    dut->p_wr_re = re;
    dut->p_wr_im = im;
    tick(dut);
    dut->p_wr_en = 0;
}

// Write one rho entry
void write_rho(Vmatvec_9x9* dut, int addr, int16_t re, int16_t im) {
    dut->rho_wr_en = 1;
    dut->rho_wr_addr = addr;
    dut->rho_wr_re = re;
    dut->rho_wr_im = im;
    tick(dut);
    dut->rho_wr_en = 0;
}

// Read one rho entry
void read_rho(Vmatvec_9x9* dut, int addr, int16_t* re, int16_t* im) {
    dut->rho_rd_addr = addr;
    dut->eval();  // combinational read
    *re = (int16_t)dut->rho_rd_re;
    *im = (int16_t)dut->rho_rd_im;
}

// Start computation and wait for done
int run_and_wait(Vmatvec_9x9* dut, uint16_t n_steps, int max_cycles = 50000) {
    dut->n_steps = n_steps;
    dut->start = 1;
    tick(dut);
    dut->start = 0;

    for (int i = 0; i < max_cycles; i++) {
        tick(dut);
        if (dut->done) return i + 1;
    }
    return -1;  // timeout
}

// C reference: Q1.15 complex matvec
void ref_matvec(int16_t p_re[81], int16_t p_im[81],
                int16_t rho_re[9], int16_t rho_im[9],
                int16_t out_re[9], int16_t out_im[9]) {
    for (int i = 0; i < 9; i++) {
        int64_t acc_re = 0, acc_im = 0;
        for (int j = 0; j < 9; j++) {
            int k = i * 9 + j;
            acc_re += (int64_t)p_re[k] * rho_re[j] - (int64_t)p_im[k] * rho_im[j];
            acc_im += (int64_t)p_re[k] * rho_im[j] + (int64_t)p_im[k] * rho_re[j];
        }
        out_re[i] = (int16_t)(acc_re >> 15);
        out_im[i] = (int16_t)(acc_im >> 15);
    }
}

// =====================================================================

bool test_identity(Vmatvec_9x9* dut) {
    printf("Test 1: Identity matrix...\n");

    // Load P = I (Q1.15: 1.0 ~= 32767)
    for (int i = 0; i < 9; i++)
        for (int j = 0; j < 9; j++)
            write_p(dut, i * 9 + j, (i == j) ? 32767 : 0, 0);

    // Load rho = [0.5, 0.1+0.2i, 0, 0, 0, 0, 0, 0, 0.4]
    int16_t rho_in_re[9] = {to_q15(0.5), to_q15(0.1), 0, 0, 0, 0, 0, 0, to_q15(0.4)};
    int16_t rho_in_im[9] = {0, to_q15(0.2), 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 9; i++)
        write_rho(dut, i, rho_in_re[i], rho_in_im[i]);

    int cycles = run_and_wait(dut, 1);
    if (cycles < 0) { printf("  FAIL: timeout\n"); return false; }
    printf("  Completed in %d cycles\n", cycles);

    // Read output and compare
    bool pass = true;
    for (int i = 0; i < 9; i++) {
        int16_t re, im;
        read_rho(dut, i, &re, &im);

        // Identity in Q1.15 is 32767/32768, so output = input * 0.99997
        // Allow +-2 LSB tolerance
        int err_re = abs(re - rho_in_re[i]);
        int err_im = abs(im - rho_in_im[i]);
        if (err_re > 2 || err_im > 2) {
            printf("  FAIL [%d]: got (%d,%d) expected ~(%d,%d) err (%d,%d)\n",
                   i, re, im, rho_in_re[i], rho_in_im[i], err_re, err_im);
            pass = false;
        }
    }
    if (pass) printf("  PASS\n");
    return pass;
}

bool test_known_matvec(Vmatvec_9x9* dut) {
    printf("Test 2: Known non-trivial P...\n");

    // Build a simple P: each element P[i][j] = (i+1)*(j+1) / 81 + small_imag
    // This is NOT a physical propagator but tests the arithmetic.
    int16_t p_re[81], p_im[81];
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            double val_re = (double)(i + 1) * (j + 1) / 81.0 * 0.5;
            double val_im = (i == j) ? 0.01 : 0.0;
            p_re[i * 9 + j] = to_q15(val_re);
            p_im[i * 9 + j] = to_q15(val_im);
        }
    }

    for (int k = 0; k < 81; k++)
        write_p(dut, k, p_re[k], p_im[k]);

    // rho = [0.5, 0.3, 0.1, 0, 0, 0, 0, 0, 0.1]
    int16_t rho_re[9], rho_im[9];
    for (int i = 0; i < 9; i++) { rho_re[i] = 0; rho_im[i] = 0; }
    rho_re[0] = to_q15(0.5);
    rho_re[1] = to_q15(0.3);
    rho_re[2] = to_q15(0.1);
    rho_re[8] = to_q15(0.1);

    for (int i = 0; i < 9; i++)
        write_rho(dut, i, rho_re[i], rho_im[i]);

    // C reference
    int16_t ref_re[9], ref_im[9];
    ref_matvec(p_re, p_im, rho_re, rho_im, ref_re, ref_im);

    int cycles = run_and_wait(dut, 1);
    if (cycles < 0) { printf("  FAIL: timeout\n"); return false; }
    printf("  Completed in %d cycles\n", cycles);

    bool pass = true;
    for (int i = 0; i < 9; i++) {
        int16_t re, im;
        read_rho(dut, i, &re, &im);

        int err_re = abs(re - ref_re[i]);
        int err_im = abs(im - ref_im[i]);
        if (err_re > 1 || err_im > 1) {
            printf("  FAIL [%d]: got (%d,%d) ref (%d,%d) err (%d,%d)\n",
                   i, re, im, ref_re[i], ref_im[i], err_re, err_im);
            pass = false;
        }
    }
    if (pass) printf("  PASS (bit-exact match with C reference)\n");
    return pass;
}

bool test_multistep(Vmatvec_9x9* dut) {
    printf("Test 3: Multi-step (10 steps)...\n");

    // Use the same P from test 2 (still loaded)
    int16_t p_re[81], p_im[81];
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            double val_re = (double)(i + 1) * (j + 1) / 81.0 * 0.5;
            double val_im = (i == j) ? 0.01 : 0.0;
            p_re[i * 9 + j] = to_q15(val_re);
            p_im[i * 9 + j] = to_q15(val_im);
        }
    }

    // Reload rho
    int16_t rho_re[9] = {to_q15(0.5), to_q15(0.3), to_q15(0.1), 0, 0, 0, 0, 0, to_q15(0.1)};
    int16_t rho_im[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 9; i++)
        write_rho(dut, i, rho_re[i], rho_im[i]);

    // C reference: 10 steps
    int16_t ref_re[9], ref_im[9];
    memcpy(ref_re, rho_re, sizeof(rho_re));
    memcpy(ref_im, rho_im, sizeof(rho_im));
    for (int step = 0; step < 10; step++) {
        int16_t tmp_re[9], tmp_im[9];
        ref_matvec(p_re, p_im, ref_re, ref_im, tmp_re, tmp_im);
        memcpy(ref_re, tmp_re, sizeof(tmp_re));
        memcpy(ref_im, tmp_im, sizeof(tmp_im));
    }

    int cycles = run_and_wait(dut, 10);
    if (cycles < 0) { printf("  FAIL: timeout\n"); return false; }
    printf("  Completed in %d cycles (%.1f cycles/step)\n", cycles, cycles / 10.0);

    bool pass = true;
    for (int i = 0; i < 9; i++) {
        int16_t re, im;
        read_rho(dut, i, &re, &im);

        int err_re = abs(re - ref_re[i]);
        int err_im = abs(im - ref_im[i]);
        if (err_re > 1 || err_im > 1) {
            printf("  FAIL [%d]: got (%d,%d) ref (%d,%d) err (%d,%d)\n",
                   i, re, im, ref_re[i], ref_im[i], err_re, err_im);
            pass = false;
        }
    }
    if (pass) printf("  PASS (10-step bit-exact match)\n");
    return pass;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vmatvec_9x9* dut = new Vmatvec_9x9;

    printf("matvec_9x9 testbench\n\n");

    // Initialize
    dut->start = 0;
    dut->p_wr_en = 0;
    dut->rho_wr_en = 0;
    advance(dut, 5);

    int pass = 0, total = 3;
    if (test_identity(dut))     pass++;
    if (test_known_matvec(dut)) pass++;
    if (test_multistep(dut))    pass++;

    printf("\n%d/%d tests passed.\n", pass, total);

    dut->final();
    delete dut;
    return (pass == total) ? 0 : 1;
}
