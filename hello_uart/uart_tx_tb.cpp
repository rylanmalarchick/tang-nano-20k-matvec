// Verilator testbench for uart_tx module
// Verifies: idle state, start bit, 8 data bits (LSB first), stop bit, baud timing.

#include <cstdio>
#include <cstdlib>
#include "Vuart_tx.h"
#include "verilated.h"

// Must match the module parameters
static const int CLK_HZ      = 27000000;
static const int BAUD         = 115200;
static const int CLKS_PER_BIT = CLK_HZ / BAUD;  // 234

static vluint64_t sim_time = 0;

void tick(Vuart_tx* dut) {
    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();
    sim_time++;
}

// Advance the specified number of clock cycles
void advance(Vuart_tx* dut, int cycles) {
    for (int i = 0; i < cycles; i++)
        tick(dut);
}

// Sample TX at the middle of a bit period (most stable point)
int sample_bit(Vuart_tx* dut) {
    advance(dut, CLKS_PER_BIT / 2);
    int val = dut->tx;
    advance(dut, CLKS_PER_BIT - CLKS_PER_BIT / 2);
    return val;
}

// Send one byte and verify the serial output
bool test_byte(Vuart_tx* dut, uint8_t byte) {
    printf("  Testing byte 0x%02X ('%c')...\n", byte, (byte >= 0x20 && byte < 0x7F) ? byte : '.');

    // Should be idle
    if (dut->busy) {
        printf("    FAIL: busy should be low before start\n");
        return false;
    }
    if (dut->tx != 1) {
        printf("    FAIL: TX should be idle high\n");
        return false;
    }

    // Pulse start
    dut->data_in = byte;
    dut->start = 1;
    tick(dut);
    dut->start = 0;

    // Verify start bit (low)
    int start_bit = sample_bit(dut);
    if (start_bit != 0) {
        printf("    FAIL: start bit should be 0, got %d\n", start_bit);
        return false;
    }

    // Verify 8 data bits, LSB first
    for (int i = 0; i < 8; i++) {
        int expected = (byte >> i) & 1;
        int actual   = sample_bit(dut);
        if (actual != expected) {
            printf("    FAIL: data bit %d: expected %d, got %d\n", i, expected, actual);
            return false;
        }
    }

    // Verify stop bit (high)
    int stop_bit = sample_bit(dut);
    if (stop_bit != 1) {
        printf("    FAIL: stop bit should be 1, got %d\n", stop_bit);
        return false;
    }

    // Should return to idle
    tick(dut);  // one extra cycle for state transition
    if (dut->busy) {
        printf("    FAIL: busy should be low after stop bit\n");
        return false;
    }

    printf("    PASS\n");
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vuart_tx* dut = new Vuart_tx;

    printf("uart_tx testbench\n");
    printf("  CLK_HZ=%d, BAUD=%d, CLKS_PER_BIT=%d\n\n", CLK_HZ, BAUD, CLKS_PER_BIT);

    // Reset: a few idle cycles
    dut->start = 0;
    dut->data_in = 0;
    advance(dut, 10);

    // Test several bytes
    uint8_t test_bytes[] = {'H', 'e', 'l', 'o', 0x00, 0xFF, 0x0A};
    int n_tests = sizeof(test_bytes) / sizeof(test_bytes[0]);
    int pass = 0;

    for (int i = 0; i < n_tests; i++) {
        if (test_byte(dut, test_bytes[i]))
            pass++;
    }

    printf("\n%d/%d tests passed.\n", pass, n_tests);

    dut->final();
    delete dut;

    return (pass == n_tests) ? 0 : 1;
}
