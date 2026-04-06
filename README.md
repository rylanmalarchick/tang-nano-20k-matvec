# Tang Nano 20K Lindblad Propagator Accelerator

FPGA implementation of a 9x9 complex matrix-vector multiply for real-time Lindblad master equation simulation. Targets the Sipeed Tang Nano 20K (Gowin GW2AR-18C) using a fully open-source toolchain.

The core algorithm — repeatedly applying a precomputed propagator matrix P to a vectorized density matrix rho — is the same operation implemented in software by [lindblad-bench](https://arxiv.org/abs/2603.18052), but realized in hardware with deterministic cycle-exact timing. This makes it suitable for real-time quantum feedback control where jitter matters more than raw throughput.

## Architecture

```
                    ┌─────────────────────────────────┐
                    │         bloch_top.v              │
   27 MHz ──► rPLL ─┤  ┌────────┐    ┌──────────────┐ ├──► HDMI
              126M  │  │matvec  │    │ bloch_renderer│ │    (640x480
              ÷5    │  │_top.v  │    │    + VGA      │ │     @60Hz)
              25.2M │  │        │    │   timing      │ │
                    │  │ ┌────┐ │    └──────────────┘ │
   UART RX ─────────┤  │ │9x9 │ │                    │
   UART TX ◄────────┤  │ │core│ │◄── Bloch coords    │
                    │  │ └────┘ │    (rho00,rho01,    │
                    │  └────────┘     rho11)           │
                    └─────────────────────────────────┘
```

**Matvec core pipeline** (4 stages, 94 cycles per step):

| Stage | Operation | Hardware |
|-------|-----------|----------|
| S0 | Address generation, latch rho[col] | BSRAM address |
| S1 | Delay rho 1 cycle to match BSRAM read latency | Register |
| S2 | 4 parallel 16x16 multiplies (ac, bd, ad, bc) | 4x DSP18 |
| S3 | Accumulate (42-bit), store result when col==8 | Adder + register |

**Fixed-point format**: Q1.15 (16-bit signed, 1 sign + 15 fractional bits, range [-1, +0.99997]).

**Resource usage**: 4 DSP18 blocks (of 48), ~1300 LUTs (of 20736), P matrix in inferred BSRAM.

## Project Structure

```
├── Makefile                    # Yosys → nextpnr → gowin_pack → openFPGALoader
├── constraints/
│   └── tangnano20k.cst         # Pin assignments (LEDs, buttons, UART, HDMI)
│
├── lib/                        # Reusable modules
│   ├── uart_tx.v               # 115200 baud 8N1 transmitter
│   ├── uart_rx.v               # 115200 baud 8N1 receiver (2-FF sync)
│   └── debounce.v              # 10ms button debouncer with single-cycle pulse
│
├── blink/                      # Phase 0: sanity check
│   └── blink.v                 # Toggle 6 LEDs at ~1 Hz
│
├── hello_uart/                 # Phase 1: serial communication
│   ├── hello_uart.v            # Send "Hello\n" on button press
│   └── uart_tx_tb.cpp          # Verilator testbench (7 tests)
│
├── fixed_point/                # Phase 2: Q1.15 arithmetic
│   ├── fixed_mul.v             # Single 16x16 signed multiply → Q1.15
│   ├── complex_mul.v           # 2-stage pipelined complex multiply (4 DSP18)
│   ├── complex_mul_top.v       # On-chip test harness (4 vectors, UART output)
│   └── complex_mul_tb.cpp      # Verilator testbench (10 tests)
│
├── matvec/                     # Phase 3: core algorithm
│   ├── matvec_9x9.v            # 4-stage pipelined 9x9 complex matvec
│   ├── matvec_top.v            # UART protocol wrapper (P/V/S/N/T commands)
│   ├── matvec_pll_top.v        # PLL wrapper for 135 MHz operation
│   ├── matvec_9x9_tb.cpp       # Verilator testbench (3 tests)
│   └── host/
│       └── matvec_host.py      # Python host: load propagator, validate vs reference
│
├── hdmi/                       # Phase 4: real-time visualization
│   ├── bloch_top.v             # Top-level integration (PLL, matvec, HDMI)
│   ├── bloch_renderer.v        # Bloch sphere circle + state dot + axes
│   ├── vga_timing.v            # 640x480 @ 60 Hz timing generator
│   ├── hdmi_tx.v               # TMDS encoder + Gowin OSER10 serialization
│   └── tmds_encoder.v          # DVI 1.0 spec 8b/10b encoder with DC balance
│
└── analysis/                   # Offline analysis scripts
    ├── precision_test.py       # Q1.15 vs Q1.31 vs double (2000 steps)
    ├── benchmark_compare.py    # FPGA cycle timing vs C reference
    ├── c_jitter_test.c         # C reference benchmark
    └── plot_jitter_comparison.py
```

## Test Results

All Verilator testbenches pass:

```
$ make sim PROJECT=hello_uart SIM_TOP=uart_tx SIM_SRC="lib/uart_tx.v"
  Testing byte 0x48 ('H')...  PASS
  Testing byte 0x65 ('e')...  PASS
  Testing byte 0x6C ('l')...  PASS
  Testing byte 0x6F ('o')...  PASS
  Testing byte 0x00 ('.')...  PASS
  Testing byte 0xFF ('.')...  PASS
  Testing byte 0x0A ('.')...  PASS
7/7 tests passed.

$ make sim PROJECT=fixed_point SIM_TOP=complex_mul SIM_SRC="fixed_point/complex_mul.v fixed_point/fixed_mul.v"
  [PASS] 1 * 1                 ( 0.5000, 0.0000)*( 0.5000, 0.0000) = ( 0.2500, 0.0000)
  [PASS] pure imaginary        ( 0.0000, 0.5000)*( 0.0000, 0.5000) = (-0.2500, 0.0000)
  [PASS] (1+i)*(1-i)/2         ( 0.5000, 0.5000)*( 0.5000,-0.5000) = ( 0.5000, 0.0000)
  [PASS] i * i = -1            ( 0.0000, 0.5000)*( 0.0000, 0.5000) = (-0.2500, 0.0000)
  [PASS] conjugate             ( 0.7000, 0.3000)*( 0.7000,-0.3000) = ( 0.5800, 0.0000)
  [PASS] small values          ( 0.0100, 0.0200)*( 0.0300, 0.0400) = (-0.0005, 0.0010)
  [PASS] near negative         (-0.9000,-0.9000)*(-0.9000,-0.9000) = ( 0.0000,-0.3800)
  [PASS] mixed signs           ( 0.8000,-0.6000)*(-0.4000, 0.3000) = (-0.1400, 0.4800)
  [PASS] identity              ( 0.5000, 0.0000)*( 0.9990, 0.0000) = ( 0.4995, 0.0000)
  [PASS] zero                  ( 0.0000, 0.0000)*( 0.5000, 0.5000) = ( 0.0000, 0.0000)
10/10 tests passed.

$ make sim PROJECT=matvec SIM_TOP=matvec_9x9 SIM_SRC="matvec/matvec_9x9.v fixed_point/complex_mul.v"
  Test 1: Identity matrix...        Completed in 94 cycles.   PASS
  Test 2: Known non-trivial P...    Completed in 94 cycles.   PASS (bit-exact match with C reference)
  Test 3: Multi-step (10 steps)...  Completed in 940 cycles (94.0 cycles/step).  PASS (10-step bit-exact match)
3/3 tests passed.
```

## UART Protocol

The matvec core is controlled via serial commands at 115200 baud (8N1). All multi-byte values are little-endian.

| Command | Payload | Response | Description |
|---------|---------|----------|-------------|
| `P` | 324 bytes (81 complex Q1.15 entries) | `K` | Load propagator matrix |
| `V` | 36 bytes (9 complex Q1.15 entries) | `K` | Load state vector rho |
| `S` | none | 36 bytes (rho result) | Single propagation step |
| `N` | 2 bytes (uint16 LE step count) | 36 bytes (rho result) | N steps, single result |
| `T` | none | 4 bytes (uint32 LE) | Read hardware cycle counter |

Each complex Q1.15 entry is 4 bytes: `re_lo, re_hi, im_lo, im_hi`.

## HDMI Bloch Sphere Demo

The `hdmi/bloch_top.v` bitstream integrates everything into a self-contained demo:

- **S1 button**: Loads a Rabi rotation propagator (Ry(pi/30) per step) and starts animation
- **S2 button**: Pause/resume
- The state dot traces a great circle on the Bloch sphere at 60 fps
- UART remains active for host control simultaneously

Clock tree: 27 MHz crystal -> rPLL -> 126 MHz serial clock -> CLKDIV /5 -> 25.2 MHz pixel clock.

## Toolchain

Fully open-source, no vendor IDE required:

| Tool | Version | Purpose |
|------|---------|---------|
| [Yosys](https://github.com/YosysHQ/yosys) | 0.63+ | Synthesis |
| [nextpnr-himbaechel](https://github.com/YosysHQ/nextpnr) | 0.10+ | Place & route (Apicula database) |
| [gowin_pack](https://github.com/YosysHQ/apicula) | | Bitstream generation |
| [openFPGALoader](https://github.com/trabucayre/openFPGALoader) | | JTAG programming |
| [Verilator](https://github.com/verilator/verilator) | 5.x | Simulation & testbenches |

Install via [oss-cad-suite](https://github.com/YosysHQ/oss-cad-suite-build) for a single-archive setup.

## Quick Start

```bash
# Build and flash the blink example
make PROJECT=blink flash

# Build and flash the matvec core (UART control)
make PROJECT=matvec TOP=matvec_top flash

# Build and flash the HDMI Bloch sphere demo
make PROJECT=hdmi TOP=bloch_top flash

# Run all testbenches
make sim PROJECT=hello_uart SIM_TOP=uart_tx SIM_SRC="lib/uart_tx.v"
make sim PROJECT=fixed_point SIM_TOP=complex_mul SIM_SRC="fixed_point/complex_mul.v fixed_point/fixed_mul.v"
make sim PROJECT=matvec SIM_TOP=matvec_9x9 SIM_SRC="matvec/matvec_9x9.v fixed_point/complex_mul.v"

# Run FPGA validation against Python reference (board must be connected)
python matvec/host/matvec_host.py --port /dev/ttyUSB1 --steps 100

# Run precision analysis (no hardware needed)
python analysis/precision_test.py --steps 2000 --plot
```

## Context

This project is a learning vehicle for FPGA-based quantum simulation, built alongside [lindblad-bench](https://arxiv.org/abs/2603.18052) (a bare-metal C implementation of the same algorithm optimized for CPU cache hierarchies and SIMD). The FPGA version trades throughput for deterministic latency — every step completes in exactly 94 clock cycles with zero jitter, a property required for real-time feedback control of quantum hardware.

## Hardware

- **Board**: [Sipeed Tang Nano 20K](https://wiki.sipeed.com/hardware/en/tang/tang-nano-20k/nano-20k.html) (~$25)
- **FPGA**: Gowin GW2AR-LV18QN88C8/I7
  - 20,736 LUTs
  - 48 DSP18 blocks
  - 828 Kbit block SRAM
  - 64 Mbit SDRAM (not used)
- **Clock**: 27 MHz crystal oscillator
- **I/O**: USB-C (JTAG + UART via BL616), HDMI connector, 2 user buttons, 6 LEDs

## License

MIT
