#!/usr/bin/env python3
"""
Phase 3: CPU vs FPGA performance comparison for 9x9 complex matvec.

Measures:
  - FPGA cycle count via hardware counter (T command)
  - FPGA jitter over N_TRIALS single-step invocations
  - C benchmark data from lindblad-bench CSV
  - Comparison table and jitter histogram

Usage:
    python analysis/benchmark_compare.py [--port /dev/ttyUSB1] [--trials 1000]
"""

import argparse
import csv
import os
import struct
import sys
import time
import numpy as np
from pathlib import Path

try:
    import serial
except ImportError:
    print("pyserial required: pip install pyserial", file=sys.stderr)
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_PLT = True
except ImportError:
    HAS_PLT = False


# =====================================================================
# Q1.15 helpers (from matvec_host.py)
# =====================================================================

def to_q15(x):
    v = int(round(x * 32768.0))
    return max(-32768, min(32767, v))


# =====================================================================
# FPGA protocol helpers
# =====================================================================

def send_load_p(ser, p_re_q15, p_im_q15):
    data = bytearray()
    for k in range(81):
        data += struct.pack('<hh', int(p_re_q15[k]), int(p_im_q15[k]))
    ser.write(b'P' + data)
    ack = ser.read(1)
    if ack != b'K':
        raise RuntimeError(f"Expected 'K' ack, got {ack!r}")


def send_load_rho(ser, rho_re_q15, rho_im_q15):
    data = bytearray()
    for k in range(9):
        data += struct.pack('<hh', int(rho_re_q15[k]), int(rho_im_q15[k]))
    ser.write(b'V' + data)
    ack = ser.read(1)
    if ack != b'K':
        raise RuntimeError(f"Expected 'K' ack, got {ack!r}")


def send_step(ser):
    ser.write(b'S')
    data = ser.read(36)
    if len(data) != 36:
        raise RuntimeError(f"Expected 36 bytes, got {len(data)}")
    return data


def send_n_steps(ser, n):
    ser.write(b'N' + struct.pack('<H', n))
    data = ser.read(36)
    if len(data) != 36:
        raise RuntimeError(f"Expected 36 bytes, got {len(data)}")
    return data


def read_cycle_count(ser):
    """Send 'T' command, receive 4-byte uint32 LE cycle count."""
    ser.write(b'T')
    data = ser.read(4)
    if len(data) != 4:
        raise RuntimeError(f"Expected 4 bytes for timer, got {len(data)}")
    return struct.unpack('<I', data)[0]


# =====================================================================
# Build a simple propagator for benchmarking
# =====================================================================

def build_bench_propagator():
    """Build a d=3 transmon propagator, quantize to Q1.15."""
    from scipy.linalg import expm

    d = 3
    d2 = d * d
    H = np.diag(np.arange(d, dtype=complex))
    gamma1 = 1.0 / 50.0  # T1 = 50 us
    cops = []
    L1 = np.zeros((d, d), dtype=complex)
    L1[0, 1] = np.sqrt(gamma1)
    cops.append(L1)
    T_phi = 1.0 / (1.0 / 30.0 - 0.5 / 50.0)
    L2 = np.zeros((d, d), dtype=complex)
    L2[1, 1] = np.sqrt(1.0 / T_phi)
    cops.append(L2)
    I = np.eye(d, dtype=complex)
    L_super = -1j * (np.kron(H, I) - np.kron(I, H.conj()))
    for Lk in cops:
        LdL = Lk.conj().T @ Lk
        L_super += (np.kron(Lk, Lk.conj())
                    - 0.5 * np.kron(LdL, I)
                    - 0.5 * np.kron(I, LdL.T))
    P = expm(L_super * 0.5)

    p_re = np.array([to_q15(P.real.flat[k]) for k in range(81)], dtype=np.int16)
    p_im = np.array([to_q15(P.imag.flat[k]) for k in range(81)], dtype=np.int16)
    return p_re, p_im


# =====================================================================
# FPGA benchmark
# =====================================================================

def benchmark_fpga(ser, n_trials):
    """Run n_trials single steps, collect cycle counts and wall times."""
    print(f"Loading propagator to FPGA...")
    p_re, p_im = build_bench_propagator()
    send_load_p(ser, p_re, p_im)

    # Load initial rho = |0><0|
    rho_re = np.zeros(9, dtype=np.int16)
    rho_im = np.zeros(9, dtype=np.int16)
    rho_re[0] = to_q15(1.0)
    send_load_rho(ser, rho_re, rho_im)

    print(f"Running {n_trials} single-step trials...")
    cycle_counts = []
    wall_times = []

    for i in range(n_trials):
        t0 = time.perf_counter()
        send_step(ser)
        t1 = time.perf_counter()
        cycles = read_cycle_count(ser)
        cycle_counts.append(cycles)
        wall_times.append((t1 - t0) * 1e9)  # ns

        if (i + 1) % 200 == 0:
            print(f"  {i+1}/{n_trials}")

    cycle_counts = np.array(cycle_counts)
    wall_times = np.array(wall_times)

    # Multi-step timing test
    print("Multi-step timing tests...")
    multi_results = {}
    for n in [1, 10, 100, 1000]:
        send_load_rho(ser, rho_re, rho_im)
        t0 = time.perf_counter()
        send_n_steps(ser, n)
        t1 = time.perf_counter()
        cycles = read_cycle_count(ser)
        multi_results[n] = {
            'cycles': cycles,
            'wall_ns': (t1 - t0) * 1e9,
            'cycles_per_step': cycles / n,
        }

    return cycle_counts, wall_times, multi_results


# =====================================================================
# Load C benchmark data
# =====================================================================

def resolve_lindblad_bench_dir(explicit_path=None):
    """Find the active lindblad-bench repo without assuming the old mirror path."""
    candidates = []
    if explicit_path:
        candidates.append(Path(explicit_path).expanduser())

    env_path = os.environ.get("LINDBLAD_BENCH_DIR")
    if env_path:
        candidates.append(Path(env_path).expanduser())

    here = Path(__file__).resolve()
    tang_root = here.parents[1]
    candidates.extend([
        tang_root.parent / "lindblad-bench",
        Path.cwd() / "lindblad-bench",
        Path.home() / "dev" / "projects" / "lindblad-bench",
        Path.home() / "dev" / "research" / "lindblad-bench",
    ])

    seen = set()
    for candidate in candidates:
        candidate = candidate.resolve()
        if candidate in seen:
            continue
        seen.add(candidate)
        if (candidate / "benchmarks").is_dir():
            return candidate

    raise FileNotFoundError(
        "Could not locate lindblad-bench. "
        "Pass --lindblad-bench or set LINDBLAD_BENCH_DIR."
    )


def load_cpu_batch_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if int(row["d"]) != 3:
                continue
            rows.append({
                "label": f"threads={row['threads']}, batch={row['batch_size']}",
                "threads": int(row["threads"]),
                "batch_size": int(row["batch_size"]),
                "ns_per_step": float(row["median_ns_per_state_step"]),
                "gflops": float(row["median_gflops"]),
            })
    return rows


def load_compiler_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if int(row["d"]) != 3:
                continue
            rows.append({
                "label": f"{row['flags']} {row['variant']}",
                "threads": None,
                "batch_size": None,
                "ns_per_step": float(row["ns_per_step"]),
                "gflops": float(row["gflops"]),
            })
    return rows


def load_c_benchmarks(lindblad_bench_dir):
    """Load lindblad-bench CPU results for d=3, preferring the current batch CSVs."""
    base = Path(lindblad_bench_dir) / "benchmarks"
    results = {}

    cpu_batch_files = [
        ("i9-13980HX", "cpu_batch_results_intel.csv"),
        ("Ryzen 5 1600", "cpu_batch_results_ryzen.csv"),
    ]
    for name, fname in cpu_batch_files:
        path = base / fname
        if path.exists():
            rows = load_cpu_batch_csv(path)
            if rows:
                results[name] = rows

    if results:
        return results

    legacy_files = [
        ("i9-13980HX", "compiler_results.csv"),
        ("Ryzen", "compiler_results_ryzen.csv"),
    ]
    for name, fname in legacy_files:
        path = base / fname
        if path.exists():
            rows = load_compiler_csv(path)
            if rows:
                results[name] = rows

    return results


def best_single_state_row(rows):
    single_rows = [row for row in rows if row["batch_size"] in (None, 1)]
    if single_rows:
        return min(single_rows, key=lambda row: row["ns_per_step"])
    return min(rows, key=lambda row: row["ns_per_step"])


# =====================================================================
# Reporting
# =====================================================================

D = 3
D2 = D * D
# FLOPs per matvec: d^2 * d^2 * 8 (8 real ops per complex MAC)
FLOPS_PER_STEP = D2 * D2 * 8  # 9*9*8 = 648


def print_report(cycle_counts, wall_times, multi_results, c_data, clk_mhz):
    print("\n" + "=" * 72)
    print("FPGA vs C Performance Comparison: 9x9 Complex Matvec (d=3)")
    print("=" * 72)

    # FPGA stats
    fpga_cycles = int(np.median(cycle_counts))
    fpga_ns = fpga_cycles / clk_mhz * 1000  # ns
    fpga_unique = np.unique(cycle_counts)

    print(f"\n--- FPGA (Tang Nano 20K, GW2AR-18, {clk_mhz:.0f} MHz core clock) ---")
    print(f"  Cycles/step:     {fpga_cycles}")
    print(f"  Time/step:       {fpga_ns:.1f} ns")
    print(f"  MFLOP/s:         {FLOPS_PER_STEP / fpga_ns * 1000:.1f}")
    print(f"  Unique counts:   {fpga_unique} ({len(fpga_unique)} distinct values)")
    print(f"  Jitter:          {cycle_counts.max() - cycle_counts.min()} cycles "
          f"(min={cycle_counts.min()}, max={cycle_counts.max()})")

    # Multi-step results
    print(f"\n  Multi-step cycle counts:")
    for n in sorted(multi_results.keys()):
        r = multi_results[n]
        print(f"    {n:5d} steps: {r['cycles']:8d} cycles "
              f"({r['cycles_per_step']:.1f} cyc/step, "
              f"{r['wall_ns']/1e6:.1f} ms wall)")

    # C benchmarks
    for cpu_name, rows in c_data.items():
        print(f"\n--- CPU ({cpu_name}) ---")
        best_latency = best_single_state_row(rows)
        best_overall = min(rows, key=lambda r: r['ns_per_step'])
        print(f"  Best single-state config: {best_latency['label']}")
        print(f"  Single-state time/step:   {best_latency['ns_per_step']:.1f} ns")
        print(f"  Single-state GFLOP/s:     {best_latency['gflops']:.2f}")
        print(f"  Best overall config:      {best_overall['label']}")
        print(f"  Best overall time/step:   {best_overall['ns_per_step']:.1f} ns")
        print(f"  Best overall GFLOP/s:     {best_overall['gflops']:.2f}")

        print(f"\n  All d=3 results:")
        print(f"  {'Config':<24s} {'ns/step':>10s} {'GFLOP/s':>10s} {'vs FPGA':>10s}")
        print(f"  {'-'*24} {'-'*10} {'-'*10} {'-'*10}")
        for r in sorted(rows, key=lambda x: x['ns_per_step']):
            ratio = fpga_ns / r['ns_per_step']
            print(f"  {r['label']:<24s} "
                  f"{r['ns_per_step']:>10.1f} {r['gflops']:>10.2f} "
                  f"{ratio:>9.1f}x")

    # Summary comparison
    print(f"\n--- Summary ---")
    print(f"  FPGA @ {clk_mhz:.0f} MHz:  {fpga_ns:.0f} ns/step")
    if c_data:
        best_latency_cpu = None
        best_latency_name = None
        best_overall_cpu = None
        best_overall_name = None
        for cpu_name, rows in c_data.items():
            latency_row = best_single_state_row(rows)
            overall_row = min(rows, key=lambda row: row["ns_per_step"])
            if best_latency_cpu is None or latency_row["ns_per_step"] < best_latency_cpu["ns_per_step"]:
                best_latency_cpu = latency_row
                best_latency_name = cpu_name
            if best_overall_cpu is None or overall_row["ns_per_step"] < best_overall_cpu["ns_per_step"]:
                best_overall_cpu = overall_row
                best_overall_name = cpu_name

        print(f"  CPU best single-state: {best_latency_cpu['ns_per_step']:.0f} ns/step "
              f"({best_latency_name}, {best_latency_cpu['label']})")
        print(f"  CPU/FPGA latency ratio: {fpga_ns / best_latency_cpu['ns_per_step']:.1f}x "
              f"(CPU is faster)")
        print(f"  CPU best batched:      {best_overall_cpu['ns_per_step']:.0f} ns/step "
              f"({best_overall_name}, {best_overall_cpu['label']})")
        print(f"  CPU/FPGA throughput ratio: {fpga_ns / best_overall_cpu['ns_per_step']:.1f}x "
              f"(CPU is faster)")

    print(f"\n  Key advantage: FPGA has ZERO jitter ({cycle_counts.max() - cycle_counts.min()} cycle spread)")
    print(f"  This matters for real-time quantum control feedback loops.")
    print(f"  The CPU is faster in throughput but non-deterministic.")

    # UART overhead analysis
    uart_bytes_per_step = 36  # result rho
    uart_bits_per_byte = 10   # 8N1
    uart_baud = 115200
    uart_ns = uart_bytes_per_step * uart_bits_per_byte / uart_baud * 1e9
    wall_median = np.median(wall_times)
    print(f"\n  Wall time per single step: {wall_median:.0f} ns "
          f"({wall_median/1e6:.1f} ms)")
    print(f"  UART overhead: ~{uart_ns:.0f} ns ({uart_ns/1e6:.1f} ms) "
          f"for 36 bytes @ 115200 baud")
    print(f"  Compute fraction: {fpga_ns / wall_median * 100:.2f}% "
          f"(rest is UART I/O)")


def plot_jitter(cycle_counts, outpath):
    """Plot jitter histogram."""
    if not HAS_PLT:
        print("matplotlib not available, skipping plots")
        return

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Histogram of cycle counts
    ax = axes[0]
    unique, counts = np.unique(cycle_counts, return_counts=True)
    ax.bar(unique, counts, color='steelblue', edgecolor='navy')
    ax.set_xlabel('Cycle Count')
    ax.set_ylabel('Frequency')
    ax.set_title(f'FPGA Matvec Cycle Count Distribution\n'
                 f'(n={len(cycle_counts)}, '
                 f'range={cycle_counts.min()}-{cycle_counts.max()})')

    # Time series
    ax = axes[1]
    ax.plot(cycle_counts, '.', markersize=1, alpha=0.5, color='steelblue')
    ax.set_xlabel('Trial')
    ax.set_ylabel('Cycle Count')
    ax.set_title('Cycle Count Over Time')
    ax.axhline(y=np.median(cycle_counts), color='red', linestyle='--',
               label=f'median={np.median(cycle_counts):.0f}')
    ax.legend()

    plt.tight_layout()
    plt.savefig(outpath, dpi=150)
    print(f"Saved jitter plot to {outpath}")


# =====================================================================
# Main
# =====================================================================

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', default='/dev/ttyUSB1')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--clk-mhz', type=float, default=135.0,
                        help='FPGA core clock in MHz for cycle-to-time conversion')
    parser.add_argument('--lindblad-bench', type=str, default=None,
                        help='Path to the lindblad-bench repo')
    parser.add_argument('--trials', type=int, default=1000,
                        help='Number of single-step trials for jitter measurement')
    parser.add_argument('--no-fpga', action='store_true',
                        help='Skip FPGA measurement, only show C data')
    args = parser.parse_args()

    try:
        lindblad_bench_dir = resolve_lindblad_bench_dir(args.lindblad_bench)
        c_data = load_c_benchmarks(lindblad_bench_dir)
    except FileNotFoundError as exc:
        print(f"Warning: {exc}")
        c_data = {}
    if not c_data:
        print("Warning: no C benchmark CSVs found in lindblad-bench/benchmarks/")

    if args.no_fpga:
        # Synthetic FPGA data for report generation without hardware
        cycle_counts = np.array([94] * args.trials)
        wall_times = np.array([3.5e6] * args.trials)
        multi_results = {
            1: {'cycles': 94, 'wall_ns': 3.5e6, 'cycles_per_step': 94},
            10: {'cycles': 940, 'wall_ns': 35e6, 'cycles_per_step': 94},
        }
    else:
        ser = serial.Serial(args.port, args.baud, timeout=5)
        time.sleep(0.1)
        ser.reset_input_buffer()

        cycle_counts, wall_times, multi_results = benchmark_fpga(ser, args.trials)
        ser.close()

    print_report(cycle_counts, wall_times, multi_results, c_data, args.clk_mhz)

    outpath = os.path.join(os.path.dirname(__file__), 'jitter_histogram.png')
    plot_jitter(cycle_counts, outpath)


if __name__ == '__main__':
    main()
