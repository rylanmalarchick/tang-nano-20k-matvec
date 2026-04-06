#!/usr/bin/env python3
"""
Fixed-point precision analysis for Lindblad propagator matvec.

Builds a d=3 transmon propagator P (9x9 complex) in double precision,
then simulates evolution using Q1.15 and Q1.31 fixed-point arithmetic.
Measures Hilbert-Schmidt norm error vs double-precision reference after
N propagation steps.

This answers the key question: does fixed-point preserve enough precision
for the FPGA matvec implementation?

Usage:
    python analysis/precision_test.py [--steps 2000] [--plot]
"""

import argparse
import numpy as np
from scipy.linalg import expm


# ---------------------------------------------------------------------------
# System construction (mirrors reference.py / bench_propagate.c)
# ---------------------------------------------------------------------------

def build_transmon_propagator(d: int = 3, T1_us: float = 50.0,
                               T2_us: float = 30.0, dt: float = 0.5):
    """
    Build a d-level transmon Lindbladian and return P = expm(L*dt).
    Returns P as (d^2, d^2) complex128 ndarray (row-major).
    """
    d2 = d * d

    # Hamiltonian: H = diag(0, 1, ..., d-1)
    H = np.diag(np.arange(d, dtype=complex))

    # Collapse operators
    gamma1 = 1.0 / T1_us
    cops = []
    if d >= 2:
        L1 = np.zeros((d, d), dtype=complex)
        L1[0, 1] = np.sqrt(gamma1)
        cops.append(L1)

    T_phi = 1.0 / (1.0 / T2_us - 0.5 / T1_us)
    if T_phi > 0 and d >= 2:
        gamma_phi = 1.0 / T_phi
        L2 = np.zeros((d, d), dtype=complex)
        L2[1, 1] = np.sqrt(gamma_phi)
        cops.append(L2)

    # Build Lindbladian superoperator L (d^2 x d^2)
    # L(rho) = -i[H, rho] + sum_k (Lk rho Lk† - 0.5{Lk†Lk, rho})
    # In superoperator form: L = -i(H⊗I - I⊗H*) + sum_k (Lk⊗Lk* - 0.5(Lk†Lk⊗I + I⊗Lk^T Lk*))
    I = np.eye(d, dtype=complex)
    L_super = -1j * (np.kron(H, I) - np.kron(I, H.conj()))

    for Lk in cops:
        LdL = Lk.conj().T @ Lk
        L_super += (np.kron(Lk, Lk.conj())
                    - 0.5 * np.kron(LdL, I)
                    - 0.5 * np.kron(I, LdL.T))

    # Propagator: P = expm(L * dt)
    P = expm(L_super * dt)
    return P


def ground_state_vec(d: int = 3):
    """Return |0><0| as a d^2 vector (vectorized density matrix)."""
    rho = np.zeros(d * d, dtype=complex)
    rho[0] = 1.0  # |0><0| in row-major vectorization: element (0,0)
    return rho


# ---------------------------------------------------------------------------
# Fixed-point simulation
# ---------------------------------------------------------------------------

def quantize_q(val: np.ndarray, frac_bits: int) -> np.ndarray:
    """Quantize complex array to Q1.{frac_bits} fixed-point (separate re/im)."""
    scale = 2 ** frac_bits
    max_val = scale - 1
    min_val = -scale

    re = np.clip(np.round(val.real * scale), min_val, max_val).astype(np.int64)
    im = np.clip(np.round(val.imag * scale), min_val, max_val).astype(np.int64)
    return re, im


def fixed_matvec(P_re: np.ndarray, P_im: np.ndarray,
                 v_re: np.ndarray, v_im: np.ndarray,
                 frac_bits: int):
    """
    9x9 complex matvec in fixed-point arithmetic.
    Matches the RTL: multiply -> accumulate -> shift.

    For Q1.15: multiply two Q1.15 values, get Q2.30, accumulate in 64-bit,
    then shift >> frac_bits to get result in Q1.{frac_bits}.
    """
    d2 = len(v_re)
    out_re = np.zeros(d2, dtype=np.int64)
    out_im = np.zeros(d2, dtype=np.int64)

    for i in range(d2):
        acc_re = np.int64(0)
        acc_im = np.int64(0)
        for j in range(d2):
            # Complex multiply: (P_re + P_im*j) * (v_re + v_im*j)
            # Real part: P_re*v_re - P_im*v_im
            # Imag part: P_re*v_im + P_im*v_re
            acc_re += P_re[i, j] * v_re[j] - P_im[i, j] * v_im[j]
            acc_im += P_re[i, j] * v_im[j] + P_im[i, j] * v_re[j]

        # Shift back to Q1.{frac_bits}
        out_re[i] = acc_re >> frac_bits
        out_im[i] = acc_im >> frac_bits

    return out_re, out_im


def fixed_to_complex(re: np.ndarray, im: np.ndarray, frac_bits: int) -> np.ndarray:
    """Convert fixed-point back to complex double for comparison."""
    scale = 2.0 ** frac_bits
    return re.astype(np.float64) / scale + 1j * im.astype(np.float64) / scale


def hs_norm(a: np.ndarray, b: np.ndarray) -> float:
    """Hilbert-Schmidt norm ||a - b||."""
    return np.linalg.norm(a - b)


# ---------------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------------

def run_analysis(n_steps: int, do_plot: bool):
    d = 3
    d2 = d * d
    dt = 0.5  # ns

    print(f"Building d={d} transmon propagator (dt={dt} ns)...")
    P = build_transmon_propagator(d=d, dt=dt)

    print(f"Propagator P shape: {P.shape}")
    print(f"Max |P_ij|: {np.abs(P).max():.6f}")
    print(f"Condition number: {np.linalg.cond(P):.2f}")

    # Quantize propagator
    P_re_15, P_im_15 = quantize_q(P, 15)
    P_re_31, P_im_31 = quantize_q(P, 31)

    # Check quantization error on P itself
    P_q15 = fixed_to_complex(P_re_15, P_im_15, 15)
    P_q31 = fixed_to_complex(P_re_31, P_im_31, 31)
    print(f"\nPropagator quantization error:")
    print(f"  Q1.15: ||P - P_q15||_HS = {hs_norm(P, P_q15):.2e}")
    print(f"  Q1.31: ||P - P_q31||_HS = {hs_norm(P, P_q31):.2e}")

    # Initial state: |0><0|
    rho_double = ground_state_vec(d)
    rho_re_15, rho_im_15 = quantize_q(rho_double, 15)
    rho_re_31, rho_im_31 = quantize_q(rho_double, 31)

    # Evolve
    print(f"\nEvolving for {n_steps} steps ({n_steps * dt:.1f} ns)...")
    errors_15 = np.zeros(n_steps)
    errors_31 = np.zeros(n_steps)
    trace_err_15 = np.zeros(n_steps)
    trace_err_31 = np.zeros(n_steps)

    for step in range(n_steps):
        # Double-precision reference
        rho_double = P @ rho_double

        # Q1.15 fixed-point
        rho_re_15, rho_im_15 = fixed_matvec(P_re_15, P_im_15,
                                              rho_re_15, rho_im_15, 15)

        # Q1.31 fixed-point
        rho_re_31, rho_im_31 = fixed_matvec(P_re_31, P_im_31,
                                              rho_re_31, rho_im_31, 31)

        # Convert back and measure error
        rho_q15 = fixed_to_complex(rho_re_15, rho_im_15, 15)
        rho_q31 = fixed_to_complex(rho_re_31, rho_im_31, 31)

        errors_15[step] = hs_norm(rho_double, rho_q15)
        errors_31[step] = hs_norm(rho_double, rho_q31)

        # Trace of density matrix (sum of diagonal elements in d x d)
        # In vectorized form, diag elements are at indices 0, d+1, 2*(d+1), ...
        diag_idx = [i * (d + 1) for i in range(d)]
        trace_15 = sum(rho_q15[i].real for i in diag_idx)
        trace_31 = sum(rho_q31[i].real for i in diag_idx)
        trace_err_15[step] = abs(trace_15 - 1.0)
        trace_err_31[step] = abs(trace_31 - 1.0)

        if (step + 1) % 500 == 0:
            print(f"  Step {step+1:5d}: "
                  f"err_Q15={errors_15[step]:.2e}, "
                  f"err_Q31={errors_31[step]:.2e}, "
                  f"tr_Q15={trace_15:.6f}, "
                  f"tr_Q31={trace_31:.6f}")

    # Final summary
    print(f"\n{'='*60}")
    print(f"Results after {n_steps} steps ({n_steps * dt:.1f} ns):")
    print(f"  Q1.15 (16-bit, ~4.5 decimal digits):")
    print(f"    Final HS error: {errors_15[-1]:.2e}")
    print(f"    Final trace error: {trace_err_15[-1]:.2e}")
    print(f"    Max HS error: {errors_15.max():.2e}")
    print(f"  Q1.31 (32-bit, ~9 decimal digits):")
    print(f"    Final HS error: {errors_31[-1]:.2e}")
    print(f"    Final trace error: {trace_err_31[-1]:.2e}")
    print(f"    Max HS error: {errors_31.max():.2e}")
    print(f"{'='*60}")

    # Verdict for FPGA
    if errors_15[-1] < 1e-2:
        print("\nQ1.15 is viable for short trajectories on FPGA.")
    else:
        print("\nQ1.15 drifts significantly -- Q1.31 or error correction needed.")

    if errors_31[-1] < 1e-6:
        print("Q1.31 maintains double-precision-like accuracy.")
    elif errors_31[-1] < 1e-3:
        print("Q1.31 has measurable but acceptable drift.")
    else:
        print("Q1.31 shows significant drift at this trajectory length.")

    if do_plot:
        plot_results(n_steps, dt, errors_15, errors_31, trace_err_15, trace_err_31)


def plot_results(n_steps, dt, errors_15, errors_31, trace_err_15, trace_err_31):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib not installed, skipping plot.")
        return

    steps = np.arange(1, n_steps + 1)
    time_ns = steps * dt

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # HS error
    ax1.semilogy(time_ns, errors_15, label="Q1.15 (16-bit)", alpha=0.8)
    ax1.semilogy(time_ns, errors_31, label="Q1.31 (32-bit)", alpha=0.8)
    ax1.set_ylabel("Hilbert-Schmidt error vs double")
    ax1.set_title("Fixed-point precision: 9x9 Lindblad propagator matvec")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Trace error
    ax2.semilogy(time_ns, trace_err_15, label="Q1.15 trace error", alpha=0.8)
    ax2.semilogy(time_ns, trace_err_31, label="Q1.31 trace error", alpha=0.8)
    ax2.set_xlabel("Time (ns)")
    ax2.set_ylabel("|Tr(rho) - 1|")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    outpath = "analysis/precision_results.png"
    plt.savefig(outpath, dpi=150)
    print(f"\nPlot saved to {outpath}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--steps", type=int, default=2000,
                        help="Number of propagation steps (default: 2000)")
    parser.add_argument("--plot", action="store_true",
                        help="Save error plot to analysis/precision_results.png")
    args = parser.parse_args()

    run_analysis(args.steps, args.plot)
