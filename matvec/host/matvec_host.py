#!/usr/bin/env python3
"""
Host-side validation script for FPGA 9x9 complex matvec.

Builds a d=3 transmon propagator, quantizes to Q1.15, sends to FPGA
over serial, runs propagation steps, and compares results to numpy
reference.

Usage:
    python matvec/host/matvec_host.py [--port /dev/ttyUSB1] [--steps 10]

Protocol:
    'P' + 324 bytes  -> Load propagator. FPGA acks with 'K'.
    'V' + 36 bytes   -> Load rho vector. FPGA acks with 'K'.
    'S'              -> Single step. Returns 36 bytes.
    'N' + 2 bytes    -> N steps (uint16 LE). Returns 36 bytes.
"""

import argparse
import struct
import sys
import time
import numpy as np
from scipy.linalg import expm

try:
    import serial
except ImportError:
    print("pyserial required: pip install pyserial", file=sys.stderr)
    sys.exit(1)


# =====================================================================
# Q1.15 fixed-point helpers
# =====================================================================

def to_q15(x: float) -> int:
    """Convert float to Q1.15 (16-bit signed)."""
    v = int(round(x * 32768.0))
    return max(-32768, min(32767, v))


def from_q15(v: int) -> float:
    """Convert Q1.15 to float."""
    if v >= 32768:
        v -= 65536  # unsigned -> signed
    return v / 32768.0


def q15_matvec(p_re, p_im, rho_re, rho_im):
    """Reference Q1.15 matvec in Python (matches FPGA exactly)."""
    out_re = np.zeros(9, dtype=np.int64)
    out_im = np.zeros(9, dtype=np.int64)
    for i in range(9):
        acc_re, acc_im = 0, 0
        for j in range(9):
            k = i * 9 + j
            acc_re += int(p_re[k]) * int(rho_re[j]) - int(p_im[k]) * int(rho_im[j])
            acc_im += int(p_re[k]) * int(rho_im[j]) + int(p_im[k]) * int(rho_re[j])
        out_re[i] = acc_re >> 15
        out_im[i] = acc_im >> 15
    return out_re.astype(np.int16), out_im.astype(np.int16)


# =====================================================================
# Propagator construction (same as precision_test.py)
# =====================================================================

def build_propagator(d=3, T1_us=50.0, T2_us=30.0, dt=0.5):
    """Build transmon propagator P = expm(L*dt), return as (d^2, d^2) complex."""
    d2 = d * d
    H = np.diag(np.arange(d, dtype=complex))
    gamma1 = 1.0 / T1_us
    cops = []
    if d >= 2:
        L1 = np.zeros((d, d), dtype=complex)
        L1[0, 1] = np.sqrt(gamma1)
        cops.append(L1)
    T_phi = 1.0 / (1.0 / T2_us - 0.5 / T1_us)
    if T_phi > 0 and d >= 2:
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
    return expm(L_super * dt)


# =====================================================================
# Serial protocol
# =====================================================================

def send_load_p(ser, p_re_q15, p_im_q15):
    """Send 'P' command + 324 bytes, wait for 'K' ack."""
    data = bytearray()
    for k in range(81):
        data += struct.pack('<hh', int(p_re_q15[k]), int(p_im_q15[k]))
    assert len(data) == 324
    ser.write(b'P' + data)
    ack = ser.read(1)
    if ack != b'K':
        raise RuntimeError(f"Expected 'K' ack, got {ack!r}")


def send_load_rho(ser, rho_re_q15, rho_im_q15):
    """Send 'V' command + 36 bytes, wait for 'K' ack."""
    data = bytearray()
    for k in range(9):
        data += struct.pack('<hh', int(rho_re_q15[k]), int(rho_im_q15[k]))
    assert len(data) == 36
    ser.write(b'V' + data)
    ack = ser.read(1)
    if ack != b'K':
        raise RuntimeError(f"Expected 'K' ack, got {ack!r}")


def send_step(ser):
    """Send 'S', receive 36 bytes of rho."""
    ser.write(b'S')
    data = ser.read(36)
    if len(data) != 36:
        raise RuntimeError(f"Expected 36 bytes, got {len(data)}")
    return decode_rho(data)


def send_n_steps(ser, n):
    """Send 'N' + uint16_le, receive 36 bytes of rho."""
    ser.write(b'N' + struct.pack('<H', n))
    data = ser.read(36)
    if len(data) != 36:
        raise RuntimeError(f"Expected 36 bytes, got {len(data)}")
    return decode_rho(data)


def decode_rho(data):
    """Decode 36 bytes into (re[9], im[9]) as Q1.15 int16 arrays."""
    re = np.zeros(9, dtype=np.int16)
    im = np.zeros(9, dtype=np.int16)
    for k in range(9):
        re[k], im[k] = struct.unpack('<hh', data[k*4:(k+1)*4])
    return re, im


# =====================================================================
# Main
# =====================================================================

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', default='/dev/ttyUSB1')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--steps', type=int, default=10)
    parser.add_argument('--clk-mhz', type=float, default=135.0,
                        help='FPGA core clock in MHz for cycle-to-time conversion')
    args = parser.parse_args()

    print(f"Building d=3 transmon propagator...")
    P = build_propagator()
    print(f"  Max |P_ij|: {np.abs(P).max():.6f}")

    # Quantize P to Q1.15
    p_re_q15 = np.array([to_q15(P.real.flat[k]) for k in range(81)], dtype=np.int16)
    p_im_q15 = np.array([to_q15(P.imag.flat[k]) for k in range(81)], dtype=np.int16)

    # Initial state: |0><0|
    rho_re_q15 = np.zeros(9, dtype=np.int16)
    rho_im_q15 = np.zeros(9, dtype=np.int16)
    rho_re_q15[0] = to_q15(1.0)  # 32767 (closest to 1.0 in Q1.15)

    print(f"Opening {args.port} at {args.baud} baud...")
    ser = serial.Serial(args.port, args.baud, timeout=5)
    time.sleep(0.1)  # let FPGA settle
    ser.reset_input_buffer()

    # Load propagator
    print("Loading propagator (324 bytes)...")
    t0 = time.perf_counter()
    send_load_p(ser, p_re_q15, p_im_q15)
    t1 = time.perf_counter()
    print(f"  OK ({(t1-t0)*1000:.0f} ms)")

    # Load initial rho
    print("Loading rho (36 bytes)...")
    send_load_rho(ser, rho_re_q15, rho_im_q15)
    print("  OK")

    # Run steps one at a time, comparing to reference
    print(f"\nRunning {args.steps} steps, comparing FPGA vs Q1.15 reference...")
    ref_re = rho_re_q15.copy()
    ref_im = rho_im_q15.copy()

    all_pass = True
    for step in range(1, args.steps + 1):
        # FPGA: single step
        fpga_re, fpga_im = send_step(ser)

        # Reference: Q1.15 matvec
        ref_re, ref_im = q15_matvec(p_re_q15, p_im_q15, ref_re, ref_im)

        # Compare
        err_re = np.max(np.abs(fpga_re.astype(int) - ref_re.astype(int)))
        err_im = np.max(np.abs(fpga_im.astype(int) - ref_im.astype(int)))
        match = err_re <= 1 and err_im <= 1

        if not match:
            print(f"  Step {step}: MISMATCH (max err re={err_re}, im={err_im})")
            for i in range(9):
                if fpga_re[i] != ref_re[i] or fpga_im[i] != ref_im[i]:
                    print(f"    [{i}] fpga=({fpga_re[i]},{fpga_im[i]}) "
                          f"ref=({ref_re[i]},{ref_im[i]})")
            all_pass = False
        elif step <= 3 or step == args.steps:
            # Print trace for first few and last step
            trace = sum(from_q15(int(fpga_re[i*(3+1)])) for i in range(3))
            print(f"  Step {step:4d}: trace={trace:.6f}, max_err=({err_re},{err_im})")

    if all_pass:
        print(f"\nAll {args.steps} steps: bit-exact match with Q1.15 reference.")
    else:
        print(f"\nSome steps had mismatches.")

    # Also test multi-step: reload and do N steps at once
    print(f"\nMulti-step test: reload + {args.steps} steps at once...")
    rho_re_q15[0] = to_q15(1.0)
    for i in range(1, 9):
        rho_re_q15[i] = 0
        rho_im_q15[i] = 0
    rho_im_q15[0] = 0
    send_load_rho(ser, rho_re_q15, rho_im_q15)

    t0 = time.perf_counter()
    fpga_re, fpga_im = send_n_steps(ser, args.steps)
    t1 = time.perf_counter()

    # Reference: N steps
    ref_re = rho_re_q15.copy()
    ref_im = rho_im_q15.copy()
    for _ in range(args.steps):
        ref_re, ref_im = q15_matvec(p_re_q15, p_im_q15, ref_re, ref_im)

    err_re = np.max(np.abs(fpga_re.astype(int) - ref_re.astype(int)))
    err_im = np.max(np.abs(fpga_im.astype(int) - ref_im.astype(int)))
    match = err_re <= 1 and err_im <= 1
    print(f"  FPGA time: {(t1-t0)*1000:.1f} ms (includes UART overhead)")
    print(f"  Compute: {args.steps} steps * 94 cycles / {args.clk_mhz:.0f} MHz = "
          f"{args.steps * 94 / args.clk_mhz:.1f} us")
    print(f"  Max error vs reference: re={err_re}, im={err_im}")
    print(f"  {'PASS' if match else 'FAIL'}")

    ser.close()


if __name__ == '__main__':
    main()
