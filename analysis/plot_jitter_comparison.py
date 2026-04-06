#!/usr/bin/env python3
"""Plot FPGA vs CPU jitter comparison from benchmark data."""

import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

here = Path(__file__).parent

# Load C jitter data
c_times = []
with open(here / 'c_jitter_data.csv') as f:
    reader = csv.DictReader(f)
    for row in reader:
        c_times.append(float(row['ns']))
c_times = np.array(c_times)

# FPGA: constant 95 cycles at 27 MHz = 3518.5 ns, zero jitter
fpga_cycles = 95
fpga_ns = fpga_cycles / 27.0 * 1000  # 3518.5 ns

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('9x9 Complex Matvec: FPGA vs CPU Latency Jitter', fontsize=14, fontweight='bold')

# --- Top left: CPU time series ---
ax = axes[0, 0]
ax.plot(c_times, '.', markersize=0.5, alpha=0.3, color='#e74c3c')
ax.axhline(y=np.median(c_times), color='#c0392b', linestyle='--', linewidth=1,
           label=f'median = {np.median(c_times):.0f} ns')
ax.set_xlabel('Trial')
ax.set_ylabel('Latency (ns)')
ax.set_title('CPU (i9-13980HX, -O3 -march=native)')
ax.legend(fontsize=9)
ax.set_ylim(0, min(500, np.percentile(c_times, 99.9) * 1.5))

# --- Top right: CPU histogram ---
ax = axes[0, 1]
# Clip to 99.5th percentile for readable histogram
clip = np.percentile(c_times, 99.5)
clipped = c_times[c_times <= clip]
ax.hist(clipped, bins=80, color='#e74c3c', edgecolor='#c0392b', alpha=0.7)
ax.axvline(x=np.median(c_times), color='black', linestyle='--', linewidth=1)
ax.set_xlabel('Latency (ns)')
ax.set_ylabel('Count')
ax.set_title(f'CPU Distribution (n={len(c_times)}, '
             f'range={c_times.min():.0f}-{c_times.max():.0f} ns)')
ax.text(0.95, 0.85,
        f'mean: {c_times.mean():.0f} ns\n'
        f'std:  {c_times.std():.0f} ns\n'
        f'min:  {c_times.min():.0f} ns\n'
        f'max:  {c_times.max():.0f} ns',
        transform=ax.transAxes, ha='right', va='top', fontsize=9,
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

# --- Bottom left: FPGA "time series" (flat line) ---
ax = axes[1, 0]
n_fpga = 1000
fpga_data = np.full(n_fpga, fpga_ns)
ax.plot(fpga_data, '.', markersize=0.5, color='#2980b9')
ax.axhline(y=fpga_ns, color='#2c3e50', linestyle='--', linewidth=1,
           label=f'{fpga_ns:.1f} ns (95 cycles)')
ax.set_xlabel('Trial')
ax.set_ylabel('Latency (ns)')
ax.set_title('FPGA (Tang Nano 20K, 27 MHz)')
ax.legend(fontsize=9)
ax.set_ylim(fpga_ns - 200, fpga_ns + 200)

# --- Bottom right: Comparison bar chart ---
ax = axes[1, 1]

categories = ['Mean\nLatency', 'Std Dev\n(Jitter)', 'Worst\nCase', 'Best\nCase']
cpu_vals = [c_times.mean(), c_times.std(), c_times.max(), c_times.min()]
fpga_vals = [fpga_ns, 0, fpga_ns, fpga_ns]

x = np.arange(len(categories))
width = 0.35
bars1 = ax.bar(x - width/2, cpu_vals, width, label='CPU (i9-13980HX)',
               color='#e74c3c', edgecolor='#c0392b')
bars2 = ax.bar(x + width/2, fpga_vals, width, label='FPGA (27 MHz)',
               color='#2980b9', edgecolor='#2471a3')

ax.set_ylabel('Nanoseconds')
ax.set_title('Latency Comparison')
ax.set_xticks(x)
ax.set_xticklabels(categories)
ax.legend(fontsize=9)
ax.set_yscale('symlog', linthresh=100)

# Add value labels on bars
for bar in bars1:
    h = bar.get_height()
    if h > 0:
        ax.text(bar.get_x() + bar.get_width()/2., h,
                f'{h:.0f}', ha='center', va='bottom', fontsize=8)
for bar in bars2:
    h = bar.get_height()
    if h > 0:
        ax.text(bar.get_x() + bar.get_width()/2., h,
                f'{h:.0f}', ha='center', va='bottom', fontsize=8)

plt.tight_layout()
outpath = here / 'jitter_comparison.png'
plt.savefig(outpath, dpi=150)
print(f"Saved to {outpath}")
