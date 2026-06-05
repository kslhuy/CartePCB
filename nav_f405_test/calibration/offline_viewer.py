"""
IMU Offline Data Viewer
========================
Record data to file, then analyze it later with rich plots.
Also compares before/after calibration side-by-side.

Usage:
    # Record 60 seconds of data:
    python offline_viewer.py --port COM4 --record --duration 60

    # Analyze the recorded data:
    python offline_viewer.py --analyze imu_recording.npz

    # Analyze with calibration applied:
    python offline_viewer.py --analyze imu_recording.npz --cal imu_calibration.json

    # Compare raw vs calibrated side by side:
    python offline_viewer.py --compare imu_recording.npz --cal imu_calibration.json
"""

import argparse
import re
import sys
import time
import json
import numpy as np
import serial
import matplotlib.pyplot as plt
from pathlib import Path

ACCEL_SENS  = 2048.0
GYRO_SENS   = 16.4
TEMP_SENS   = 132.48
TEMP_OFFSET = 25.0

LINE_RE = re.compile(
    r"AX:\s*(-?\d+)\s+AY:\s*(-?\d+)\s+AZ:\s*(-?\d+)\s+"
    r"GX:\s*(-?\d+)\s+GY:\s*(-?\d+)\s+GZ:\s*(-?\d+)\s+"
    r"T:\s*(-?\d+)"
)


def raw_temp_to_c(raw):
    return (raw / TEMP_SENS) + TEMP_OFFSET


def apply_calibration(raw_data, cal):
    """
    Apply calibration to raw data array (N, 7).
    Returns (accel_g (N,3), gyro_dps (N,3)).
    """
    raw_a = raw_data[:, 0:3]
    raw_g = raw_data[:, 3:6]

    gb = np.array(cal.get("gyro", {}).get("gyro_bias_lsb", [0, 0, 0]))
    corr_g = (raw_g - gb) / GYRO_SENS

    accel_cal = cal.get("accel", {})
    if "full" in accel_cal:
        M = np.array(accel_cal["full"]["misalignment_matrix"])
        ab = np.array(accel_cal["full"]["accel_bias_lsb"])
        corr_a = ((M @ (raw_a - ab).T).T) / ACCEL_SENS
    elif "simple" in accel_cal:
        ab = np.array(accel_cal["simple"]["accel_bias_lsb"])
        sc = np.array(accel_cal["simple"]["accel_scale"])
        corr_a = sc * (raw_a - ab) / ACCEL_SENS
    else:
        corr_a = raw_a / ACCEL_SENS

    return corr_a, corr_g


# ── RECORD ──────────────────────────────────────────────────────────────

def record_data(port, baud, duration_s, output):
    """Record raw IMU data from UART4 to a .npz file."""
    ser = serial.Serial(port, baud, timeout=2.0)
    ser.reset_input_buffer()
    time.sleep(0.3)
    ser.reset_input_buffer()

    print(f"  Recording for {duration_s}s from {port} ...")
    data = []
    timestamps = []
    t0 = time.time()

    while time.time() - t0 < duration_s:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        m = LINE_RE.search(line)
        if m:
            data.append([int(m.group(i)) for i in range(1, 8)])
            timestamps.append(time.time() - t0)
            n = len(data)
            elapsed = time.time() - t0
            print(f"\r  {n} samples | {elapsed:.0f}/{duration_s}s", end="", flush=True)

    ser.close()
    data = np.array(data, dtype=np.float64)
    timestamps = np.array(timestamps)
    np.savez(output, data=data, timestamps=timestamps)
    print(f"\n  Saved {len(data)} samples -> {output}")
    print(f"  Effective rate: {len(data)/duration_s:.1f} Hz")


# ── ANALYZE ─────────────────────────────────────────────────────────────

def analyze_data(npz_file, cal_file=None):
    """Comprehensive offline analysis of recorded IMU data."""
    d = np.load(npz_file)
    data = d["data"]
    ts = d["timestamps"]
    n = len(data)

    print(f"  Loaded {n} samples from {npz_file}")
    print(f"  Duration: {ts[-1]:.1f}s  Rate: {n/ts[-1]:.1f} Hz")

    # Raw converted
    raw_a = data[:, 0:3] / ACCEL_SENS
    raw_g = data[:, 3:6] / GYRO_SENS
    temp = np.array([raw_temp_to_c(t) for t in data[:, 6]])

    # Calibrated (if available)
    cal = None
    cal_a, cal_g = raw_a, raw_g
    if cal_file and Path(cal_file).exists():
        with open(cal_file) as f:
            cal = json.load(f)
        cal_a, cal_g = apply_calibration(data, cal)
        print(f"  Calibration: {cal_file}")

    label = "Calibrated" if cal else "Raw"

    # === Figure 1: Time series ===
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle(f"IMU Data Analysis ({label}) - {npz_file}", fontweight="bold", fontsize=13)

    a = cal_a if cal else raw_a
    g = cal_g if cal else raw_g

    # Accel
    axes[0].plot(ts, a[:, 0], 'r-', lw=0.5, label="AX")
    axes[0].plot(ts, a[:, 1], 'g-', lw=0.5, label="AY")
    axes[0].plot(ts, a[:, 2], 'b-', lw=0.5, label="AZ")
    axes[0].set_ylabel("Accel (g)")
    axes[0].legend(loc="upper right", ncol=3, fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(0, color="gray", lw=0.5)

    # Gyro
    axes[1].plot(ts, g[:, 0], 'r-', lw=0.5, label="GX")
    axes[1].plot(ts, g[:, 1], 'g-', lw=0.5, label="GY")
    axes[1].plot(ts, g[:, 2], 'b-', lw=0.5, label="GZ")
    axes[1].set_ylabel("Gyro (deg/s)")
    axes[1].legend(loc="upper right", ncol=3, fontsize=8)
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(0, color="gray", lw=0.5)

    # Norm
    norm_a = np.linalg.norm(a, axis=1)
    axes[2].plot(ts, norm_a, 'k-', lw=0.5)
    axes[2].axhline(1.0, color="green", lw=1.5, ls="--", label="1.00 g")
    axes[2].axhspan(0.99, 1.01, alpha=0.1, color="green")
    axes[2].set_ylabel("|a| (g)")
    axes[2].legend(loc="upper right", fontsize=8)
    axes[2].grid(True, alpha=0.3)

    # Temperature
    axes[3].plot(ts, temp, 'm-', lw=0.8)
    axes[3].set_ylabel("Temp (C)")
    axes[3].set_xlabel("Time (s)")
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("imu_analysis.png", dpi=150)
    print("  Saved -> imu_analysis.png")

    # === Figure 2: Statistics ===
    fig2, axes2 = plt.subplots(2, 4, figsize=(16, 8))
    fig2.suptitle(f"IMU Statistics ({label})", fontweight="bold", fontsize=13)

    labels_a = ["AX (g)", "AY (g)", "AZ (g)"]
    labels_g = ["GX (d/s)", "GY (d/s)", "GZ (d/s)"]
    colors = ["#e74c3c", "#2ecc71", "#3498db"]

    for i in range(3):
        axes2[0, i].hist(a[:, i], bins=100, color=colors[i], alpha=0.7, edgecolor="white", lw=0.3)
        mean_v = a[:, i].mean()
        axes2[0, i].axvline(mean_v, color="k", ls="--", lw=1.5)
        axes2[0, i].set_title(f"{labels_a[i]}\nmean={mean_v:.4f} std={a[:, i].std():.4f}")
        axes2[0, i].set_xlabel(labels_a[i])
        axes2[0, i].grid(True, alpha=0.2)

        axes2[1, i].hist(g[:, i], bins=100, color=colors[i], alpha=0.7, edgecolor="white", lw=0.3)
        mean_g = g[:, i].mean()
        axes2[1, i].axvline(mean_g, color="k", ls="--", lw=1.5)
        axes2[1, i].set_title(f"{labels_g[i]}\nmean={mean_g:.4f} std={g[:, i].std():.4f}")
        axes2[1, i].set_xlabel(labels_g[i])
        axes2[1, i].grid(True, alpha=0.2)

    # Norm histogram
    axes2[0, 3].hist(norm_a, bins=100, color="black", alpha=0.7, edgecolor="white", lw=0.3)
    axes2[0, 3].axvline(1.0, color="green", ls="--", lw=2)
    norm_err_mg = abs(norm_a.mean() - 1.0) * 1000
    axes2[0, 3].set_title(f"|a| norm\nmean={norm_a.mean():.4f}g err={norm_err_mg:.1f}mg")
    axes2[0, 3].grid(True, alpha=0.2)

    # Temperature
    axes2[1, 3].plot(ts, temp, 'm-', lw=0.8)
    axes2[1, 3].set_title(f"Temperature\n{temp.min():.1f} - {temp.max():.1f} C")
    axes2[1, 3].set_xlabel("Time (s)")
    axes2[1, 3].grid(True, alpha=0.2)

    plt.tight_layout()
    plt.savefig("imu_statistics.png", dpi=150)
    print("  Saved -> imu_statistics.png")

    # Print summary
    print(f"\n  === Quality Summary ===")
    print(f"  |a| mean:   {norm_a.mean():.4f} g  (error: {norm_err_mg:.1f} mg)")
    print(f"  |a| std:    {norm_a.std() * 1000:.1f} mg")
    print(f"  Gyro bias:  X={g[:, 0].mean():.3f}  Y={g[:, 1].mean():.3f}  Z={g[:, 2].mean():.3f} d/s")
    print(f"  Temp:       {temp.mean():.1f} C")

    if cal:
        if norm_err_mg < 5:
            verdict = "EXCELLENT (< 5 mg)"
        elif norm_err_mg < 15:
            verdict = "GOOD (< 15 mg)"
        elif norm_err_mg < 50:
            verdict = "FAIR (< 50 mg) -- consider re-calibrating"
        else:
            verdict = "POOR -- re-calibrate needed"
        print(f"  Verdict:    {verdict}")
    else:
        print(f"  (Run with --cal to see calibrated quality)")

    plt.show()


# ── COMPARE RAW vs CALIBRATED ──────────────────────────────────────────

def compare_raw_vs_cal(npz_file, cal_file):
    """Side-by-side comparison of raw vs calibrated data."""
    d = np.load(npz_file)
    data = d["data"]
    ts = d["timestamps"]

    with open(cal_file) as f:
        cal = json.load(f)

    raw_a = data[:, 0:3] / ACCEL_SENS
    raw_g = data[:, 3:6] / GYRO_SENS
    cal_a, cal_g = apply_calibration(data, cal)

    raw_norm = np.linalg.norm(raw_a, axis=1)
    cal_norm = np.linalg.norm(cal_a, axis=1)

    fig, axes = plt.subplots(3, 2, figsize=(16, 10), sharex=True)
    fig.suptitle("Raw vs Calibrated Comparison", fontsize=14, fontweight="bold")

    axes[0, 0].set_title("Accelerometer (RAW)", fontweight="bold")
    axes[0, 1].set_title("Accelerometer (CALIBRATED)", fontweight="bold")
    axes[1, 0].set_title("Gyroscope (RAW)")
    axes[1, 1].set_title("Gyroscope (CALIBRATED)")
    axes[2, 0].set_title(f"|a| RAW  mean={raw_norm.mean():.4f}g  err={abs(raw_norm.mean()-1)*1000:.1f}mg")
    axes[2, 1].set_title(f"|a| CAL  mean={cal_norm.mean():.4f}g  err={abs(cal_norm.mean()-1)*1000:.1f}mg")

    for col, (a, g, norm) in enumerate([(raw_a, raw_g, raw_norm), (cal_a, cal_g, cal_norm)]):
        axes[0, col].plot(ts, a[:, 0], 'r-', lw=0.5, label="AX")
        axes[0, col].plot(ts, a[:, 1], 'g-', lw=0.5, label="AY")
        axes[0, col].plot(ts, a[:, 2], 'b-', lw=0.5, label="AZ")
        axes[0, col].legend(ncol=3, fontsize=7, loc="upper right")
        axes[0, col].set_ylabel("g")
        axes[0, col].grid(True, alpha=0.3)

        axes[1, col].plot(ts, g[:, 0], 'r-', lw=0.5, label="GX")
        axes[1, col].plot(ts, g[:, 1], 'g-', lw=0.5, label="GY")
        axes[1, col].plot(ts, g[:, 2], 'b-', lw=0.5, label="GZ")
        axes[1, col].legend(ncol=3, fontsize=7, loc="upper right")
        axes[1, col].set_ylabel("d/s")
        axes[1, col].grid(True, alpha=0.3)
        axes[1, col].axhline(0, color="gray", lw=0.5)

        axes[2, col].plot(ts, norm, 'k-', lw=0.5)
        axes[2, col].axhline(1.0, color="green", lw=1.5, ls="--")
        axes[2, col].axhspan(0.99, 1.01, alpha=0.1, color="green")
        axes[2, col].set_ylabel("g")
        axes[2, col].set_xlabel("Time (s)")
        axes[2, col].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("raw_vs_calibrated.png", dpi=150)
    print(f"  Saved -> raw_vs_calibrated.png")
    print(f"\n  Improvement:")
    print(f"  |a| error:  {abs(raw_norm.mean()-1)*1000:.1f} mg  ->  {abs(cal_norm.mean()-1)*1000:.1f} mg")
    print(f"  |a| noise:  {raw_norm.std()*1000:.1f} mg  ->  {cal_norm.std()*1000:.1f} mg")
    gyro_before = np.sqrt(np.mean(raw_g.mean(axis=0)**2))
    gyro_after  = np.sqrt(np.mean(cal_g.mean(axis=0)**2))
    print(f"  Gyro bias:  {gyro_before:.3f} d/s  ->  {gyro_after:.3f} d/s")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="IMU Offline Data Viewer")
    parser.add_argument("--port", type=str, help="Serial port for recording")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--record", action="store_true", help="Record data from UART")
    parser.add_argument("--duration", type=int, default=60, help="Record duration (s)")
    parser.add_argument("--output", default="imu_recording.npz", help="Recording output file")
    parser.add_argument("--analyze", type=str, help="Analyze a .npz recording")
    parser.add_argument("--compare", type=str, help="Compare raw vs calibrated")
    parser.add_argument("--cal", type=str, default=None, help="Calibration JSON file")
    args = parser.parse_args()

    if args.record:
        if not args.port:
            print("  --port required for recording")
            sys.exit(1)
        record_data(args.port, args.baud, args.duration, args.output)
    elif args.analyze:
        analyze_data(args.analyze, args.cal)
    elif args.compare:
        if not args.cal:
            print("  --cal required for comparison")
            sys.exit(1)
        compare_raw_vs_cal(args.compare, args.cal)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
