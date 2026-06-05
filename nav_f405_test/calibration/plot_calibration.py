"""
IMU Calibration Visualization
==============================
Plot calibration results from imu_calibrate.py output files.

Usage:
    python plot_calibration.py                        # plot everything available
    python plot_calibration.py --cal imu_calibration.json
"""

import argparse
import json
import numpy as np
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("[WARNING] matplotlib not installed. Install with: pip install matplotlib")


ACCEL_SENS = 2048.0
GYRO_SENS  = 16.4
TEMP_SENS  = 132.48
TEMP_OFFSET = 25.0


def plot_gyro_noise(data: np.ndarray):
    """Plot gyro raw noise histograms + time series from stationary data."""
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    fig.suptitle("Gyroscope Stationary Noise Analysis", fontsize=14, fontweight="bold")

    labels = ["GX", "GY", "GZ"]
    colors = ["#e74c3c", "#2ecc71", "#3498db"]

    for i in range(3):
        raw = data[:, 3 + i]
        dps = raw / GYRO_SENS
        t = np.arange(len(raw)) / 100.0  # assume 100 Hz

        # Time series
        ax = axes[0, i]
        ax.plot(t, dps, color=colors[i], alpha=0.5, linewidth=0.3)
        ax.axhline(dps.mean(), color="black", linestyle="--", linewidth=1,
                   label=f"mean={dps.mean():.4f}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("deg/s")
        ax.set_title(f"{labels[i]} Time Series")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # Histogram
        ax = axes[1, i]
        ax.hist(dps, bins=80, color=colors[i], alpha=0.7, edgecolor="white", linewidth=0.5)
        ax.axvline(dps.mean(), color="black", linestyle="--", linewidth=1.5)
        ax.set_xlabel("deg/s")
        ax.set_ylabel("Count")
        ax.set_title(f"{labels[i]} Distribution (std={dps.std():.4f})")
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("gyro_noise_analysis.png", dpi=150)
    print("  Saved -> gyro_noise_analysis.png")
    plt.show()


def plot_allan_deviation(allan: dict):
    """Plot Allan deviation curves for gyro axes."""
    fig, ax = plt.subplots(1, 1, figsize=(10, 7))
    ax.set_title("Gyroscope Allan Deviation", fontsize=14, fontweight="bold")

    colors = {"gx": "#e74c3c", "gy": "#2ecc71", "gz": "#3498db"}
    for name in ["gx", "gy", "gz"]:
        if name in allan:
            tau = allan[name]["tau"]
            adev = allan[name]["adev_dps"]
            ax.loglog(tau, adev, "o-", color=colors[name], label=name.upper(),
                      markersize=4, linewidth=1.5)

    ax.set_xlabel("Cluster time tau (s)", fontsize=12)
    ax.set_ylabel("Allan Deviation (deg/s)", fontsize=12)
    ax.legend(fontsize=11)
    ax.grid(True, which="both", alpha=0.3)

    # Annotate regions
    ax.annotate("Angle Random Walk\n(slope = -1/2)",
                xy=(0.15, 0.85), xycoords="axes fraction", fontsize=9,
                color="gray", fontstyle="italic")
    ax.annotate("Bias Instability\n(slope = 0, minimum)",
                xy=(0.55, 0.15), xycoords="axes fraction", fontsize=9,
                color="gray", fontstyle="italic")

    plt.tight_layout()
    plt.savefig("allan_deviation.png", dpi=150)
    print("  Saved -> allan_deviation.png")
    plt.show()


def plot_accel_6pos(cal_accel: dict):
    """Plot raw vs corrected accel for each position."""
    if "raw_means_lsb" not in cal_accel or "expected_g" not in cal_accel:
        print("  No 6-position raw data in calibration file.")
        return

    raw = np.array(cal_accel["raw_means_lsb"])
    expected = np.array(cal_accel["expected_g"])

    # Use full model if available
    if "full" in cal_accel:
        M = np.array(cal_accel["full"]["misalignment_matrix"])
        b = np.array(cal_accel["full"]["accel_bias_lsb"])
        corrected_lsb = (M @ (raw - b).T).T
        method = "Full 3x3"
    elif "simple" in cal_accel:
        sc = np.array(cal_accel["simple"]["accel_scale"])
        b  = np.array(cal_accel["simple"]["accel_bias_lsb"])
        corrected_lsb = sc * (raw - b)
        method = "Simple"
    else:
        return

    corrected_g = corrected_lsb / ACCEL_SENS
    raw_g = raw / ACCEL_SENS

    pos_labels = ["Z+", "Z-", "X+", "X-", "Y+", "Y-"]
    axes_labels = ["AX", "AY", "AZ"]
    colors = ["#e74c3c", "#2ecc71", "#3498db"]

    fig, axes = plt.subplots(1, 3, figsize=(14, 6))
    fig.suptitle(f"Accelerometer 6-Position Calibration ({method})",
                 fontsize=14, fontweight="bold")

    x = np.arange(len(pos_labels))
    width = 0.25

    for i in range(3):
        ax = axes[i]
        ax.bar(x - width, raw_g[:, i], width, label="Raw", alpha=0.6, color="gray")
        ax.bar(x, corrected_g[:, i], width, label="Corrected", alpha=0.8, color=colors[i])
        ax.bar(x + width, expected[:, i], width, label="Expected", alpha=0.4,
               color="black", edgecolor="black", linewidth=0.5)
        ax.set_xticks(x)
        ax.set_xticklabels(pos_labels)
        ax.set_ylabel("g")
        ax.set_title(axes_labels[i])
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    plt.savefig("accel_6pos_calibration.png", dpi=150)
    print("  Saved -> accel_6pos_calibration.png")
    plt.show()


def plot_temp_sweep(cal_temp: dict):
    """Plot bias vs temperature for gyro and accel."""
    temps = np.array(cal_temp["temperatures_c"])
    gyro = np.array(cal_temp["gyro_biases_lsb"])
    accel = np.array(cal_temp["accel_biases_lsb"])

    fig, axes = plt.subplots(2, 3, figsize=(14, 9))
    fig.suptitle("Bias vs Temperature", fontsize=14, fontweight="bold")

    g_labels = ["GX", "GY", "GZ"]
    a_labels = ["AX", "AY", "AZ"]
    colors_g = ["#e74c3c", "#2ecc71", "#3498db"]
    colors_a = ["#e67e22", "#9b59b6", "#1abc9c"]

    for i in range(3):
        # Gyro
        ax = axes[0, i]
        ax.scatter(temps, gyro[:, i] / GYRO_SENS, s=8, alpha=0.5, color=colors_g[i])
        # Linear fit
        model = cal_temp.get("gyro_temp_model", {}).get(g_labels[i], {})
        if model:
            t_fit = np.linspace(temps.min(), temps.max(), 100)
            bias_fit = (model["slope_lsb_per_c"] * t_fit + model["intercept_lsb"]) / GYRO_SENS
            ax.plot(t_fit, bias_fit, "k--", linewidth=1.5,
                    label=f"slope={model['slope_lsb_per_c']/GYRO_SENS:.4f} d/s/C")
        ax.set_xlabel("Temperature (C)")
        ax.set_ylabel("Bias (deg/s)")
        ax.set_title(f"{g_labels[i]} Gyro")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # Accel
        ax = axes[1, i]
        ax.scatter(temps, accel[:, i] / ACCEL_SENS * 1000, s=8, alpha=0.5, color=colors_a[i])
        model_a = cal_temp.get("accel_temp_model", {}).get(a_labels[i], {})
        if model_a:
            bias_fit_a = (model_a["slope_lsb_per_c"] * t_fit + model_a["intercept_lsb"]) / ACCEL_SENS * 1000
            ax.plot(t_fit, bias_fit_a, "k--", linewidth=1.5,
                    label=f"slope={model_a['slope_lsb_per_c']/ACCEL_SENS*1000:.2f} mg/C")
        ax.set_xlabel("Temperature (C)")
        ax.set_ylabel("Bias (mg)")
        ax.set_title(f"{a_labels[i]} Accel")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("temp_sweep.png", dpi=150)
    print("  Saved -> temp_sweep.png")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot IMU calibration results")
    parser.add_argument("--cal", default="imu_calibration.json", help="Calibration JSON file")
    args = parser.parse_args()

    if not HAS_MPL:
        print("Install matplotlib first: pip install matplotlib")
        return

    print("\n  IMU Calibration Plots")
    print("  " + "=" * 40)

    # Plot gyro noise if raw data exists
    gyro_npy = Path("gyro_raw_stationary.npy")
    if gyro_npy.exists():
        print("\n  Plotting gyro noise analysis...")
        data = np.load(gyro_npy)
        plot_gyro_noise(data)

    # Load calibration JSON
    cal_path = Path(args.cal)
    if not cal_path.exists():
        print(f"  {args.cal} not found. Run calibration first.")
        return

    with open(cal_path) as f:
        cal = json.load(f)

    # Allan deviation
    if "gyro" in cal and "allan" in cal["gyro"]:
        print("\n  Plotting Allan deviation...")
        plot_allan_deviation(cal["gyro"]["allan"])

    # Accel 6-position
    if "accel" in cal:
        print("\n  Plotting accel 6-position...")
        plot_accel_6pos(cal["accel"])

    # Temperature sweep
    if "temp_sweep" in cal:
        print("\n  Plotting temperature sweep...")
        plot_temp_sweep(cal["temp_sweep"])

    print("\n  All plots done.\n")


if __name__ == "__main__":
    main()
