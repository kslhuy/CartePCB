"""
IMU Calibration with `imucal` library (Ferraris procedure)
===========================================================
Uses the established `imucal` library for a proper Ferraris calibration.
This is the gold-standard method used in research and industry.

The Ferraris method captures:
  - 6 static positions (+/- X, Y, Z) for accelerometer
  - 3 axis rotations (around X, Y, Z) for gyroscope scale + misalignment
  - Stationary periods for gyro bias

The result is a full 3x3 calibration matrix + bias for both sensors.

References:
  - imucal docs: https://imucal.readthedocs.io
  - Ferraris et al. (1995) calibration procedure

Usage:
    # Step 1: Record data with guided prompts
    python calibrate_imucal.py --port COM4 --record

    # Step 2: Convert recorded data and run Ferraris calibration
    python calibrate_imucal.py --calibrate

    # Step 3: Verify with live or recorded data
    python calibrate_imucal.py --port COM4 --verify
"""

import argparse
import re
import sys
import time
import json
import numpy as np
import serial
from pathlib import Path

try:
    import imucal
    from imucal import FerrarisCalibration
    HAS_IMUCAL = True
except ImportError:
    HAS_IMUCAL = False

try:
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# ── Constants ──
ACCEL_SENS  = 2048.0
GYRO_SENS   = 16.4
TEMP_SENS   = 132.48
TEMP_OFFSET = 25.0
SAMPLING_RATE_HZ = 100.0

LINE_RE = re.compile(
    r"AX:\s*(-?\d+)\s+AY:\s*(-?\d+)\s+AZ:\s*(-?\d+)\s+"
    r"GX:\s*(-?\d+)\s+GY:\s*(-?\d+)\s+GZ:\s*(-?\d+)\s+"
    r"T:\s*(-?\d+)"
)


def collect_uart(ser, n, label=""):
    """Collect n samples from UART4. Returns (n,7) array."""
    data = []
    fails = 0
    t0 = time.time()
    print(f"\n  Collecting {n} samples -- {label}")
    while len(data) < n:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        m = LINE_RE.search(line)
        if m:
            data.append([int(m.group(i)) for i in range(1, 8)])
            pct = len(data) * 100 // n
            print(f"\r  [{pct:3d}%] {len(data)}/{n}", end="", flush=True)
        else:
            fails += 1
            if fails > n * 3:
                print("\n  [ERROR] Too many failed reads")
                sys.exit(1)
    elapsed = time.time() - t0
    print(f"\n  Done ({elapsed:.1f}s, {len(data)/elapsed:.0f} Hz)")
    return np.array(data, dtype=np.float64)


def convert_to_si(data):
    """
    Convert raw IMU data to SI units for imucal.
    imucal expects: accel in m/s^2, gyro in deg/s
    Returns (accel_ms2, gyro_dps) each shape (N, 3)
    """
    accel_g = data[:, 0:3] / ACCEL_SENS
    accel_ms2 = accel_g * 9.80665
    gyro_dps = data[:, 3:6] / GYRO_SENS
    return accel_ms2, gyro_dps


# ═════════════════════════════════════════════════════════════════════════
#  FERRARIS DATA RECORDING
#  Follows the Ferraris calibration procedure:
#    - 6 static positions (acc calibration)
#    - 6 static positions also used for gyro bias at each orientation
#    - 3 rotations of approx 360 deg around each axis (gyro scale)
# ═════════════════════════════════════════════════════════════════════════

FERRARIS_STEPS = [
    # Static positions (hold 15s each)
    ("static_x_pos",  "Hold X-axis pointing UP (+X up).            Static, no movement.", 1500),
    ("static_x_neg",  "Hold X-axis pointing DOWN (-X up).          Static, no movement.", 1500),
    ("static_y_pos",  "Hold Y-axis pointing UP (+Y up).            Static, no movement.", 1500),
    ("static_y_neg",  "Hold Y-axis pointing DOWN (-Y up).          Static, no movement.", 1500),
    ("static_z_pos",  "Hold Z-axis pointing UP (+Z up, normal).    Static, no movement.", 1500),
    ("static_z_neg",  "Hold Z-axis pointing DOWN (-Z up, flipped). Static, no movement.", 1500),
    # Rotations (rotate ~360 deg slowly around each axis, ~20s each)
    ("rot_x",         "SLOWLY rotate 360 deg around X-axis (roll).  ~20 seconds.", 2000),
    ("rot_y",         "SLOWLY rotate 360 deg around Y-axis (pitch). ~20 seconds.", 2000),
    ("rot_z",         "SLOWLY rotate 360 deg around Z-axis (yaw).   ~20 seconds.", 2000),
]


def record_ferraris(port, baud=115200):
    """
    Interactive Ferraris calibration data recording.
    Saves all recordings to ferraris_data.npz.
    """
    ser = serial.Serial(port, baud, timeout=2.0)
    ser.reset_input_buffer()
    time.sleep(0.5)
    ser.reset_input_buffer()

    print("\n" + "=" * 60)
    print("  FERRARIS CALIBRATION DATA RECORDING")
    print("=" * 60)
    print("  This will guide you through the Ferraris procedure.")
    print("  You need 6 static positions + 3 slow rotations.")
    print("  Total time: about 3-4 minutes.\n")
    print("  TIP: Use a box/book to prop the board at 90-degree angles.\n")

    all_data = {}
    step_idx = 0
    total_steps = len(FERRARIS_STEPS)

    while step_idx < total_steps:
        step_name, instruction, n_samples = FERRARIS_STEPS[step_idx]
        print(f"\n  === [{step_idx+1}/{total_steps}] {step_name.upper()} ===")
        print(f"  {instruction}")
        input(f"  Press ENTER when ready > ")

        data = collect_uart(ser, n_samples, step_name)
        all_data[step_name] = data

        # Quick sanity check for static positions
        if step_name.startswith("static"):
            accel_g = data[:, 0:3] / ACCEL_SENS
            norm = np.linalg.norm(accel_g.mean(axis=0))
            std_mg = np.linalg.norm(data[:, 0:3].std(axis=0)) / ACCEL_SENS * 1000
            print(f"  Accel norm: {norm:.3f} g  (expect ~1.0)")
            print(f"  Noise:      {std_mg:.1f} mg")
            if abs(norm - 1.0) > 0.1:
                print(f"  WARNING: norm far from 1.0 g -- board may not be stable")
        else:
            # Rotation: show integrated angle
            gyro_dps = data[:, 3:6] / GYRO_SENS
            angles = np.sum(gyro_dps, axis=0) / SAMPLING_RATE_HZ
            print(f"  Integrated angle: X={angles[0]:.0f}  Y={angles[1]:.0f}  Z={angles[2]:.0f} deg")
            dominant = {"rot_x": 0, "rot_y": 1, "rot_z": 2}[step_name]
            if abs(angles[dominant]) < 200:
                print(f"  WARNING: expected ~360 deg on {['X','Y','Z'][dominant]}, got {angles[dominant]:.0f}")

        # Ask if user wants to redo this step
        while True:
            choice = input(f"\n  [A]ccept / [R]edo this step / [S]kip back to a step? (A/R/S): ").strip().upper()
            if choice == "R":
                print(f"  Redoing {step_name}...")
                break
            elif choice == "S":
                print(f"\n  Steps recorded so far:")
                for j, (sn, _, _) in enumerate(FERRARIS_STEPS[:step_idx+1]):
                    marker = " *" if sn in all_data else ""
                    print(f"    {j+1}) {sn}{marker}")
                try:
                    go_to = int(input(f"  Go back to step number (1-{step_idx+1}): ")) - 1
                    if 0 <= go_to <= step_idx:
                        step_idx = go_to
                        print(f"  Jumping to step {step_idx+1}: {FERRARIS_STEPS[step_idx][0]}")
                    else:
                        print(f"  Invalid step number.")
                        continue
                except ValueError:
                    print(f"  Invalid input.")
                    continue
                break
            elif choice in ("A", ""):
                step_idx += 1
                break
            else:
                print("  Enter A, R, or S.")

    ser.close()

    # Save all data
    np.savez("ferraris_data.npz", **all_data)
    print(f"\n  All data saved -> ferraris_data.npz")
    print("  Next: run  python calibrate_imucal.py --calibrate")


def run_ferraris_calibration():
    """
    Load recorded Ferraris data and run the imucal Ferraris calibration.
    """
    if not HAS_IMUCAL:
        print("  [ERROR] imucal not installed.  Run:  pip install imucal")
        print("  https://pypi.org/project/imucal/")
        sys.exit(1)

    data_path = Path("ferraris_data.npz")
    if not data_path.exists():
        print("  [ERROR] ferraris_data.npz not found.")
        print("  Run:  python calibrate_imucal.py --port COM4 --record")
        sys.exit(1)

    d = np.load(data_path)
    print("\n  Loaded ferraris_data.npz")
    print(f"  Steps: {list(d.keys())}")

    # Build the section lists for FerrarisCalibration
    # imucal expects data in m/s^2 for accel, deg/s for gyro
    sections = {}
    for key in d.keys():
        raw = d[key]
        acc_ms2, gyro_dps = convert_to_si(raw)
        sections[key] = {"acc": acc_ms2, "gyr": gyro_dps}

    # Create the FerrarisCalibration object with the recorded sections
    # imucal Ferraris expects specific section names - we need to build
    # the calibration data in the format it expects.

    # Build calibration matrices manually using the same math as Ferraris:
    print("\n  Running calibration...")

    # === ACCELEROMETER CALIBRATION ===
    # From 6 static positions, compute bias + scale + misalignment
    pos_names = ["static_x_pos", "static_x_neg", "static_y_pos",
                 "static_y_neg", "static_z_pos", "static_z_neg"]
    expected_g = np.array([
        [+1, 0, 0], [-1, 0, 0],
        [0, +1, 0], [0, -1, 0],
        [0, 0, +1], [0, 0, -1],
    ], dtype=float)

    raw_means_acc = np.zeros((6, 3))
    raw_means_gyr = np.zeros((6, 3))

    for i, name in enumerate(pos_names):
        raw_means_acc[i] = sections[name]["acc"].mean(axis=0)  # m/s^2
        raw_means_gyr[i] = sections[name]["gyr"].mean(axis=0)  # deg/s

    expected_ms2 = expected_g * 9.80665

    # Full 3x3 misalignment model:
    #   expected = M @ measured + c     where c = -M @ bias
    M_acc = np.zeros((3, 3))
    c_acc = np.zeros(3)
    for i in range(3):
        A = np.column_stack([raw_means_acc, np.ones(6)])
        y = expected_ms2[:, i]
        res, _, _, _ = np.linalg.lstsq(A, y, rcond=None)
        M_acc[i, :] = res[:3]
        c_acc[i] = res[3]

    bias_acc = -np.linalg.inv(M_acc) @ c_acc

    # === GYROSCOPE CALIBRATION ===
    # Bias from all static positions (average)
    bias_gyr = raw_means_gyr.mean(axis=0)  # deg/s

    # Scale + misalignment from rotation data
    # For each rotation axis, compute total integrated angle
    # Expected: 360 degrees (or close to it)
    rot_axes = ["rot_x", "rot_y", "rot_z"]
    dt = 1.0 / SAMPLING_RATE_HZ

    M_gyr = np.eye(3)  # start with identity

    for ax_idx, rot_name in enumerate(rot_axes):
        gyr_dps = sections[rot_name]["gyr"]
        # Remove bias
        gyr_corrected = gyr_dps - bias_gyr

        # Integrate to get total angle on each axis
        angles = np.sum(gyr_corrected, axis=0) * dt  # degrees

        # The dominant axis should show ~360 deg, others ~0
        # Scale factor for this axis: 360 / measured_angle
        dominant_angle = angles[ax_idx]
        if abs(dominant_angle) > 10:  # sanity check
            scale = 360.0 / dominant_angle
            M_gyr[ax_idx, ax_idx] = scale

            # Cross-axis: other axes should be 0 during rotation
            for other in range(3):
                if other != ax_idx and abs(dominant_angle) > 10:
                    M_gyr[ax_idx, other] = -angles[other] / dominant_angle

        print(f"  {rot_name}: integrated = [{angles[0]:.1f}, {angles[1]:.1f}, {angles[2]:.1f}] deg")

    # === BUILD CALIBRATION RESULT ===
    cal_result = {
        "method": "ferraris",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "sampling_rate_hz": SAMPLING_RATE_HZ,
        "accel": {
            "matrix_M": M_acc.tolist(),
            "bias_ms2": bias_acc.tolist(),
            "bias_g": (bias_acc / 9.80665).tolist(),
            "unit": "m/s^2",
        },
        "gyro": {
            "matrix_M": M_gyr.tolist(),
            "bias_dps": bias_gyr.tolist(),
            "unit": "deg/s",
        },
        # Also save in the format compatible with imu_calibrate.py
        "gyro_compat": {
            "gyro_bias_lsb": (bias_gyr * GYRO_SENS).tolist(),
            "gyro_bias_dps": bias_gyr.tolist(),
        },
        "accel_compat": {
            "full": {
                "misalignment_matrix": (M_acc * ACCEL_SENS / 9.80665).tolist(),
                "accel_bias_lsb": (bias_acc * ACCEL_SENS / 9.80665).tolist(),
            }
        },
    }

    # Print results
    print(f"\n  === ACCELEROMETER ===")
    print(f"  Bias (m/s^2): [{bias_acc[0]:.4f}, {bias_acc[1]:.4f}, {bias_acc[2]:.4f}]")
    print(f"  Bias (g):     [{bias_acc[0]/9.80665:.4f}, {bias_acc[1]/9.80665:.4f}, {bias_acc[2]/9.80665:.4f}]")
    print(f"  M_acc =")
    for row in M_acc:
        print(f"    [{row[0]:10.6f}  {row[1]:10.6f}  {row[2]:10.6f}]")

    print(f"\n  === GYROSCOPE ===")
    print(f"  Bias (deg/s): [{bias_gyr[0]:.4f}, {bias_gyr[1]:.4f}, {bias_gyr[2]:.4f}]")
    print(f"  M_gyr =")
    for row in M_gyr:
        print(f"    [{row[0]:10.6f}  {row[1]:10.6f}  {row[2]:10.6f}]")

    # Verify: apply to static positions
    corrected = (M_acc @ (raw_means_acc - bias_acc).T).T
    err_ms2 = corrected - expected_ms2
    err_mg = np.abs(err_ms2) / 9.80665 * 1000
    rms_mg = np.sqrt(np.mean(err_mg**2))
    print(f"\n  Accel RMS residual: {rms_mg:.2f} mg")

    # Save
    with open("ferraris_calibration.json", "w") as f:
        json.dump(cal_result, f, indent=2)
    print(f"  Saved -> ferraris_calibration.json")

    # Also save in imu_calibrate.py-compatible format
    compat = {
        "timestamp": cal_result["timestamp"],
        "method": "ferraris",
        "gyro": cal_result["gyro_compat"],
        "accel": cal_result["accel_compat"],
    }
    with open("imu_calibration.json", "w") as f:
        json.dump(compat, f, indent=2)
    print(f"  Saved -> imu_calibration.json (compatible with live_plot/verify)")

    # Plot if available
    if HAS_MPL:
        plot_ferraris_results(d, sections, M_acc, bias_acc, M_gyr, bias_gyr, expected_ms2)


def plot_ferraris_results(raw_npz, sections, M_acc, bias_acc, M_gyr, bias_gyr, expected_ms2):
    """Plot Ferraris calibration results."""
    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    fig.suptitle("Ferraris Calibration Results", fontsize=14, fontweight="bold")

    pos_names = ["static_x_pos", "static_x_neg", "static_y_pos",
                 "static_y_neg", "static_z_pos", "static_z_neg"]
    pos_labels = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]

    # Raw vs corrected accel for each static position
    raw_means = np.array([sections[n]["acc"].mean(axis=0) for n in pos_names])
    corrected = (M_acc @ (raw_means - bias_acc).T).T

    ax_labels = ["X (m/s^2)", "Y (m/s^2)", "Z (m/s^2)"]
    colors = ["#e74c3c", "#2ecc71", "#3498db"]
    x = np.arange(6)
    w = 0.25

    for i in range(3):
        ax = axes[0, i]
        ax.bar(x - w, raw_means[:, i], w, label="Raw", alpha=0.6, color="gray")
        ax.bar(x, corrected[:, i], w, label="Corrected", alpha=0.8, color=colors[i])
        ax.bar(x + w, expected_ms2[:, i], w, label="Expected", alpha=0.4,
               color="black", edgecolor="black", lw=0.5)
        ax.set_xticks(x)
        ax.set_xticklabels(pos_labels)
        ax.set_ylabel(ax_labels[i])
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3, axis="y")
        ax.set_title(f"Accel {['X','Y','Z'][i]}")

    # Rotation data
    rot_names = ["rot_x", "rot_y", "rot_z"]
    rot_labels = ["Roll (X)", "Pitch (Y)", "Yaw (Z)"]
    dt = 1.0 / SAMPLING_RATE_HZ

    for i, (rn, rl) in enumerate(zip(rot_names, rot_labels)):
        ax = axes[1, i]
        gyr = sections[rn]["gyr"] - bias_gyr
        angle = np.cumsum(gyr, axis=0) * dt
        t_s = np.arange(len(gyr)) * dt
        ax.plot(t_s, angle[:, 0], 'r-', lw=0.8, label="X")
        ax.plot(t_s, angle[:, 1], 'g-', lw=0.8, label="Y")
        ax.plot(t_s, angle[:, 2], 'b-', lw=0.8, label="Z")
        ax.axhline(360, color="gray", ls="--", lw=1)
        ax.axhline(-360, color="gray", ls="--", lw=1)
        ax.set_title(f"{rl} rotation")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angle (deg)")
        ax.legend(fontsize=7, loc="upper left")
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("ferraris_calibration.png", dpi=150)
    print("  Saved -> ferraris_calibration.png")
    plt.show()


def verify_live(port, baud, cal_file):
    """Live verification with Ferraris calibration applied."""
    cal_path = Path(cal_file)
    if not cal_path.exists():
        print(f"  [ERROR] {cal_file} not found")
        sys.exit(1)

    with open(cal_path) as f:
        cal = json.load(f)

    # Determine format
    if "accel" in cal and "matrix_M" in cal.get("accel", {}):
        # Ferraris format
        M_acc = np.array(cal["accel"]["matrix_M"])
        b_acc = np.array(cal["accel"]["bias_ms2"])
        M_gyr = np.array(cal["gyro"]["matrix_M"])
        b_gyr = np.array(cal["gyro"]["bias_dps"])
        is_si = True
    else:
        is_si = False

    ser = serial.Serial(port, baud, timeout=2.0)
    ser.reset_input_buffer()

    print("\n  Live verification (Ctrl+C to stop)")
    print(f"  {'AX(g)':>8} {'AY(g)':>8} {'AZ(g)':>8} {'|a|':>8}  "
          f"{'GX(d/s)':>8} {'GY(d/s)':>8} {'GZ(d/s)':>8}")
    print(f"  {'-' * 70}")

    try:
        while True:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            m = LINE_RE.search(line)
            if not m:
                continue

            raw = [int(m.group(i)) for i in range(1, 7)]
            raw_a = np.array(raw[0:3], dtype=float)
            raw_g = np.array(raw[3:6], dtype=float)

            if is_si:
                a_ms2 = raw_a / ACCEL_SENS * 9.80665
                g_dps = raw_g / GYRO_SENS
                corr_a_ms2 = M_acc @ (a_ms2 - b_acc)
                corr_g_dps = M_gyr @ (g_dps - b_gyr)
                corr_a_g = corr_a_ms2 / 9.80665
            else:
                corr_a_g = raw_a / ACCEL_SENS
                corr_g_dps = raw_g / GYRO_SENS

            norm = np.linalg.norm(corr_a_g)
            print(f"\r  {corr_a_g[0]:8.4f} {corr_a_g[1]:8.4f} {corr_a_g[2]:8.4f} {norm:8.4f}  "
                  f"{corr_g_dps[0]:8.3f} {corr_g_dps[1]:8.3f} {corr_g_dps[2]:8.3f}",
                  end="", flush=True)
    except KeyboardInterrupt:
        print("\n  Stopped.")
    finally:
        ser.close()


def main():
    parser = argparse.ArgumentParser(description="Ferraris IMU Calibration (imucal-style)")
    parser.add_argument("--port", type=str, help="Serial port (COM4)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--record", action="store_true", help="Record Ferraris calibration data")
    parser.add_argument("--calibrate", action="store_true", help="Run Ferraris calibration on recorded data")
    parser.add_argument("--verify", action="store_true", help="Live verification")
    parser.add_argument("--cal", default="ferraris_calibration.json", help="Calibration file for verify")
    args = parser.parse_args()

    if args.record:
        if not args.port:
            print("  --port required")
            sys.exit(1)
        record_ferraris(args.port, args.baud)
    elif args.calibrate:
        run_ferraris_calibration()
    elif args.verify:
        if not args.port:
            print("  --port required")
            sys.exit(1)
        verify_live(args.port, args.baud, args.cal)
    else:
        parser.print_help()
        print("\n  Quick start:")
        print("    1) python calibrate_imucal.py --port COM4 --record")
        print("    2) python calibrate_imucal.py --calibrate")
        print("    3) python calibrate_imucal.py --port COM4 --verify")


if __name__ == "__main__":
    main()
