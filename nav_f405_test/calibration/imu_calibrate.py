"""
IIM-42652 IMU Calibration Suite
================================
High-quality calibration for the NAV F405 board using the 100 Hz binary
stream on USART6 (921600 baud) or the 2 Hz debug text on UART4 (115200).

Calibration steps (run in order for best results):
  1) Gyroscope bias        -- stationary, ~60 s
  2) Accel 6-position      -- bias + scale + cross-axis misalignment
  3) Temperature sweep     -- bias vs. temperature (optional, ~30 min)

Usage:
    python imu_calibrate.py --port COM3 --step gyro
    python imu_calibrate.py --port COM3 --step accel6
    python imu_calibrate.py --port COM3 --step tempsweep
    python imu_calibrate.py --port COM3 --step all
    python imu_calibrate.py --port COM3 --step verify

  Add  --source uart4  to use the slow debug text (default: usart6 binary).
"""

import argparse
import struct
import re
import sys
import time
import json
import numpy as np
import serial
from pathlib import Path
from datetime import datetime

# =====================================================================
#  IIM-42652 SENSITIVITY CONSTANTS (datasheet)
#  Configured: +/-16 g accel, +/-2000 dps gyro, 1 kHz ODR
# =====================================================================
ACCEL_SENS  = 2048.0      # LSB/g   at +/-16 g
GYRO_SENS   = 16.4        # LSB/dps at +/-2000 dps
TEMP_SENS   = 132.48      # LSB/deg C
TEMP_OFFSET = 25.0        # 0 LSB = 25 deg C

GRAVITY_MS2 = 9.80665     # m/s^2

# -- Binary packet format (USART6, 18 bytes) --
PKT_HEADER = 0xAA
PKT_FOOTER = 0x55
PKT_SIZE   = 18

# Regex for UART4 text lines
LINE_RE = re.compile(
    r"AX:\s*(-?\d+)\s+AY:\s*(-?\d+)\s+AZ:\s*(-?\d+)\s+"
    r"GX:\s*(-?\d+)\s+GY:\s*(-?\d+)\s+GZ:\s*(-?\d+)\s+"
    r"T:\s*(-?\d+)"
)


def raw_temp_to_c(raw: float) -> float:
    return (raw / TEMP_SENS) + TEMP_OFFSET


# =====================================================================
#  SERIAL READER -- supports both binary (USART6) and text (UART4)
# =====================================================================

class IMUReader:
    """Reads and parses IMU packets from the NAV board."""

    def __init__(self, port: str, source: str = "usart6"):
        self.source = source
        baud = 921600 if source == "usart6" else 115200
        try:
            self.ser = serial.Serial(port, baud, timeout=2.0)
            self.ser.reset_input_buffer()
            print(f"  [OK] {port} @ {baud} baud ({source})")
        except serial.SerialException as e:
            print(f"  [ERROR] Cannot open {port}: {e}")
            sys.exit(1)
        self._buf = bytearray()

    def close(self):
        self.ser.close()

    def read_one(self) -> dict | None:
        """Return one parsed sample or None on timeout."""
        if self.source == "usart6":
            return self._read_binary()
        else:
            return self._read_text()

    # -- Binary parser (USART6, 100 Hz) --
    def _read_binary(self) -> dict | None:
        avail = self.ser.in_waiting or 1
        self._buf += self.ser.read(avail)

        while len(self._buf) >= PKT_SIZE:
            idx = self._buf.find(PKT_HEADER)
            if idx < 0:
                self._buf.clear()
                return None
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < PKT_SIZE:
                return None

            pkt = bytes(self._buf[:PKT_SIZE])
            del self._buf[:PKT_SIZE]

            if pkt[17] != PKT_FOOTER:
                continue
            chk = 0
            for i in range(16):
                chk ^= pkt[i]
            if chk != pkt[16]:
                continue

            ax = struct.unpack(">h", pkt[2:4])[0]
            ay = struct.unpack(">h", pkt[4:6])[0]
            az = struct.unpack(">h", pkt[6:8])[0]
            gx = struct.unpack(">h", pkt[8:10])[0]
            gy = struct.unpack(">h", pkt[10:12])[0]
            gz = struct.unpack(">h", pkt[12:14])[0]
            temp = struct.unpack(">h", pkt[14:16])[0]

            return {"ax": ax, "ay": ay, "az": az,
                    "gx": gx, "gy": gy, "gz": gz,
                    "temp_raw": temp, "status": pkt[1]}
        return None

    # -- Text parser (UART4, ~2 Hz) --
    def _read_text(self) -> dict | None:
        line = self.ser.readline().decode("ascii", errors="ignore").strip()
        m = LINE_RE.search(line)
        if m:
            return {
                "ax": int(m.group(1)), "ay": int(m.group(2)), "az": int(m.group(3)),
                "gx": int(m.group(4)), "gy": int(m.group(5)), "gz": int(m.group(6)),
                "temp_raw": int(m.group(7)), "status": 1,
            }
        return None


# =====================================================================
#  DATA COLLECTOR
# =====================================================================

def collect(reader: IMUReader, n: int, label: str = "") -> np.ndarray:
    """
    Collect n valid samples.  Returns (n, 7) array:
    columns = [ax, ay, az, gx, gy, gz, temp_raw]
    """
    data = []
    fails = 0
    t0 = time.time()
    tag = f" -- {label}" if label else ""
    print(f"\n  Collecting {n} samples{tag} ...")

    while len(data) < n:
        s = reader.read_one()
        if s is None:
            fails += 1
            if fails > n * 5:
                print("\n  [ERROR] Too many read failures. Check wiring.")
                sys.exit(1)
            continue
        data.append([s["ax"], s["ay"], s["az"],
                     s["gx"], s["gy"], s["gz"], s["temp_raw"]])
        pct = len(data) * 100 // n
        bar = "#" * (pct // 2) + "-" * (50 - pct // 2)
        elapsed = time.time() - t0
        print(f"\r  [{bar}] {pct:3d}%  {len(data)}/{n}  ({elapsed:.0f}s)", end="", flush=True)

    elapsed = time.time() - t0
    print(f"\n  Done in {elapsed:.1f}s  ({len(data) / elapsed:.1f} samples/s)")
    return np.array(data, dtype=np.float64)


# =====================================================================
#  STEP 1:  GYROSCOPE BIAS CALIBRATION
# =====================================================================

def step_gyro_bias(reader: IMUReader, n: int = 6000) -> dict:
    """
    Gyro bias -- device perfectly stationary for ~60 s at 100 Hz.
    Also computes Allan deviation from the same data.
    """
    print("\n" + "=" * 60)
    print("  STEP 1: GYROSCOPE BIAS CALIBRATION")
    print("=" * 60)
    print("  > Place the board on a FLAT, VIBRATION-FREE surface.")
    print("  > Do NOT touch the board during acquisition.")
    print(f"  > {n} samples = ~{n / 100:.0f}s at 100 Hz")
    input("\n  Press ENTER when ready > ")

    data = collect(reader, n, "gyro bias (stationary)")
    gx, gy, gz = data[:, 3], data[:, 4], data[:, 5]
    temp = data[:, 6]

    bias_lsb = np.array([gx.mean(), gy.mean(), gz.mean()])
    bias_dps = bias_lsb / GYRO_SENS
    std_lsb  = np.array([gx.std(), gy.std(), gz.std()])
    std_dps  = std_lsb / GYRO_SENS
    mean_temp = raw_temp_to_c(temp.mean())

    # Allan deviation (quick estimate from this data)
    dt = 1.0 / 100.0  # ~100 Hz (both usart6 binary and uart4 text)
    allan = {}
    for i, name in enumerate(["gx", "gy", "gz"]):
        rate = data[:, 3 + i] / GYRO_SENS
        angle = np.cumsum(rate) * dt
        max_k = int(np.floor(np.log2(len(rate) / 2)))
        ms = 2 ** np.arange(1, max_k + 1)
        taus, adevs = [], []
        for m in ms:
            nn = len(angle)
            if nn < 2 * m:
                break
            d = angle[2*m:nn] - 2*angle[m:nn-m] + angle[:nn-2*m]
            taus.append(float(m * dt))
            adevs.append(float(np.sqrt(np.mean(d**2) / (2 * (m * dt)**2))))
        allan[name] = {"tau": taus, "adev_dps": adevs}

    print(f"\n  -- Gyro Bias (T = {mean_temp:.1f} C) --")
    print(f"  {'Axis':<5} {'Bias (LSB)':>11} {'Bias (d/s)':>11} {'Noise s (d/s)':>14}")
    print(f"  {'-' * 45}")
    for i, ax in enumerate(["GX", "GY", "GZ"]):
        print(f"  {ax:<5} {bias_lsb[i]:11.2f} {bias_dps[i]:11.4f} {std_dps[i]:14.4f}")

    # Save raw gyro data for external analysis
    np.save("gyro_raw_stationary.npy", data)
    print("  Raw data saved -> gyro_raw_stationary.npy")

    return {
        "gyro_bias_lsb": bias_lsb.tolist(),
        "gyro_bias_dps": bias_dps.tolist(),
        "gyro_std_dps": std_dps.tolist(),
        "temperature_c": mean_temp,
        "n_samples": n,
        "allan": allan,
    }


# =====================================================================
#  STEP 2:  ACCELEROMETER 6-POSITION (bias + scale + cross-axis)
# =====================================================================

POSITIONS_6 = [
    ("Z-UP",    np.array([ 0,  0, +1], dtype=float), "Board flat -- Z pointing UP (normal)"),
    ("Z-DOWN",  np.array([ 0,  0, -1], dtype=float), "Board flipped -- Z pointing DOWN"),
    ("X-UP",    np.array([+1,  0,  0], dtype=float), "Board on edge -- X pointing UP"),
    ("X-DOWN",  np.array([-1,  0,  0], dtype=float), "Board on edge -- X pointing DOWN"),
    ("Y-UP",    np.array([ 0, +1,  0], dtype=float), "Board on edge -- Y pointing UP"),
    ("Y-DOWN",  np.array([ 0, -1,  0], dtype=float), "Board on edge -- Y pointing DOWN"),
]


def step_accel_6pos(reader: IMUReader, n_per_pos: int = 3000) -> dict:
    """
    Full 6-position accelerometer calibration.

    Solves a 12-parameter model:
        a_corrected = M @ (a_raw - b)

    where M is a 3x3 matrix (scale + cross-axis)
    and b is the 3x1 bias vector.

    This corrects: offset, scale error, and cross-axis misalignment.
    """
    print("\n" + "=" * 60)
    print("  STEP 2: ACCELEROMETER 6-POSITION CALIBRATION")
    print("=" * 60)
    print("  You will orient the board in 6 positions.")
    print("  Hold each position perfectly still on a flat surface.")
    print(f"  {n_per_pos} samples per position = ~{n_per_pos / 100:.0f}s each\n")

    raw_means = []
    expected_g = []

    for pos_name, g_vec, instruction in POSITIONS_6:
        print(f"  === Position: {pos_name} ===")
        print(f"  {instruction}")
        print(f"  Expected gravity: ax={g_vec[0]:+.0f}g  ay={g_vec[1]:+.0f}g  az={g_vec[2]:+.0f}g")
        input(f"  Press ENTER when stable > ")

        data = collect(reader, n_per_pos, pos_name)
        mean_a = data[:, 0:3].mean(axis=0)
        std_a  = data[:, 0:3].std(axis=0)
        raw_means.append(mean_a)
        expected_g.append(g_vec)

        print(f"  Mean: AX={mean_a[0]:8.1f}  AY={mean_a[1]:8.1f}  AZ={mean_a[2]:8.1f}")
        print(f"  Std:  AX={std_a[0]:8.1f}  AY={std_a[1]:8.1f}  AZ={std_a[2]:8.1f}\n")

    raw_means  = np.array(raw_means)
    expected_g = np.array(expected_g)
    expected_lsb = expected_g * ACCEL_SENS

    # -- Method 1: Simple per-axis bias + scale (least squares) --
    bias_simple  = np.zeros(3)
    scale_simple = np.zeros(3)
    for j in range(3):
        A = np.column_stack([raw_means[:, j], np.ones(6)])
        y = expected_lsb[:, j]
        res, _, _, _ = np.linalg.lstsq(A, y, rcond=None)
        scale_simple[j] = res[0]
        bias_simple[j]  = -res[1] / res[0]

    # -- Method 2: Full 3x3 misalignment matrix (12-parameter model) --
    # Model:  expected = M @ (raw - b)
    # Rearrange:  expected = M @ raw - M @ b = M @ raw + c
    #             where c = -M @ b
    # For each output axis i:
    #   E[:, i] = [raw_means, 1] @ [M[i,:], c[i]]^T

    M_full = np.zeros((3, 3))
    c_full = np.zeros(3)
    for i in range(3):
        A = np.column_stack([raw_means, np.ones(6)])  # (6, 4)
        y = expected_lsb[:, i]                         # (6,)
        res, _, _, _ = np.linalg.lstsq(A, y, rcond=None)
        M_full[i, :] = res[:3]
        c_full[i]    = res[3]

    # Recover bias:  c = -M @ b  ->  b = -M^{-1} @ c
    bias_full = -np.linalg.inv(M_full) @ c_full

    # Verify: compute corrected values and residual
    corrected_simple = scale_simple * (raw_means - bias_simple)
    corrected_full   = (M_full @ (raw_means - bias_full).T).T

    err_simple_g = (corrected_simple - expected_lsb) / ACCEL_SENS
    err_full_g   = (corrected_full - expected_lsb) / ACCEL_SENS
    rms_simple = np.sqrt(np.mean(err_simple_g**2)) * 1000  # mg
    rms_full   = np.sqrt(np.mean(err_full_g**2)) * 1000

    print("  -- Simple (per-axis bias + scale) --")
    print(f"  {'Axis':<5} {'Bias (LSB)':>11} {'Scale':>10}")
    print(f"  {'-' * 30}")
    for i, ax in enumerate(["AX", "AY", "AZ"]):
        print(f"  {ax:<5} {bias_simple[i]:11.2f} {scale_simple[i]:10.6f}")
    print(f"  RMS error: {rms_simple:.2f} mg")

    print("\n  -- Full misalignment matrix --")
    print("  M =")
    for row in M_full:
        print(f"    [{row[0]:10.6f}  {row[1]:10.6f}  {row[2]:10.6f}]")
    print(f"  bias = [{bias_full[0]:.2f}, {bias_full[1]:.2f}, {bias_full[2]:.2f}] LSB")
    print(f"  RMS error: {rms_full:.2f} mg")

    # Cross-axis angles
    diag = np.diag(M_full)
    print("\n  -- Cross-axis misalignment angles --")
    pairs = [("XY", M_full[0,1]/diag[0]), ("XZ", M_full[0,2]/diag[0]),
             ("YX", M_full[1,0]/diag[1]), ("YZ", M_full[1,2]/diag[1]),
             ("ZX", M_full[2,0]/diag[2]), ("ZY", M_full[2,1]/diag[2])]
    for name, val in pairs:
        angle_deg = np.degrees(np.arcsin(np.clip(val, -1, 1)))
        print(f"  {name}: {angle_deg:+.3f} deg")

    np.save("accel_6pos_raw.npy", raw_means)

    return {
        "simple": {
            "accel_bias_lsb": bias_simple.tolist(),
            "accel_scale": scale_simple.tolist(),
            "rms_mg": rms_simple,
        },
        "full": {
            "misalignment_matrix": M_full.tolist(),
            "accel_bias_lsb": bias_full.tolist(),
            "rms_mg": rms_full,
        },
        "raw_means_lsb": raw_means.tolist(),
        "expected_g": expected_g.tolist(),
    }


# =====================================================================
#  STEP 3:  TEMPERATURE SWEEP -- bias vs temperature
# =====================================================================

def step_temp_sweep(reader: IMUReader, duration_min: int = 30,
                    sample_interval_s: float = 1.0) -> dict:
    """
    Collect bias data over a temperature range.
    Start cold, let it warm up naturally or use a heat gun.
    Board must be STATIONARY the entire time.
    Fits a linear model:  bias(T) = a * T + b  per axis.
    """
    print("\n" + "=" * 60)
    print("  STEP 3: TEMPERATURE SWEEP CALIBRATION")
    print("=" * 60)
    print("  > Board must be STATIONARY the entire time.")
    print("  > Start at a cool temperature if possible.")
    print("  > Let it warm up naturally or apply gentle heat.")
    print(f"  > Duration: {duration_min} minutes")
    print("  > TIP: Cool the board first, then let it warm.\n")
    input("  Press ENTER to start > ")

    n_points = int(duration_min * 60 / sample_interval_s)
    window = max(10, int(sample_interval_s * 100))

    temperatures = []
    gyro_biases  = []
    accel_biases = []
    timestamps   = []

    t0 = time.time()
    for i in range(n_points):
        chunk = collect(reader, window, f"T-sweep {i+1}/{n_points}")
        ax_m, ay_m, az_m = chunk[:, 0].mean(), chunk[:, 1].mean(), chunk[:, 2].mean()
        gx_m, gy_m, gz_m = chunk[:, 3].mean(), chunk[:, 4].mean(), chunk[:, 5].mean()
        t_m = raw_temp_to_c(chunk[:, 6].mean())

        temperatures.append(t_m)
        gyro_biases.append([gx_m, gy_m, gz_m])
        accel_biases.append([ax_m, ay_m, az_m])
        timestamps.append(time.time() - t0)

        elapsed = time.time() - t0
        remain = max(0, duration_min * 60 - elapsed)
        print(f"  T={t_m:.1f}C  GX={gx_m:.1f} GY={gy_m:.1f} GZ={gz_m:.1f}  "
              f"({elapsed/60:.1f}/{duration_min} min, {remain/60:.1f} min left)")

        if remain <= 0:
            break

    temperatures = np.array(temperatures)
    gyro_biases  = np.array(gyro_biases)
    accel_biases = np.array(accel_biases)

    # Fit linear model per axis
    results = {"gyro_temp_model": {}, "accel_temp_model": {}}
    print("\n  -- Temperature Compensation Coefficients --")

    for i, name in enumerate(["GX", "GY", "GZ"]):
        coeffs = np.polyfit(temperatures, gyro_biases[:, i], 1)
        results["gyro_temp_model"][name] = {
            "slope_lsb_per_c": float(coeffs[0]),
            "intercept_lsb": float(coeffs[1]),
        }
        slope_dps = coeffs[0] / GYRO_SENS
        print(f"  {name}: bias(T) = {coeffs[0]:.3f} * T + {coeffs[1]:.1f} "
              f"({slope_dps:.4f} d/s per C)")

    for i, name in enumerate(["AX", "AY", "AZ"]):
        coeffs = np.polyfit(temperatures, accel_biases[:, i], 1)
        results["accel_temp_model"][name] = {
            "slope_lsb_per_c": float(coeffs[0]),
            "intercept_lsb": float(coeffs[1]),
        }
        slope_mg = coeffs[0] / ACCEL_SENS * 1000
        print(f"  {name}: bias(T) = {coeffs[0]:.3f} * T + {coeffs[1]:.1f} "
              f"({slope_mg:.2f} mg per C)")

    temp_range = temperatures.max() - temperatures.min()
    print(f"\n  Temperature range: {temperatures.min():.1f} -- {temperatures.max():.1f} C"
          f"  (dT = {temp_range:.1f} C)")
    if temp_range < 10:
        print("  WARNING: dT < 10 C -- temperature model may be unreliable.")

    results["temperatures_c"]  = temperatures.tolist()
    results["gyro_biases_lsb"] = gyro_biases.tolist()
    results["accel_biases_lsb"] = accel_biases.tolist()
    results["timestamps_s"]    = timestamps

    np.savez("temp_sweep_raw.npz",
             temperatures=temperatures, gyro=gyro_biases,
             accel=accel_biases, timestamps=timestamps)
    print("  Raw data saved -> temp_sweep_raw.npz")

    return results


# =====================================================================
#  VERIFY:  Apply calibration and show live corrected data
# =====================================================================

def step_verify(reader: IMUReader, cal_file: str = "imu_calibration.json"):
    """Load calibration and display live corrected values."""
    print("\n" + "=" * 60)
    print("  VERIFICATION -- live corrected data")
    print("=" * 60)

    try:
        with open(cal_file) as f:
            cal = json.load(f)
    except FileNotFoundError:
        print(f"  [ERROR] {cal_file} not found. Run calibration first.")
        return

    gb = np.array(cal.get("gyro", {}).get("gyro_bias_lsb", [0, 0, 0]))

    accel_cal = cal.get("accel", {})
    if "full" in accel_cal:
        M = np.array(accel_cal["full"]["misalignment_matrix"])
        ab = np.array(accel_cal["full"]["accel_bias_lsb"])
        use_full = True
    elif "simple" in accel_cal:
        ab = np.array(accel_cal["simple"]["accel_bias_lsb"])
        sc = np.array(accel_cal["simple"]["accel_scale"])
        use_full = False
    else:
        ab = np.array(accel_cal.get("accel_bias_lsb", [0, 0, 0]))
        sc = np.array(accel_cal.get("accel_scale", [1, 1, 1]))
        use_full = False

    temp_model = cal.get("temp_sweep", {})
    gyro_temp = temp_model.get("gyro_temp_model", {})
    has_temp_comp = len(gyro_temp) > 0

    print("  Showing corrected g (accel) and d/s (gyro).")
    print("  Hold board flat -> expect AX~0  AY~0  AZ~1.00 g")
    print("  Press Ctrl+C to stop.\n")

    print(f"  {'AX(g)':>8} {'AY(g)':>8} {'AZ(g)':>8} {'|a|(g)':>8}  "
          f"{'GXd/s':>8} {'GYd/s':>8} {'GZd/s':>8}  {'T C':>6}")
    print(f"  {'-' * 75}")

    try:
        while True:
            s = reader.read_one()
            if s is None:
                continue

            raw_a = np.array([s["ax"], s["ay"], s["az"]], dtype=float)
            raw_g = np.array([s["gx"], s["gy"], s["gz"]], dtype=float)
            temp_c = raw_temp_to_c(s["temp_raw"])

            if has_temp_comp:
                gb_t = np.array([
                    gyro_temp["GX"]["slope_lsb_per_c"] * temp_c + gyro_temp["GX"]["intercept_lsb"],
                    gyro_temp["GY"]["slope_lsb_per_c"] * temp_c + gyro_temp["GY"]["intercept_lsb"],
                    gyro_temp["GZ"]["slope_lsb_per_c"] * temp_c + gyro_temp["GZ"]["intercept_lsb"],
                ])
                corr_g = (raw_g - gb_t) / GYRO_SENS
            else:
                corr_g = (raw_g - gb) / GYRO_SENS

            if use_full:
                corr_a = (M @ (raw_a - ab)) / ACCEL_SENS
            else:
                corr_a = sc * (raw_a - ab) / ACCEL_SENS

            norm_a = np.linalg.norm(corr_a)

            print(f"\r  {corr_a[0]:8.4f} {corr_a[1]:8.4f} {corr_a[2]:8.4f} {norm_a:8.4f}  "
                  f"{corr_g[0]:8.3f} {corr_g[1]:8.3f} {corr_g[2]:8.3f}  {temp_c:6.1f}",
                  end="", flush=True)
    except KeyboardInterrupt:
        print("\n\n  Verification stopped.")


# =====================================================================
#  MAIN
# =====================================================================

def main():
    parser = argparse.ArgumentParser(description="IIM-42652 IMU Calibration Suite")
    parser.add_argument("--port", required=True, help="Serial port (COM3, /dev/ttyUSB0)")
    parser.add_argument("--source", choices=["usart6", "uart4"], default="uart4",
                        help="Data source: usart6 (binary 100Hz) or uart4 (text 100Hz)")
    parser.add_argument("--step", choices=["gyro", "accel6", "tempsweep", "all", "verify"],
                        default="all", help="Calibration step to run")
    parser.add_argument("--samples", type=int, default=3000,
                        help="Samples per position/step (default: 3000)")
    parser.add_argument("--output", default="imu_calibration.json",
                        help="Output JSON file")
    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("  IIM-42652 IMU CALIBRATION SUITE")
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 60)

    reader = IMUReader(args.port, args.source)

    time.sleep(0.3)
    reader.ser.reset_input_buffer()
    print("\n  Verifying data stream...")
    test = None
    for _ in range(50):
        test = reader.read_one()
        if test:
            break
    if not test:
        print("  [ERROR] No valid IMU data received. Check port and firmware.")
        sys.exit(1)
    print(f"  [OK] AX={test['ax']} AY={test['ay']} AZ={test['az']} "
          f"GX={test['gx']} GY={test['gy']} GZ={test['gz']} "
          f"T={raw_temp_to_c(test['temp_raw']):.1f} C")

    # Load existing cal (to merge steps)
    out_path = Path(args.output)
    cal = {}
    if out_path.exists():
        with open(out_path) as f:
            cal = json.load(f)
        print(f"  Loaded existing calibration from {out_path.name}")

    cal["timestamp"] = datetime.now().isoformat()
    cal["source"] = args.source

    # -- Run requested step(s) --
    if args.step in ("gyro", "all"):
        cal["gyro"] = step_gyro_bias(reader, args.samples)

    if args.step in ("accel6", "all"):
        cal["accel"] = step_accel_6pos(reader, args.samples)

    if args.step in ("tempsweep", "all"):
        if args.step == "all":
            print("\n  Temperature sweep takes ~30 min.")
            ans = input("  Run temp sweep now? (y/N): ").strip().lower()
            if ans != "y":
                print("  Skipping temp sweep. Run later with --step tempsweep")
            else:
                cal["temp_sweep"] = step_temp_sweep(reader)
        else:
            cal["temp_sweep"] = step_temp_sweep(reader)

    if args.step == "verify":
        step_verify(reader, args.output)
        reader.close()
        return

    # -- Save --
    with open(out_path, "w") as f:
        json.dump(cal, f, indent=2)

    print(f"\n{'=' * 60}")
    print(f"  Calibration saved -> {out_path.resolve()}")
    print(f"{'=' * 60}")

    # -- Summary for firmware --
    print("\n  -- HOW TO APPLY IN FIRMWARE --\n")

    if "gyro" in cal:
        gb = cal["gyro"]["gyro_bias_lsb"]
        print("  Gyro correction (subtract bias from raw):")
        for i, ax in enumerate(["X", "Y", "Z"]):
            print(f"    gyro_{ax.lower()}_corr = gyro_{ax.lower()}_raw - ({gb[i]:.1f});")

    if "accel" in cal and "full" in cal["accel"]:
        M  = np.array(cal["accel"]["full"]["misalignment_matrix"])
        ab = np.array(cal["accel"]["full"]["accel_bias_lsb"])
        print("\n  Accel correction (full 3x3 misalignment):")
        print(f"    float bx={ab[0]:.1f}f, by={ab[1]:.1f}f, bz={ab[2]:.1f}f;")
        print(f"    float M[3][3] = {{")
        for r in range(3):
            print(f"      {{{M[r,0]:.6f}f, {M[r,1]:.6f}f, {M[r,2]:.6f}f}},")
        print(f"    }};")
        print("    // corr[i] = M[i][0]*(ax-bx) + M[i][1]*(ay-by) + M[i][2]*(az-bz);")

    reader.close()
    print("\n  [DONE]\n")


if __name__ == "__main__":
    main()
