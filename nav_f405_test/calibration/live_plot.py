"""
IMU Real-Time Live Plotter
===========================
Streams IMU data from UART4 and displays scrolling plots in real time.
Shows both raw and (optionally) calibration-corrected data.

How to know if calibration is correct:
  - Accel norm |a| should be a flat line at 1.00 g when stationary
  - Gyro should hover near 0 deg/s when stationary
  - Accel X,Y should be near 0 when board is flat, Z near +1.0 g
  - Rotating around one axis should only move that gyro channel

Usage:
    python live_plot.py --port COM4
    python live_plot.py --port COM4 --cal imu_calibration.json
    python live_plot.py --port COM4 --seconds 30
"""

import argparse
import sys
import re
import time
import json
import collections
import numpy as np
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pathlib import Path

# ── Constants ──
ACCEL_SENS  = 2048.0
GYRO_SENS   = 16.4
TEMP_SENS   = 132.48
TEMP_OFFSET = 25.0

# Single-line format  (F405 direct  or  H753 UART path)
#   AX:  123 AY:  456 AZ:  789 GX:  12 GY:  34 GZ:  56 T:1234
#   [UART 1] st=0x01  AX=  123 AY=  456 ...  T=1234
LINE_RE = re.compile(
    r"AX[=:]\s*(-?\d+)\s+AY[=:]\s*(-?\d+)\s+AZ[=:]\s*(-?\d+)\s+"
    r"GX[=:]\s*(-?\d+)\s+GY[=:]\s*(-?\d+)\s+GZ[=:]\s*(-?\d+)\s+"
    r"T[=:]\s*(-?\d+)"
)

# Multi-line format  (H753 SPI debug path via IMU_PrintData)
#   [1] IMU  status=0x01
#     Accel  X=   113  Y= -1348  Z=  1483
#     Gyro   X=    34  Y=   125  Z=    -4
#     Temp   402
ACCEL_RE = re.compile(r"Accel\s+X=\s*(-?\d+)\s+Y=\s*(-?\d+)\s+Z=\s*(-?\d+)")
GYRO_RE  = re.compile(r"Gyro\s+X=\s*(-?\d+)\s+Y=\s*(-?\d+)\s+Z=\s*(-?\d+)")
TEMP_RE  = re.compile(r"Temp\s+(-?\d+)")


def raw_temp_to_c(raw):
    return (raw / TEMP_SENS) + TEMP_OFFSET


class LivePlotter:
    def __init__(self, port, baud=115200, window_s=10, cal_file=None):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.ser.reset_input_buffer()
        self.window = int(window_s * 100)  # 100 Hz * seconds

        # Ring buffers
        self.t   = collections.deque(maxlen=self.window)
        self.ax  = collections.deque(maxlen=self.window)
        self.ay  = collections.deque(maxlen=self.window)
        self.az  = collections.deque(maxlen=self.window)
        self.gx  = collections.deque(maxlen=self.window)
        self.gy  = collections.deque(maxlen=self.window)
        self.gz  = collections.deque(maxlen=self.window)
        self.anorm = collections.deque(maxlen=self.window)
        self.temp = collections.deque(maxlen=self.window)
        self.t0 = time.time()

        # Load calibration if provided
        self.cal = None
        self.use_full = False
        self.M = None
        self.ab = np.zeros(3)
        self.gb = np.zeros(3)
        self.accel_scale = np.ones(3)

        # Multi-line parse state (H753 format)
        self._partial = None

        if cal_file and Path(cal_file).exists():
            with open(cal_file) as f:
                self.cal = json.load(f)
            self.gb = np.array(self.cal.get("gyro", {}).get("gyro_bias_lsb", [0, 0, 0]))
            accel_cal = self.cal.get("accel", {})
            if "full" in accel_cal:
                self.M = np.array(accel_cal["full"]["misalignment_matrix"])
                self.ab = np.array(accel_cal["full"]["accel_bias_lsb"])
                self.use_full = True
            elif "simple" in accel_cal:
                self.ab = np.array(accel_cal["simple"]["accel_bias_lsb"])
                self.accel_scale = np.array(accel_cal["simple"]["accel_scale"])
            print(f"  [CAL] Loaded calibration from {cal_file}")
        else:
            print(f"  [RAW] No calibration -- showing raw converted data")

    def read_samples(self):
        """Read all available samples from serial buffer.
        Handles two formats:
          - Single-line (F405 / H753 UART path): AX: val AY: val ...
          - Multi-line  (H753 SPI debug path):   Accel X= / Gyro X= / Temp lines
        """
        samples = []
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode("ascii", errors="ignore").strip()
            except Exception:
                break

            # ── Try single-line format first ──
            m = LINE_RE.search(line)
            if m:
                samples.append({
                    "ax": int(m.group(1)), "ay": int(m.group(2)), "az": int(m.group(3)),
                    "gx": int(m.group(4)), "gy": int(m.group(5)), "gz": int(m.group(6)),
                    "temp_raw": int(m.group(7)),
                })
                continue

            # ── Try multi-line H753 format ──
            ma = ACCEL_RE.search(line)
            if ma:
                self._partial = {"ax": int(ma.group(1)),
                                 "ay": int(ma.group(2)),
                                 "az": int(ma.group(3))}
                continue

            mg = GYRO_RE.search(line)
            if mg and hasattr(self, '_partial') and self._partial:
                self._partial["gx"] = int(mg.group(1))
                self._partial["gy"] = int(mg.group(2))
                self._partial["gz"] = int(mg.group(3))
                continue

            mt = TEMP_RE.search(line)
            if mt and hasattr(self, '_partial') and self._partial and "gx" in self._partial:
                self._partial["temp_raw"] = int(mt.group(1))
                samples.append(self._partial)
                self._partial = None

        return samples

    def apply_cal(self, raw_a, raw_g):
        """Apply calibration to raw data."""
        if self.cal:
            corr_g = (raw_g - self.gb) / GYRO_SENS
            if self.use_full:
                corr_a = (self.M @ (raw_a - self.ab)) / ACCEL_SENS
            else:
                corr_a = self.accel_scale * (raw_a - self.ab) / ACCEL_SENS
        else:
            corr_a = raw_a / ACCEL_SENS
            corr_g = raw_g / GYRO_SENS
        return corr_a, corr_g

    def run(self):
        """Launch the live matplotlib plot."""
        fig = plt.figure(figsize=(14, 10))
        fig.canvas.manager.set_window_title("IMU Live Monitor")

        # 4 subplots: Accel, Gyro, Accel Norm, Temperature
        gs = fig.add_gridspec(4, 1, hspace=0.35)
        ax1 = fig.add_subplot(gs[0])
        ax2 = fig.add_subplot(gs[1])
        ax3 = fig.add_subplot(gs[2])
        ax4 = fig.add_subplot(gs[3])

        cal_label = " (calibrated)" if self.cal else " (raw)"

        # Accel lines
        ax1.set_title("Accelerometer" + cal_label, fontweight="bold")
        ax1.set_ylabel("g")
        ax1.set_ylim(-2.5, 2.5)
        ax1.axhline(0, color="gray", linewidth=0.5)
        ax1.axhline(1, color="gray", linewidth=0.5, linestyle="--")
        ax1.axhline(-1, color="gray", linewidth=0.5, linestyle="--")
        ax1.grid(True, alpha=0.3)
        line_ax, = ax1.plot([], [], 'r-', linewidth=0.8, label="AX")
        line_ay, = ax1.plot([], [], 'g-', linewidth=0.8, label="AY")
        line_az, = ax1.plot([], [], 'b-', linewidth=0.8, label="AZ")
        ax1.legend(loc="upper right", fontsize=8, ncol=3)

        # Gyro lines
        ax2.set_title("Gyroscope" + cal_label, fontweight="bold")
        ax2.set_ylabel("deg/s")
        ax2.set_ylim(-50, 50)
        ax2.axhline(0, color="gray", linewidth=0.5)
        ax2.grid(True, alpha=0.3)
        line_gx, = ax2.plot([], [], 'r-', linewidth=0.8, label="GX")
        line_gy, = ax2.plot([], [], 'g-', linewidth=0.8, label="GY")
        line_gz, = ax2.plot([], [], 'b-', linewidth=0.8, label="GZ")
        ax2.legend(loc="upper right", fontsize=8, ncol=3)

        # Accel norm
        ax3.set_title("Accel Norm |a|  (should be ~1.0 g when still)", fontweight="bold")
        ax3.set_ylabel("g")
        ax3.set_ylim(0.8, 1.2)
        ax3.axhline(1.0, color="green", linewidth=1.5, linestyle="--", label="1.00 g")
        ax3.axhspan(0.99, 1.01, alpha=0.15, color="green", label="+/- 10 mg")
        ax3.grid(True, alpha=0.3)
        line_norm, = ax3.plot([], [], 'k-', linewidth=1.0, label="|a|")
        ax3.legend(loc="upper right", fontsize=8, ncol=3)

        # Temperature
        ax4.set_title("Temperature", fontweight="bold")
        ax4.set_ylabel("deg C")
        ax4.grid(True, alpha=0.3)
        line_temp, = ax4.plot([], [], 'm-', linewidth=1.0)
        ax4.set_xlabel("Time (s)")

        # Status text
        status_text = fig.text(0.02, 0.98, "", fontsize=9, verticalalignment="top",
                               fontfamily="monospace",
                               bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.8))

        def update(frame):
            samples = self.read_samples()
            for s in samples:
                now = time.time() - self.t0
                raw_a = np.array([s["ax"], s["ay"], s["az"]], dtype=float)
                raw_g = np.array([s["gx"], s["gy"], s["gz"]], dtype=float)
                corr_a, corr_g = self.apply_cal(raw_a, raw_g)

                self.t.append(now)
                self.ax.append(corr_a[0])
                self.ay.append(corr_a[1])
                self.az.append(corr_a[2])
                self.gx.append(corr_g[0])
                self.gy.append(corr_g[1])
                self.gz.append(corr_g[2])
                self.anorm.append(np.linalg.norm(corr_a))
                self.temp.append(raw_temp_to_c(s["temp_raw"]))

            if len(self.t) < 2:
                return []

            t_arr = list(self.t)
            tmin, tmax = t_arr[-1] - (self.window / 100.0), t_arr[-1]

            # Update data
            line_ax.set_data(t_arr, list(self.ax))
            line_ay.set_data(t_arr, list(self.ay))
            line_az.set_data(t_arr, list(self.az))
            line_gx.set_data(t_arr, list(self.gx))
            line_gy.set_data(t_arr, list(self.gy))
            line_gz.set_data(t_arr, list(self.gz))
            line_norm.set_data(t_arr, list(self.anorm))
            line_temp.set_data(t_arr, list(self.temp))

            # Update x limits
            for a in [ax1, ax2, ax3, ax4]:
                a.set_xlim(tmin, tmax)

            # Auto-scale temp y
            if self.temp:
                tmin_t = min(self.temp) - 1
                tmax_t = max(self.temp) + 1
                ax4.set_ylim(tmin_t, tmax_t)

            # Status text
            if self.anorm:
                norm_val = list(self.anorm)[-1]
                norm_mean = np.mean(list(self.anorm)[-100:]) if len(self.anorm) >= 100 else np.mean(list(self.anorm))
                temp_val = list(self.temp)[-1] if self.temp else 0
                status = (f"Live: |a|={norm_val:.4f}g  mean={norm_mean:.4f}g  "
                         f"T={temp_val:.1f}C  "
                         f"err={abs(norm_mean - 1.0)*1000:.1f}mg  "
                         f"n={len(self.t)}")
                if abs(norm_mean - 1.0) < 0.01:
                    status += "  OK"
                elif abs(norm_mean - 1.0) < 0.05:
                    status += "  FAIR"
                else:
                    status += "  NEEDS CAL"
                status_text.set_text(status)

            return [line_ax, line_ay, line_az, line_gx, line_gy, line_gz,
                    line_norm, line_temp, status_text]

        ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
        plt.tight_layout(rect=[0, 0, 1, 0.97])
        plt.show()
        self.ser.close()


def main():
    parser = argparse.ArgumentParser(description="IMU Real-Time Live Plotter")
    parser.add_argument("--port", required=True, help="Serial port (COM4)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--cal", type=str, default=None,
                        help="Calibration JSON file (shows corrected data)")
    parser.add_argument("--seconds", type=int, default=10,
                        help="Visible time window in seconds (default: 10)")
    args = parser.parse_args()

    print("\n  IMU Live Plotter")
    print("  " + "=" * 40)
    print("  What to look for:")
    print("  - Board flat: AX~0, AY~0, AZ~1.0 g, |a|~1.000 g")
    print("  - Gyro still: GX~0, GY~0, GZ~0 deg/s")
    print("  - After cal:  |a| error < 10 mg = GOOD")
    print("  - Close the window to stop.\n")

    plotter = LivePlotter(args.port, args.baud, args.seconds, args.cal)
    plotter.run()


if __name__ == "__main__":
    main()
