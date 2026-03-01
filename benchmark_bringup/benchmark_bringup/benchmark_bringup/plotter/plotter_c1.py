# +-------------------------------------------------------------------------+
# |                       Benchmark control simulator                       |
# |                                                                         |
# | Copyright (C) 2025  Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | Category 1: Cat 1 (Low Level)                                           |
# +-------------------------------------------------------------------------+

# ---------------------------------------------------------------------------
#                  Low Level Control Plotter - Category 1
# ---------------------------------------------------------------------------
#!/usr/bin/env python3
import math
import os
import sys
import csv
import argparse
from datetime import datetime
from collections import deque

import matplotlib
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
from matplotlib.gridspec import GridSpec
from matplotlib.table import Table

# =================== STYLE ===================
mpl.rcParams.update(
    {
        "font.family": "STIXGeneral",
        "mathtext.fontset": "stix",
        "font.size": 12,
        "font.weight": "bold",
        "axes.labelsize": 13,
        "axes.labelweight": "bold",
        "axes.titlesize": 14,
        "axes.titleweight": "bold",
        "xtick.labelsize": 11,
        "ytick.labelsize": 11,
        "xtick.major.width": 1.2,
        "ytick.major.width": 1.2,
        "legend.fontsize": 11,
        "legend.frameon": True,
        "lines.linewidth": 2.2,
        "grid.alpha": 0.25,
    }
)


# ==================== OFFLINE MODE ====================


def run_offline(csv_dir):
    pose_csv = os.path.join(csv_dir, "mvsim_AGI_pose.csv")
    wheel_csv = os.path.join(csv_dir, "mvsim_AGI_wheel_1.csv")

    if not os.path.isfile(wheel_csv):
        print(f"[ERROR] Cannot find {wheel_csv}")
        return

    # Read wheel CSV
    wheel_data = {}
    with open(wheel_csv) as f:
        reader = csv.DictReader(f)
        for col in reader.fieldnames:
            wheel_data[col] = []
        for row in reader:
            for col in reader.fieldnames:
                wheel_data[col].append(float(row[col]))

    t = np.array(wheel_data.get("time", []))
    sp_l = np.array(wheel_data.get("pid_sp_vel_l", []))
    sp_r = np.array(wheel_data.get("pid_sp_vel_r", []))
    act_l = np.array(wheel_data.get("pid_act_vel_l", []))
    act_r = np.array(wheel_data.get("pid_act_vel_r", []))
    err_l = np.array(wheel_data.get("pid_error_l", []))
    err_r = np.array(wheel_data.get("pid_error_r", []))
    torque_l = np.array(wheel_data.get("pid_torque_l", []))
    torque_r = np.array(wheel_data.get("pid_torque_r", []))

    # Read pose CSV for pitch
    pitch = np.zeros_like(t)
    if os.path.isfile(pose_csv):
        pose_data = {}
        with open(pose_csv) as f:
            reader = csv.DictReader(f)
            for col in reader.fieldnames:
                pose_data[col] = []
            for row in reader:
                for col in reader.fieldnames:
                    pose_data[col].append(float(row[col]))
        if "q4pitch" in pose_data:
            pitch_raw = np.array(pose_data["q4pitch"])
            pitch_t = np.array(pose_data.get("time", []))
            pitch = np.interp(t, pitch_t, np.degrees(pitch_raw))

    # Compute indices
    W_MAX = 3.2
    T_MAX = 20.0
    e = 0.5 * (np.abs(err_l) + np.abs(err_r))
    u = 0.5 * (np.abs(torque_l) + np.abs(torque_r))
    e_norm = e / W_MAX
    u_norm = u / T_MAX
    SAE = float(np.trapz(e_norm, t))
    SCI = float(np.trapz(u_norm, t))
    J = SAE + SCI

    # Plot
    fig = plt.figure(figsize=(14.5, 9.5))
    gs = GridSpec(4, 2, width_ratios=[3.6, 1.4], figure=fig)

    ax1_L = fig.add_subplot(gs[0, 0])
    ax1_R = fig.add_subplot(gs[1, 0])
    ax2 = fig.add_subplot(gs[2, 0])
    ax3 = fig.add_subplot(gs[3, 0])
    ax_table = fig.add_subplot(gs[:, 1])
    ax_table.axis("off")

    ax1_L.set_title("Category 1: Low Level Control. Angular Velocities of Left and Right Wheels")
    ax1_L.plot(t, sp_l, color="#d62728", label=r"sp $\omega_L$")
    ax1_L.plot(t, act_l, "--", color="#2c41a0", label=r"$\omega_L$")
    ax1_L.set_ylabel(r"Left $\omega$ [rad/s]")
    ax1_L.legend()
    ax1_L.grid(True)

    ax1_R.plot(t, sp_r, color="#af1fb4", label=r"sp $\omega_R$")
    ax1_R.plot(t, act_r, "--", color="#0ee3ff", label=r"$\omega_R$")
    ax1_R.set_ylabel(r"Right $\omega$ [rad/s]")
    ax1_R.legend()
    ax1_R.grid(True)

    ax2.plot(t, torque_l, color="#67bd72", label=r"$\tau_L$")
    ax2.plot(t, torque_r, "--", color="#a06a2cff", label=r"$\tau_R$")
    ax2.set_ylabel(r"Torque [$\tau$] [Nm]")
    ax2.grid(True)
    ax2.legend()

    ax3.plot(t, pitch, color="#08519c", label="Slope")
    ax3.set_ylabel(r"Slope [$\degree$]")
    ax3.set_xlabel("Time [s]")
    ax3.grid(True)
    ax3.legend()

    # Table
    rows = [
        ("Performance Index", "Real value"),
        ("SAE", f"{SAE:.4f}"),
        ("SCI", f"{SCI:.4f}"),
        ("J", f"{J:.4f}"),
    ]
    table = Table(ax_table, bbox=[0.05, 0.08, 0.9, 0.84])
    h = 0.07
    for i, (k, v) in enumerate(rows):
        for j, txt in enumerate([k, v]):
            cell = table.add_cell(i, j, width=[0.6, 0.3][j], height=h, text=txt,
                                  loc="left" if j == 0 else "right")
            cell.set_edgecolor("#CCCCCC")
            cell.set_linewidth(0.8)
            cell.set_fontsize(11)
            cell.PAD = 0.18
            if i == 0:
                cell.set_text_props(weight="bold")
    ax_table.add_table(table)

    fig.tight_layout()
    out_path = os.path.join(csv_dir, "plotter_c1.png")
    fig.savefig(out_path, dpi=150)
    print(f"Saved plot to {out_path}")
    plt.close(fig)


# ==================== REAL-TIME MODE (ROS2) ====================


class PlotterC1:

    def __init__(self, node):
        from std_msgs.msg import Float64

        self.node = node
        self.MAX_TIME = 55.0
        self.finished = False

        maxlen = 10000
        self.t = deque(maxlen=maxlen)

        self.sp_l = deque(maxlen=maxlen)
        self.sp_r = deque(maxlen=maxlen)
        self.act_l = deque(maxlen=maxlen)
        self.act_r = deque(maxlen=maxlen)

        self.torque_l = deque(maxlen=maxlen)
        self.torque_r = deque(maxlen=maxlen)
        self.err_l = deque(maxlen=maxlen)
        self.err_r = deque(maxlen=maxlen)

        self.pitch = deque(maxlen=maxlen)

        self.csv_rows = []
        self.start_time = None

        # Data storage for latest values
        self._latest = {}

        # Subscribe to individual Float64 topics
        topics = {
            "pid_sp_vel_l": "AGI/log/wheel_1/pid_sp_vel_l",
            "pid_sp_vel_r": "AGI/log/wheel_1/pid_sp_vel_r",
            "pid_act_vel_l": "AGI/log/wheel_1/pid_act_vel_l",
            "pid_act_vel_r": "AGI/log/wheel_1/pid_act_vel_r",
            "pid_error_l": "AGI/log/wheel_1/pid_error_l",
            "pid_error_r": "AGI/log/wheel_1/pid_error_r",
            "pid_torque_l": "AGI/log/wheel_1/pid_torque_l",
            "pid_torque_r": "AGI/log/wheel_1/pid_torque_r",
            "q4pitch": "AGI/log/pose/q4pitch",
        }
        for key, topic in topics.items():
            self._latest[key] = 0.0
            node.create_subscription(
                Float64, topic,
                lambda msg, k=key: self._on_float64(k, msg),
                50,
            )

        # Use pid_sp_vel_l as the "tick" to sample all values
        self._tick_key = "pid_sp_vel_l"

        matplotlib.use("TkAgg")
        plt.ion()
        self.fig = plt.figure(figsize=(14.5, 9.5))
        gs = GridSpec(4, 2, width_ratios=[3.6, 1.4], figure=self.fig)

        self.ax1_L = self.fig.add_subplot(gs[0, 0])
        self.ax1_R = self.fig.add_subplot(gs[1, 0])
        self.ax2 = self.fig.add_subplot(gs[2, 0])
        self.ax2r = self.ax2.twinx()
        self.ax3 = self.fig.add_subplot(gs[3, 0])
        self.ax_table = self.fig.add_subplot(gs[:, 1])
        self.ax_table.axis("off")

        self.timer = node.create_timer(0.1, self.update_plot)

    def now(self):
        t = self.node.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = t
        return t - self.start_time

    def _on_float64(self, key, msg):
        self._latest[key] = msg.data

        if key == self._tick_key:
            t = self.now()
            if t >= self.MAX_TIME:
                if not self.finished:
                    self.finished = True
                return

            self.t.append(t)
            self.sp_l.append(self._latest["pid_sp_vel_l"])
            self.sp_r.append(self._latest["pid_sp_vel_r"])
            self.act_l.append(self._latest["pid_act_vel_l"])
            self.act_r.append(self._latest["pid_act_vel_r"])
            self.torque_l.append(self._latest["pid_torque_l"])
            self.torque_r.append(self._latest["pid_torque_r"])
            self.err_l.append(self._latest["pid_error_l"])
            self.err_r.append(self._latest["pid_error_r"])
            pitch_deg = math.degrees(self._latest["q4pitch"])
            self.pitch.append(pitch_deg)

            self.csv_rows.append([
                t,
                self._latest["pid_sp_vel_l"],
                self._latest["pid_sp_vel_r"],
                self._latest["pid_act_vel_l"],
                self._latest["pid_act_vel_r"],
                self._latest["pid_error_l"],
                self._latest["pid_error_r"],
                self._latest["pid_torque_l"],
                self._latest["pid_torque_r"],
                pitch_deg,
            ])

    def compute_indices(self):
        if len(self.t) < 2:
            return 0.0, 0.0, 0.0

        t = np.array(self.t)
        e = 0.5 * (np.array(self.err_l) + np.array(self.err_r))
        u = 0.5 * (np.array(self.torque_l) + np.array(self.torque_r))

        W_MAX = 3.2
        T_MAX = 20.0
        e_norm = np.abs(e) / W_MAX
        u_norm = np.abs(u) / T_MAX
        SAE = float(np.trapz(e_norm, t))
        SCI = float(np.trapz(u_norm, t))
        J = SAE + SCI

        return SAE, SCI, J

    def draw_table(self, MSAE, MSIC, J):
        self.ax_table.cla()
        self.ax_table.axis("off")

        # Read PID config from env vars
        kp = os.environ.get("KP", "-")
        ki = os.environ.get("KI", "-")
        kd = os.environ.get("KD", "-")
        n_val = os.environ.get("N", "-")
        ff = os.environ.get("ENABLE_FEEDFORWARD", "false")
        aw = os.environ.get("ENABLE_ANTIWINDUP", "false")
        rf = os.environ.get("ENABLE_REFERENCE_FILTER", "false")

        rows = [
            ("Control Params", "Values"),
            ("Kp", str(kp)),
            ("Ki", str(ki)),
            ("Kd", str(kd)),
            ("N", str(n_val)),
            ("Feedforward", "ON" if ff.lower() == "true" else "OFF"),
            ("Anti-windup", "ON" if aw.lower() == "true" else "OFF"),
            ("Ref. filter", "ON" if rf.lower() == "true" else "OFF"),
            ("Performance Index", "Real value"),
            ("SAE", f"{MSAE:.4f}"),
            ("SCI", f"{MSIC:.4f}"),
            ("J", f"{J:.4f}"),
        ]

        table = Table(self.ax_table, bbox=[0.05, 0.08, 0.9, 0.84])
        h = 0.07

        for i, (k, v) in enumerate(rows):
            for j, txt in enumerate([k, v]):
                cell = table.add_cell(
                    i, j, width=[0.6, 0.3][j], height=h, text=txt,
                    loc="left" if j == 0 else "right",
                )
                cell.set_edgecolor("#CCCCCC")
                cell.set_linewidth(0.8)
                cell.set_fontsize(11)
                cell.PAD = 0.18

                if i == 0 or i == 8:
                    cell.set_text_props(weight="bold")
                else:
                    cell.set_text_props(weight="normal")

        self.ax_table.add_table(table)

    def update_plot(self):
        if len(self.t) < 2:
            return

        self.ax1_L.cla()
        self.ax1_R.cla()
        self.ax2.cla()
        self.ax2r.cla()
        self.ax3.cla()

        self.ax1_L.set_title(
            "Category 1: Low Level Control. Angular Velocities of Left and Right Wheels"
        )
        self.ax1_L.plot(self.t, self.sp_l, color="#d62728", label=r"sp $\omega_L$")
        self.ax1_L.plot(self.t, self.act_l, "--", color="#2c41a0", label=r"$\omega_L$")
        self.ax1_L.set_ylabel(r"Left $\omega$ [rad/s]")
        self.ax1_L.legend()
        self.ax1_L.grid(True)

        self.ax1_R.plot(self.t, self.sp_r, color="#af1fb4", label=r"sp $\omega_R$")
        self.ax1_R.plot(self.t, self.act_r, "--", color="#0ee3ff", label=r"$\omega_R$")
        self.ax1_R.set_ylabel(r"Right $\omega$ [rad/s]")
        self.ax1_R.legend()
        self.ax1_R.grid(True)

        self.ax2.plot(self.t, self.torque_l, color="#67bd72", label=r"$\tau_L$")
        self.ax2.plot(self.t, self.torque_r, "--", color="#a06a2cff", label=r"$\tau_R$")
        self.ax2.set_ylabel(r"Torque [$\tau$] [Nm]")
        self.ax2.grid(True)
        self.ax2.legend()
        self.ax2r.set_visible(False)

        self.ax3.plot(self.t, self.pitch, color="#08519c", label="Slope")
        self.ax3.set_ylabel(r"Slope [$\degree$]")
        self.ax3.set_xlabel("Time [s]")
        self.ax3.grid(True)
        self.ax3.legend(loc="upper left")

        MSAE, MSIC, J = self.compute_indices()
        self.draw_table(MSAE, MSIC, J)

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def save_csv(self):
        base_dir = os.path.join(os.getcwd(), "result", "categorie_1")
        os.makedirs(base_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(base_dir, f"categoria_1_{ts}.csv")

        header = [
            "time",
            "sp_vel_l",
            "sp_vel_r",
            "act_vel_l",
            "act_vel_r",
            "error_l",
            "error_r",
            "torque_l",
            "torque_r",
            "pitch_deg",
        ]

        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.csv_rows)

        self.node.get_logger().info(f"CSV saved in: {path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv-dir", type=str, default=None,
                        help="Path to directory with mvsim CSV files for offline plotting")
    # Filter out ROS2 args
    args, _ = parser.parse_known_args()

    if args.csv_dir:
        run_offline(args.csv_dir)
        return

    import rclpy
    from rclpy.node import Node as RosNode

    rclpy.init()
    ros_node = RosNode("plotter_c1_benchmark")
    plotter = PlotterC1(ros_node)
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        ros_node.get_logger().info("Stopping benchmark and saving CSV...")
    finally:
        plotter.save_csv()
        ros_node.destroy_node()
        rclpy.shutdown()
        plt.close("all")


if __name__ == "__main__":
    main()
