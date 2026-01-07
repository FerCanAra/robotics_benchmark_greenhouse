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
import csv
from datetime import datetime
from collections import deque

import rclpy
from rclpy.node import Node
from benchmark_msg.msg import BenchmarkParams

import matplotlib

matplotlib.use("TkAgg")
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

# ==================== PLOTTER ====================


class PlotterC1(Node):

    def __init__(self):
        super().__init__("plotter_c1_benchmark")

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
        self.sector = deque(maxlen=maxlen)
        self.current_sector = 1

        self.csv_rows = []

        self.start_time = None
        self.last_msg = None

        self.create_subscription(BenchmarkParams, "/benchmark_params", self.cb, 50)

        plt.ion()
        self.fig = plt.figure(figsize=(14.5, 9.5))
        gs = GridSpec(4, 2, width_ratios=[3.6, 1.4], figure=self.fig)

        self.ax1_L = self.fig.add_subplot(gs[0, 0])
        self.ax1_R = self.fig.add_subplot(gs[1, 0])
        self.ax2 = self.fig.add_subplot(gs[2, 0])
        self.ax2r = self.ax2.twinx()
        self.ax3 = self.fig.add_subplot(gs[3, 0])
        self.ax3r = self.ax3.twinx()
        self.ax_table = self.fig.add_subplot(gs[:, 1])
        self.ax_table.axis("off")

        self.timer = self.create_timer(0.1, self.update_plot)

    def now(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = t
        return t - self.start_time

    def cb(self, msg: BenchmarkParams):
        t = self.now()
        self.last_msg = msg

        if t >= self.MAX_TIME:
            if not self.finished:
                self.finished = True
            return

        self.t.append(t)
        self.sp_l.append(msg.sp_vel_l)
        self.sp_r.append(msg.sp_vel_r)
        self.act_l.append(msg.act_vel_l)
        self.act_r.append(msg.act_vel_r)

        self.torque_l.append(msg.torque_l)
        self.torque_r.append(msg.torque_r)
        self.err_l.append(msg.error_l)
        self.err_r.append(msg.error_r)

        pitch_deg = math.degrees(msg.pitch)
        self.pitch.append(pitch_deg)

        if msg.mu >= msg.mu_zone1:
            self.current_sector = 1

        if msg.mu >= msg.mu_zone2:
            self.current_sector = 2

        if msg.mu <= msg.mu_zone3:
            self.current_sector = 3

        self.sector.append(self.current_sector)

        self.csv_rows.append(
            [
                t,
                msg.sp_vel_l,
                msg.sp_vel_r,
                msg.act_vel_l,
                msg.act_vel_r,
                msg.error_l,
                msg.error_r,
                msg.torque_l,
                msg.torque_r,
                pitch_deg,
                self.current_sector,
            ]
        )

    def compute_indices(self):
        if len(self.t) < 2:
            return 0.0, 0.0, 0.0

        t = np.array(self.t)
        e = 0.5 * (np.array(self.err_l) + np.array(self.err_r))
        u = 0.5 * (np.array(self.torque_l) + np.array(self.torque_r))

        W_MAX = 3.2  # rad/s
        T_MAX = 20.0  # Nm
        e_norm = np.abs(e) / W_MAX
        u_norm = np.abs(u) / T_MAX
        SAE = float(np.trapz(e_norm, t))
        SCI = float(np.trapz(u_norm, t))
        J = SAE + SCI

        return SAE, SCI, J

    def draw_table(self, MSAE, MSIC, J):
        self.ax_table.cla()
        self.ax_table.axis("off")

        rows = [
            ("Control Params", "Values"),
            ("Kp", f"{self.last_msg.kp:.2f}"),
            ("Ki", f"{self.last_msg.ki:.2f}"),
            ("Kd", f"{self.last_msg.kd:.2f}"),
            ("N", f"{self.last_msg.n:.1f}"),
            ("Feedforward", "ON" if self.last_msg.enable_feedforward else "OFF"),
            ("Anti-windup", "ON" if self.last_msg.enable_antiwindup else "OFF"),
            ("Ref. filter", "ON" if self.last_msg.enable_referencefilter else "OFF"),
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
                    i,
                    j,
                    width=[0.6, 0.3][j],
                    height=h,
                    text=txt,
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
        self.ax3r.cla()

        # ---------- Subplot (4,1,1): ωL ----------
        self.ax1_L.set_title(
            "Category 1: Low Level Control. Angular Velocities of Left and Right Wheels"
        )
        self.ax1_L.plot(self.t, self.sp_l, color="#d62728", label=r"sp $\omega_L$")
        self.ax1_L.plot(self.t, self.act_l, "--", color="#2c41a0", label=r"$\omega_L$")
        self.ax1_L.set_ylabel(r"Left $\omega$ [rad/s]")
        self.ax1_L.legend()
        self.ax1_L.grid(True)

        # ---------- Subplot (4,1,2): ωR ----------
        self.ax1_R.plot(self.t, self.sp_r, color="#af1fb4", label=r"sp $\omega_R$")
        self.ax1_R.plot(self.t, self.act_r, "--", color="#0ee3ff", label=r"$\omega_R$")
        self.ax1_R.set_ylabel(r"Right $\omega$ [rad/s]")
        self.ax1_R.legend()
        self.ax1_R.grid(True)

        # ---------- Subplot (4,1,3): Inputs  ----------
        self.ax2.plot(self.t, self.torque_l, color="#67bd72", label=r"$\tau_L$")
        self.ax2.plot(self.t, self.torque_r, "--", color="#a06a2cff", label=r"$\tau_R$")
        self.ax2.set_ylabel(r"Torque [$\tau$] [Nm]")
        self.ax2.grid(True)
        self.ax2.legend()
        self.ax2r.set_visible(False)

        # ---------- Subplot (4,1,4): Disturbances ----------
        self.ax3.plot(self.t, self.pitch, color="#08519c", label="Slope")
        self.ax3r.step(
            self.t,
            self.sector,
            where="post",
            linestyle="-",
            linewidth=2.5,
            color="#a50f15",
            label="Terrain sector",
        )

        self.ax3.set_ylabel(r"Slope [$\degree$]")
        self.ax3r.set_ylabel("Sector")
        self.ax3r.set_yticks([1, 2, 3])
        self.ax3r.set_yticklabels(["S1", "S2", "S3"])
        self.ax3.set_xlabel("Time [s]")
        self.ax3.grid(True)

        lines = self.ax3.get_lines() + self.ax3r.get_lines()
        self.ax3.legend(lines, [l.get_label() for l in lines], loc="upper left")

        # ---------- TABLE ----------
        if self.last_msg:
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
            "sector",
        ]

        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.csv_rows)

        self.get_logger().info(f"CSV guardado en: {path}")


def main():
    rclpy.init()
    node = PlotterC1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Finalizando benchmark y guardando CSV...")
    finally:
        node.save_csv()
        node.destroy_node()
        rclpy.shutdown()
        plt.close("all")


if __name__ == "__main__":
    main()
