# +-------------------------------------------------------------------------+
# | Benchmark control simulator                                            |
# |                                                                         |
# | Copyright (C) 2025 Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | Category 3: PID Control + MPC Control + Theta* Planning                 |
# +-------------------------------------------------------------------------+
# ---------------------------------------------------------------------------
# High Level Control Plotter - Category 3
# ---------------------------------------------------------------------------

#!/usr/bin/env python3
import os
import sys
import csv
import math
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
    SAE1 = float(np.trapz(np.abs(e) / W_MAX, t))
    SCI1 = float(np.trapz(np.abs(u) / T_MAX, t))
    J1 = SAE1 + SCI1

    # Plot Figure 1 (low-level only for offline)
    fig = plt.figure(figsize=(17, 10))
    gs = GridSpec(4, 2, width_ratios=[3.6, 1.4], figure=fig)

    ax1_L = fig.add_subplot(gs[0, 0])
    ax1_R = fig.add_subplot(gs[1, 0])
    ax2 = fig.add_subplot(gs[2, 0])
    ax3 = fig.add_subplot(gs[3, 0])
    ax_table = fig.add_subplot(gs[:, 1])
    ax_table.axis("off")

    ax1_L.set_title("Category 3: Low Level Control (offline)")
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

    rows = [
        ("Performance Index", "Real value"),
        (r"$SAE_1$", f"{SAE1:.4f}"),
        (r"$SCI_1$", f"{SCI1:.4f}"),
        (r"$J_1$", f"{J1:.4f}"),
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
    out_path = os.path.join(csv_dir, "plotter_c3.png")
    fig.savefig(out_path, dpi=150)
    print(f"Saved plot to {out_path}")
    plt.close(fig)


# ==================== REAL-TIME MODE (ROS2) ====================


class PlotterC3:
    def __init__(self, node):
        from std_msgs.msg import Float64
        from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseArray
        from nav_msgs.msg import Path

        self.node = node
        self.MAX_TIME = 80.0
        maxlen = 15000
        self.t = deque(maxlen=maxlen)
        self.sp_l = deque(maxlen=maxlen)
        self.sp_r = deque(maxlen=maxlen)
        self.act_l = deque(maxlen=maxlen)
        self.act_r = deque(maxlen=maxlen)
        self.torque_l = deque(maxlen=maxlen)
        self.torque_r = deque(maxlen=maxlen)
        self.err_l = deque(maxlen=maxlen)
        self.err_r = deque(maxlen=maxlen)
        self.v_cmd = deque(maxlen=maxlen)
        self.w_cmd = deque(maxlen=maxlen)
        self.x = deque(maxlen=maxlen)
        self.y = deque(maxlen=maxlen)
        self.ref_x, self.ref_y = [], []
        self.teb_x, self.teb_y = [], []
        self.ref_near_x = deque(maxlen=maxlen)
        self.ref_near_y = deque(maxlen=maxlen)
        self.teb_near_x = deque(maxlen=maxlen)
        self.teb_near_y = deque(maxlen=maxlen)
        self.err_pred_x = deque(maxlen=maxlen)
        self.err_pred_y = deque(maxlen=maxlen)
        self.err_teb_ref = deque(maxlen=maxlen)
        self.theta_x = []
        self.theta_y = []
        self.theta_err = deque(maxlen=maxlen)
        self.pitch = deque(maxlen=maxlen)
        self.start_time = None
        self.t_err_pred = deque(maxlen=maxlen)
        self._latest = {}

        # Float64 subscriptions
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

        node.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.cb_amcl, 20
        )
        node.create_subscription(Path, "/global_plan", self.cb_global_plan, 10)
        node.create_subscription(PoseArray, "/teb_poses", self.cb_teb, 10)
        node.create_subscription(Twist, "/cmd_vel_nav", self.cb_cmd, 20)
        node.create_subscription(Path, "/plan", self.cb_theta, 10)

        # ================= FIGURE 1 =================
        self.timer_plot = node.create_timer(1.0, self.update_plot)
        matplotlib.use("TkAgg")
        matplotlib.rcParams["figure.raise_window"] = False
        plt.ion()
        self.fig1 = plt.figure(figsize=(17, 10))
        gs1 = GridSpec(4, 2, width_ratios=[3.6, 1.4], figure=self.fig1)
        self.ax1_L = self.fig1.add_subplot(gs1[0, 0])
        self.ax1_R = self.fig1.add_subplot(gs1[1, 0])
        self.ax2 = self.fig1.add_subplot(gs1[2, 0])
        self.ax2r = None
        self.ax3 = self.fig1.add_subplot(gs1[3, 0])
        self.ax_table1 = self.fig1.add_subplot(gs1[:, 1])
        self.ax_table1.axis("off")
        # ================= FIGURE 2 =================
        self.fig2 = plt.figure(figsize=(17, 10))
        gs2 = GridSpec(3, 2, width_ratios=[3.6, 1.4], figure=self.fig2)
        self.ax_xy = self.fig2.add_subplot(gs2[0, 0])
        self.ax_vw = self.fig2.add_subplot(gs2[1, 0])
        self.ax_vw_r = self.ax_vw.twinx()
        self.ax_err = self.fig2.add_subplot(gs2[2, 0])
        self.ax_table2 = self.fig2.add_subplot(gs2[:, 1])
        self.ax_table2.axis("off")
        # ================= FIGURE 3 =================
        self.fig3 = plt.figure(figsize=(17, 10))
        gs3 = GridSpec(2, 2, width_ratios=[3.6, 1.4], figure=self.fig3)
        self.ax_theta_traj = self.fig3.add_subplot(gs3[0, 0])
        self.ax_theta_err = self.fig3.add_subplot(gs3[1, 0])
        self.ax_table3 = self.fig3.add_subplot(gs3[:, 1])
        self.ax_table3.axis("off")

    def now(self):
        t = self.node.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = t
        return t - self.start_time

    def _on_float64(self, key, msg):
        self._latest[key] = msg.data

        if key == "pid_sp_vel_l":
            t = self.now()
            if t >= self.MAX_TIME:
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

            if self.x and self.y:
                xr = self.x[-1]
                yr = self.y[-1]
            else:
                xr = np.nan
                yr = np.nan

            if self.ref_x and self.ref_y and not np.isnan(xr):
                ref_x_arr = np.array(self.ref_x)
                ref_y_arr = np.array(self.ref_y)
                d_ref = (ref_x_arr - xr) ** 2 + (ref_y_arr - yr) ** 2
                idx_ref = int(np.argmin(d_ref))
                self.ref_near_x.append(float(ref_x_arr[idx_ref]))
                self.ref_near_y.append(float(ref_y_arr[idx_ref]))
            else:
                self.ref_near_x.append(np.nan)
                self.ref_near_y.append(np.nan)

            if self.teb_x and self.teb_y and not np.isnan(xr):
                teb_x_arr = np.array(self.teb_x)
                teb_y_arr = np.array(self.teb_y)
                d_teb = (teb_x_arr - xr) ** 2 + (teb_y_arr - yr) ** 2
                idx_teb = int(np.argmin(d_teb))
                self.teb_near_x.append(float(teb_x_arr[idx_teb]))
                self.teb_near_y.append(float(teb_y_arr[idx_teb]))
            else:
                self.teb_near_x.append(np.nan)
                self.teb_near_y.append(np.nan)

            if self.ref_x and self.ref_y and self.teb_x and self.teb_y:
                ref_x_arr = np.array(self.ref_x)
                ref_y_arr = np.array(self.ref_y)
                teb_x_arr = np.array(self.teb_x)
                teb_y_arr = np.array(self.teb_y)
                n = min(len(ref_x_arr), len(teb_x_arr))
                if n > 0:
                    dx = ref_x_arr[:n] - teb_x_arr[:n]
                    dy = ref_y_arr[:n] - teb_y_arr[:n]
                    dist = np.sqrt(dx**2 + dy**2)
                    e_mean = float(np.mean(dist))
                else:
                    e_mean = 0.0
            else:
                e_mean = 0.0
            self.err_teb_ref.append(e_mean)

            if self.theta_x and self.theta_y and self.x and self.y:
                xr = self.x[-1]
                yr = self.y[-1]
                th_x = np.array(self.theta_x)
                th_y = np.array(self.theta_y)
                dx = th_x - xr
                dy = th_y - yr
                dist = np.sqrt(dx**2 + dy**2)
                e_theta = float(np.min(dist))
            else:
                e_theta = 0.0
            self.theta_err.append(e_theta)

    def cb_cmd(self, msg):
        self.v_cmd.append(msg.linear.x)
        self.w_cmd.append(msg.angular.z)

    def cb_amcl(self, msg):
        self.x.append(msg.pose.pose.position.x)
        self.y.append(msg.pose.pose.position.y)

    def cb_global_plan(self, msg):
        self.ref_x = [p.pose.position.x for p in msg.poses]
        self.ref_y = [p.pose.position.y for p in msg.poses]

    def cb_teb(self, msg):
        self.teb_x = [p.position.x for p in msg.poses]
        self.teb_y = [p.position.y for p in msg.poses]

        if self.x and self.y and self.teb_x:
            xr, yr = self.x[-1], self.y[-1]
            xp, yp = self.teb_x[0], self.teb_y[0]
            self.err_pred_x.append(xr - xp)
            self.err_pred_y.append(yr - yp)
            self.t_err_pred.append(self.t[-1])

    def cb_theta(self, msg):
        self.theta_x = [p.pose.position.x for p in msg.poses]
        self.theta_y = [p.pose.position.y for p in msg.poses]

    # --------------------- INDICES ---------------------
    def compute_indices_wheels(self):
        if len(self.t) < 2:
            return 0.0, 0.0, 0.0
        t = np.array(self.t)
        e = 0.5 * (np.array(self.err_l) + np.array(self.err_r))
        u = 0.5 * (np.array(self.torque_l) + np.array(self.torque_r))
        W_MAX = 3.2
        T_MAX = 20.0
        e_norm = np.abs(e) / W_MAX
        u_norm = np.abs(u) / T_MAX
        SAE1 = float(np.trapz(e_norm, t))
        SCI1 = float(np.trapz(u_norm, t))
        J1 = SAE1 + SCI1
        return SAE1, SCI1, J1

    def compute_indices_teb_ref(self):
        if (
            len(self.t) < 2
            or len(self.err_teb_ref) < 2
            or len(self.v_cmd) < 2
            or len(self.w_cmd) < 2
        ):
            return 0.0, 0.0, 0.0
        n = min(len(self.t), len(self.err_teb_ref), len(self.v_cmd), len(self.w_cmd))
        t_arr = np.array(list(self.t)[-n:])
        e_path = np.array(list(self.err_teb_ref)[-n:])
        v_arr = np.array(list(self.v_cmd)[-n:])
        w_arr = np.array(list(self.w_cmd)[-n:])
        SAE2 = float(np.trapz(np.abs(e_path), t_arr))
        V_MAX = 0.75
        W_MAX = 3.2
        ev_norm = np.abs(v_arr) / V_MAX
        ew_norm = np.abs(w_arr) / W_MAX
        u_norm = 0.5 * (ev_norm + ew_norm)
        SCI2 = float(np.trapz(u_norm, t_arr))
        J2 = SAE2 + SCI2
        return SAE2, SCI2, J2

    def compute_indices_theta(self):
        if len(self.t) < 2 or len(self.theta_err) < 2 or len(self.theta_x) < 2:
            return 0.0, 0.0, 0.0, 0.0
        n = min(len(self.t), len(self.theta_err))
        t_arr = np.array(list(self.t)[-n:])
        e_th_arr = np.array(list(self.theta_err)[-n:])
        SAE3 = float(np.trapz(np.abs(e_th_arr), t_arr))
        th_x = np.array(self.theta_x, dtype=float)
        th_y = np.array(self.theta_y, dtype=float)
        if th_x.size < 2:
            return SAE3, 0.0, SAE3, 0.0
        seg = np.sqrt(np.diff(th_x) ** 2 + np.diff(th_y) ** 2)
        path_len = float(np.sum(seg))
        W_EUC = 1.0
        W_TRAV = 2.0
        L_MAX = 50.0
        C_total = (W_EUC + W_TRAV) * path_len
        C_max = (W_EUC + W_TRAV) * L_MAX
        SCI3 = 0.0
        if C_max > 0.0:
            SCI3 = C_total / C_max
        if SCI3 > 1.0:
            SCI3 = 1.0
        J3 = SAE3
        return SAE3, SCI3, J3, path_len

    # --------------------- TABLES ---------------------
    def draw_table_fig1(self, SAE1, SCI1, J1):
        self.ax_table1.cla()
        self.ax_table1.axis("off")

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
            (r"$SAE_1$", f"{SAE1:.4f}"),
            (r"$SCI_1$", f"{SCI1:.4f}"),
            (r"J1 = $SAE_1$ + $SCI_1$", f"{J1:.4f}"),
        ]
        table = Table(self.ax_table1, bbox=[0.05, 0.08, 0.9, 0.84])
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
                if i in (0, 8):
                    cell.set_text_props(weight="bold")
                else:
                    cell.set_text_props(weight="normal")
        self.ax_table1.add_table(table)

    def draw_table_fig2(self, SAE1, SCI1, J1, SAE2, SCI2, J2):
        self.ax_table2.cla()
        self.ax_table2.axis("off")
        V_MAX = 0.75
        W_MAX = 3.2
        J = J1 + J2
        rows = [
            ("Params", "Values"),
            (r"$v_{max}$ [m/s]", f"{V_MAX:.2f}"),
            (r"$\omega_{max}$ [rad/s]", f"{W_MAX:.2f}"),
            (r"$d_{obst}$ [m]", "0.4"),
            ("Low level Index", ""),
            (r"$SAE_1$", f"{SAE1:.4f}"),
            (r"$SCI_1$", f"{SCI1:.4f}"),
            (r"J1 = $SAE_1$ + $SCI_1$", f"{J1:.4f}"),
            ("Mid level Index", ""),
            (r"$SAE_2$", f"{SAE2:.4f}"),
            (r"$SCI_2$", f"{SCI2:.4f}"),
            (r"J2 = $SAE_2$ + $SCI_2$", f"{J2:.4f}"),
        ]
        table = Table(self.ax_table2, bbox=[0.05, 0.08, 0.9, 0.84])
        h = 0.06
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
                if i in (0, 4, 8):
                    cell.set_text_props(weight="bold")
                else:
                    cell.set_text_props(weight="normal")
        self.ax_table2.add_table(table)

    def draw_table_fig3(self, SAE1, SCI1, J1, SAE2, SCI2, J2, SAE3, SCI3, J3):
        self.ax_table3.cla()
        self.ax_table3.axis("off")
        J_final = (J1 + J2 + J3) / 3
        rows = [
            ("Low level Index", ""),
            (r"$SAE_1$", f"{SAE1:.4f}"),
            (r"$SCI_1$", f"{SCI1:.4f}"),
            (r"J1 = $SAE_1$ + $SCI_1$", f"{J1:.4f}"),
            ("Mid level Index", ""),
            (r"$SAE_2$", f"{SAE2:.4f}"),
            (r"$SCI_2$", f"{SCI2:.4f}"),
            (r"J2 = $SAE_2$ + $SCI_2$", f"{J2:.4f}"),
            ("High level Index", ""),
            (r"$SAE_3$", f"{SAE3:.4f}"),
            (r"J3 = $SAE_3$", f"{SAE3:.4f}"),
            (r"J=$\frac{1}{3}$(J1 + J2 + J3)", f"{J_final:.4f}"),
        ]
        table = Table(self.ax_table3, bbox=[0.05, 0.08, 0.9, 0.84])
        h = 0.07
        for i, (k, v) in enumerate(rows):
            for j, txt in enumerate([k, v]):
                cell = table.add_cell(
                    i, j, width=[0.7, 0.3][j], height=h, text=txt,
                    loc="left" if j == 0 else "right",
                )
                cell.set_edgecolor("#CCCCCC")
                cell.set_linewidth(0.8)
                cell.set_fontsize(11)
                cell.PAD = 0.18
                if i in (0, 4, 8):
                    cell.set_text_props(weight="bold")
                else:
                    cell.set_text_props(weight="normal")
        self.ax_table3.add_table(table)

    def update_plot(self):
        if self.t and self.t[-1] >= self.MAX_TIME:
            return
        if len(self.t) < 2:
            return
        # ===================== FIGURE 1 =====================
        self.ax1_L.cla()
        self.ax1_L.set_title(
            "Figure 1, Category 3: Low Level Control. Angular Velocities of Left and Right Wheels (PID)"
        )
        self.ax1_L.plot(self.t, self.sp_l, color="#d62728", label=r"sp $\omega_L$")
        self.ax1_L.plot(self.t, self.act_l, "--", color="#2c41a0", label=r"$\omega_L$")
        self.ax1_L.set_ylabel(r"Left $\omega$ [rad/s]")
        self.ax1_L.legend()
        self.ax1_L.grid(True)

        self.ax1_R.cla()
        self.ax1_R.plot(self.t, self.sp_r, color="#af1fb4", label=r"sp $\omega_R$")
        self.ax1_R.plot(self.t, self.act_r, "--", color="#0ee3ff", label=r"$\omega_R$")
        self.ax1_R.set_ylabel(r"Right $\omega$ [rad/s]")
        self.ax1_R.legend()
        self.ax1_R.grid(True)

        self.ax2.cla()
        self.ax2.plot(self.t, self.torque_l, color="#67bd72", label=r"$\tau_L$")
        self.ax2.plot(self.t, self.torque_r, "--", color="#a06a2cff", label=r"$\tau_R$")
        self.ax2.set_ylabel(r"Torque [$\tau$] [Nm]")
        self.ax2.grid(True)
        self.ax2.legend()

        self.ax3.cla()
        if self.pitch:
            n = min(len(self.t), len(self.pitch))
            t_plot = list(self.t)[-n:]
            self.ax3.plot(t_plot, list(self.pitch)[-n:], color="#08519c", label="Slope")
            self.ax3.set_ylabel(r"Slope [$\degree$]")
        self.ax3.set_xlabel("Time [s]")
        self.ax3.grid(True)
        lines3 = self.ax3.get_lines()
        if lines3:
            self.ax3.legend(lines3, [l.get_label() for l in lines3], loc="upper left")

        SAE1, SCI1, J1 = self.compute_indices_wheels()
        self.draw_table_fig1(SAE1, SCI1, J1)
        self.fig1.tight_layout()
        self.fig1.canvas.draw_idle()
        self.fig1.canvas.flush_events()
        plt.pause(0.001)

        # ===================== FIGURE 2 =====================
        self.ax_xy.cla()
        if self.x and self.y:
            xr = self.x[-1]
            yr = self.y[-1]
            self.ax_xy.set_title(
                "Figure 2, Category 3: Mid Level Control. Robot Trajectory (TEB - MPC)"
            )
            if self.teb_x and self.teb_y:
                self.ax_xy.plot(
                    self.teb_x, self.teb_y, color="#2ca02c", label="TEB prediction"
                )
            self.ax_xy.plot(self.x, self.y, label="Trajectory robot")
            self.ax_xy.scatter(
                xr, yr, s=60, marker="s", color="black", zorder=5, label="Robot"
            )
            self.ax_xy.set_xlim(xr - 2.5, xr + 2.5)
            self.ax_xy.set_ylim(yr - 2.5, yr + 2.5)
            self.ax_xy.set_aspect("equal", adjustable="datalim")
            self.ax_xy.set_xlabel("x [m]")
            self.ax_xy.set_ylabel("y [m]")
            self.ax_xy.legend()
            self.ax_xy.grid(True)

        self.ax_vw.cla()
        self.ax_vw_r.cla()
        if self.v_cmd:
            t_cmd = list(self.t)[-len(self.v_cmd):]
            self.ax_vw.plot(t_cmd, self.v_cmd, label="v", color="#d62728", linewidth=2)
            self.ax_vw.set_ylabel("Longitudinal velocity [m/s]", fontweight="bold")
            self.ax_vw_r.plot(
                t_cmd, self.w_cmd, label="\u03c9", color="#1f77b4", linewidth=2
            )
            self.ax_vw_r.set_ylabel("Angular velocity [rad/s]", fontweight="bold")
            lines_vw = self.ax_vw.get_lines() + self.ax_vw_r.get_lines()
            labels_vw = [l.get_label() for l in lines_vw]
            self.ax_vw.legend(lines_vw, labels_vw)
            self.ax_vw.grid(True)

        self.ax_err.cla()
        if self.t and self.t[-1] >= self.MAX_TIME:
            return
        self.ax_err.plot(self.t_err_pred, self.err_pred_x, "m", label="e_x")
        self.ax_err.plot(self.t_err_pred, self.err_pred_y, "c", label="e_y")
        self.ax_err.set_xlabel("Time [s]")
        self.ax_err.set_ylabel("Local planner error [m]")
        self.ax_err.legend()
        self.ax_err.grid(True)

        SAE2, SCI2, J2 = self.compute_indices_teb_ref()
        self.draw_table_fig2(SAE1, SCI1, J1, SAE2, SCI2, J2)
        self.fig2.tight_layout()
        self.fig2.canvas.draw_idle()
        self.fig2.canvas.flush_events()
        plt.pause(0.001)

        # ===================== FIGURE 3 =====================
        self.ax_theta_traj.cla()
        self.ax_theta_traj.set_title(
            "Figure 3, Category 3: High Level. Planner trajectory (Theta*)"
        )
        if self.ref_x and self.ref_y:
            self.ax_theta_traj.plot(
                self.ref_x, self.ref_y, "--r", label=r"$\theta^*$ plan"
            )
        if self.teb_x and self.teb_y:
            self.ax_theta_traj.plot(
                self.teb_x, self.teb_y, color="#2ca02c", label="TEB prediction"
            )
        if self.x and self.y:
            self.ax_theta_traj.plot(self.x, self.y, "b", label="Trajectory robot")
            xr = self.x[-1]
            yr = self.y[-1]
            self.ax_theta_traj.scatter(
                xr, yr, s=60, marker="s", color="black", zorder=5, label="Robot"
            )

        self.ax_theta_traj.set_xlabel("x [m]")
        self.ax_theta_traj.set_ylabel("y [m]")
        self.ax_theta_traj.grid(True)
        self.ax_theta_traj.legend()
        self.ax_theta_traj.set_aspect("equal", adjustable="datalim")

        self.ax_theta_err.cla()
        if self.theta_err:
            t_err_th = list(self.t)[-len(self.theta_err):]
            self.ax_theta_err.plot(
                t_err_th, list(self.theta_err), "m", label="Theta* error"
            )
        self.ax_theta_err.set_ylabel("Global planner error [m]")
        self.ax_theta_err.set_xlabel("Time [s]")
        self.ax_theta_err.grid(True)
        self.ax_theta_err.legend()

        SAE3, SCI3, J3, path_len = self.compute_indices_theta()
        self.draw_table_fig3(SAE1, SCI1, J1, SAE2, SCI2, J2, SAE3, SCI3, J3)
        self.fig3.tight_layout()
        self.fig3.canvas.draw_idle()
        self.fig3.canvas.flush_events()
        plt.pause(0.001)

    def save_csv(self):
        base_dir = os.path.join(os.getcwd(), "result", "category_3")
        os.makedirs(base_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(base_dir, f"category_3_{ts}.csv")
        SAE1, SCI1, J1 = self.compute_indices_wheels()
        SAE2, SCI2, J2 = self.compute_indices_teb_ref()
        SAE3, SCI3, J3, path_len = self.compute_indices_theta()
        J_final = (J1 + J2 + J3) / 3.0
        header = [
            "time", "sp_l", "sp_r", "act_l", "act_r",
            "torque_l", "torque_r", "err_l", "err_r",
            "v", "w", "x", "y", "pitch",
            "err_pred_x", "err_pred_y",
            "ref_near_x", "ref_near_y",
            "teb_near_x", "teb_near_y", "err_teb_ref",
            "theta_err",
            "SAE1", "SCI1", "J1",
            "SAE2", "SCI2", "J2",
            "SAE3", "J3", "J_final",
        ]
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            N = len(self.t)
            for i in range(N):
                writer.writerow([
                    self.t[i],
                    self.sp_l[i] if i < len(self.sp_l) else "",
                    self.sp_r[i] if i < len(self.sp_r) else "",
                    self.act_l[i] if i < len(self.act_l) else "",
                    self.act_r[i] if i < len(self.act_r) else "",
                    self.torque_l[i] if i < len(self.torque_l) else "",
                    self.torque_r[i] if i < len(self.torque_r) else "",
                    self.err_l[i] if i < len(self.err_l) else "",
                    self.err_r[i] if i < len(self.err_r) else "",
                    self.v_cmd[i] if i < len(self.v_cmd) else "",
                    self.w_cmd[i] if i < len(self.w_cmd) else "",
                    self.x[i] if i < len(self.x) else "",
                    self.y[i] if i < len(self.y) else "",
                    self.pitch[i] if i < len(self.pitch) else "",
                    self.err_pred_x[i] if i < len(self.err_pred_x) else "",
                    self.err_pred_y[i] if i < len(self.err_pred_y) else "",
                    self.ref_near_x[i] if i < len(self.ref_near_x) else "",
                    self.ref_near_y[i] if i < len(self.ref_near_y) else "",
                    self.teb_near_x[i] if i < len(self.teb_near_x) else "",
                    self.teb_near_y[i] if i < len(self.teb_near_y) else "",
                    self.err_teb_ref[i] if i < len(self.err_teb_ref) else "",
                    self.theta_err[i] if i < len(self.theta_err) else "",
                    SAE1, SCI1, J1,
                    SAE2, SCI2, J2,
                    SAE3, J3, J_final,
                ])
        self.node.get_logger().info(f"CSV saved in: {path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv-dir", type=str, default=None,
                        help="Path to directory with mvsim CSV files for offline plotting")
    args, _ = parser.parse_known_args()

    if args.csv_dir:
        run_offline(args.csv_dir)
        return

    import rclpy
    from rclpy.node import Node as RosNode

    rclpy.init()
    ros_node = RosNode("plotter_c3_benchmark")
    plotter = PlotterC3(ros_node)
    plt.show(block=False)
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
