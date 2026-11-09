#!/usr/bin/env python3
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
# The evaluation window is defined in SIMULATED time: it starts when the
# category script publishes the STARTED event (t0_sim) and lasts t_end seconds.

import math
import os

import numpy as np
import rclpy

from benchmark_bringup.metrics import low_level_indices
from benchmark_bringup.plotter.common import BenchmarkRecorder, run_recorder


class PlotterC1(BenchmarkRecorder):
    CATEGORY = 1

    def __init__(self):
        super().__init__("plotter_c1_benchmark")
        self.fig = None
        if self.plot:
            self._create_figure()
            self.plot_period = 0.5
        self.create_timer(0.2, self._check_end)

    # ------------------------------------------------------------- termination
    def on_bench(self, msg):
        if self.t0_sim is None and self.t0_wall is None:
            return  # waiting for the STARTED event of the category script
        if self.t0_sim is None:  # fallback: STARTED event without simulated time
            self.t0_sim = msg.sim_time
        # The window closes t_settle seconds after the end of the velocity profile
        # (PROFILE_END event of the category script), or at t_end at the latest.
        t_close = self.t0_sim + self.t_end
        for ev in self.events:
            if ev.get("state") == "PROFILE_END" and ev.get("t_sim") is not None:
                t_close = min(t_close, float(ev["t_sim"]) + self.t_settle)
        if msg.sim_time >= t_close:
            self.request_finish("FINISHED")

    def _check_end(self):
        # Safety net: if the simulator stops publishing, close after t_end (+10 s) of wall time
        if not self.done and self.t0_wall is not None:
            if self.now() - self.t0_wall > self.t_end + 10.0:
                self.request_finish("TIMEOUT")

    # ----------------------------------------------------------------- metrics
    def _window_data(self):
        c = self.bench_columns()
        if not c["sim_time"]:
            return None
        t = np.asarray(c["sim_time"], dtype=float)
        t0 = self.t0_sim if self.t0_sim is not None else t[0]
        return c, t, t0, t0 + self.t_end

    def compute_metrics(self) -> dict:
        wd = self._window_data()
        if wd is None:
            return {}
        c, t, t0, t1 = wd
        m = low_level_indices(
            t, c["error_l"], c["error_r"], c["torque_l"], c["torque_r"], self.cfg, t0, t1
        )
        m["JT1"] = m["J1"]
        return m

    # ------------------------------------------------------------------ figure
    def _create_figure(self):
        from matplotlib.gridspec import GridSpec

        self.fig = self.plt.figure(figsize=(14.5, 9.5))
        gs = GridSpec(4, 2, width_ratios=[3.6, 1.4], figure=self.fig)
        self.ax1_L = self.fig.add_subplot(gs[0, 0])
        self.ax1_R = self.fig.add_subplot(gs[1, 0])
        self.ax2 = self.fig.add_subplot(gs[2, 0])
        self.ax3 = self.fig.add_subplot(gs[3, 0])
        self.ax3r = self.ax3.twinx()
        self.ax_table = self.fig.add_subplot(gs[:, 1])
        self.ax_table.axis("off")

    def _draw(self):
        from matplotlib.table import Table

        wd = self._window_data()
        if wd is None or self.fig is None:
            return
        c, t_abs, t0, t1 = wd
        t = t_abs - t0
        for ax in (self.ax1_L, self.ax1_R, self.ax2, self.ax3, self.ax3r):
            ax.cla()

        self.ax1_L.set_title("Category 1: Low Level Control. Linear Velocities of Left and Right Wheels")
        self.ax1_L.plot(t, c["sp_vel_l"], color="#d62728", label=r"sp $v_L$")
        self.ax1_L.plot(t, c["act_vel_l"], "--", color="#2c41a0", label=r"$v_L$")
        self.ax1_L.set_ylabel(r"Left $v$ [m/s]")
        self.ax1_L.legend(loc="upper right")
        self.ax1_L.grid(True)

        self.ax1_R.plot(t, c["sp_vel_r"], color="#af1fb4", label=r"sp $v_R$")
        self.ax1_R.plot(t, c["act_vel_r"], "--", color="#0ee3ff", label=r"$v_R$")
        self.ax1_R.set_ylabel(r"Right $v$ [m/s]")
        self.ax1_R.legend(loc="upper right")
        self.ax1_R.grid(True)

        self.ax2.plot(t, c["torque_l"], color="#67bd72", label=r"$\tau_L$")
        self.ax2.plot(t, c["torque_r"], "--", color="#a06a2c", label=r"$\tau_R$")
        self.ax2.set_ylabel(r"Torque $\tau$ [N m]")
        self.ax2.legend(loc="upper right")
        self.ax2.grid(True)

        self.ax3.plot(t, np.degrees(c["pitch_rad"]), color="#08519c", label="Slope")
        self.ax3r.step(t, c["sector"], where="post", linewidth=2.5, color="#a50f15",
                       label="Terrain sector")
        self.ax3.set_ylabel(r"Slope [$\degree$]")
        self.ax3r.set_ylabel("Sector")
        self.ax3r.yaxis.set_label_position("right")
        self.ax3r.yaxis.tick_right()
        self.ax3r.set_yticks([0, 1, 2, 3])
        self.ax3r.set_yticklabels(["-", "S1", "S2", "S3"])
        self.ax3.set_xlabel("Time [s]")
        self.ax3.grid(True)
        lines = self.ax3.get_lines() + self.ax3r.get_lines()
        self.ax3.legend(lines, [ln.get_label() for ln in lines], loc="upper left")
        for ax in (self.ax1_L, self.ax1_R, self.ax2, self.ax3):
            ax.set_xlim(0, self.t_end)

        # Table
        m = self.compute_metrics()
        lb = self.last_bench
        self.ax_table.cla()
        self.ax_table.axis("off")
        rows = [("Control Params", "Values")]
        if lb is not None:
            rows += [
                ("Kp", f"{lb.kp:.2f}"), ("Ki", f"{lb.ki:.2f}"), ("Kd", f"{lb.kd:.2f}"),
                ("N", f"{lb.n:.1f}"),
                ("Feedforward", "ON" if lb.enable_feedforward else "OFF"),
                ("Anti-windup", "ON" if lb.enable_antiwindup else "OFF"),
                ("Ref. filter", "ON" if lb.enable_referencefilter else "OFF"),
            ]
        rows += [("Performance Index", "Value")]
        for k in ("SAE1", "SCI1", "J1"):
            v = m.get(k, float("nan"))
            rows.append((k, "-" if v is None or math.isnan(v) else f"{v:.4f}"))
        table = Table(self.ax_table, bbox=[0.05, 0.08, 0.9, 0.84])
        for i, (k, v) in enumerate(rows):
            for j, txt in enumerate([k, v]):
                cell = table.add_cell(i, j, width=[0.6, 0.3][j], height=0.07, text=txt,
                                      loc="left" if j == 0 else "right")
                cell.set_edgecolor("#CCCCCC")
                cell.set_fontsize(11)
                bold = txt in ("Control Params", "Performance Index") or (j == 1 and k in ("Control Params", "Performance Index"))
                cell.set_text_props(weight="bold" if bold else "normal")
        self.ax_table.add_table(table)
        self.fig.tight_layout()

    def update_plot(self):
        if self.done:
            return
        self._draw()
        self.fig.canvas.draw_idle()

    def save_figures(self, out_dir: str):
        if self.fig is None:
            self._create_figure()
        self._draw()
        self.save_fig(self.fig, os.path.join(out_dir, "category_1_low_level.png"))


def main():
    rclpy.init()
    node = PlotterC1()
    run_recorder(node, node.plt if node.plot else None)


if __name__ == "__main__":
    main()
