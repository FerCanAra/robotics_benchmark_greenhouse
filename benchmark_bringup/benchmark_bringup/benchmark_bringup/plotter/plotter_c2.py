#!/usr/bin/env python3
# +-------------------------------------------------------------------------+
# |                       Benchmark control simulator                       |
# |                                                                         |
# | Copyright (C) 2025  Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | Category 2: PID Control + MPC Control                                   |
# +-------------------------------------------------------------------------+

# ---------------------------------------------------------------------------
#               Mid Level Control Plotter - Category 2
# ---------------------------------------------------------------------------
# There is no reference path in this category (the area is not mapped): the
# robot follows the waypoints with the local planner only. Tracking error:
# prediction error of the local planner: distance from the ground-truth robot
# position at t + h (h = 1 s) to the local path of the controller at t (the
# trajectory it plans, or the path it follows for path followers),
# normalised by the fixed distance d_ref. All the
# signals are resampled to a common time base (fs) using their timestamps.

import os

import numpy as np
import rclpy

from benchmark_bringup.metrics import mid_level_indices, path_quality
from benchmark_bringup.plotter.common import run_recorder
from benchmark_bringup.metrics import dist_points_polyline
from benchmark_bringup.plotter.nav_common import NavRecorder, prediction_error_series, fit_xy

# Default route (same as categories/categorie_2.py). The script publishes the
# route actually used in the STARTED event, which takes precedence.
DEFAULT_ROUTE = [(10.2, 13.7), (15.0, 13.7), (18.0, 13.7), (18.0, 17.4)]  # same as categories/categorie_2.py


class PlotterC2(NavRecorder):
    CATEGORY = 2

    def __init__(self):
        super().__init__("plotter_c2_benchmark")
        self.fig1 = self.fig2 = None
        if self.plot:
            self._create_figures()
            self.plot_period = 1.0

    # ---------------------------------------------------------------- waypoints
    def waypoints(self) -> np.ndarray:
        """Route sent to the waypoint follower (there is no reference path in this
        category: the local planner is the reference of the mid level)."""
        route = self.route if self.route else DEFAULT_ROUTE
        return np.asarray([tuple(p[:2]) for p in route], dtype=float)

    def reference_path(self, t0) -> np.ndarray:
        """Common reference of every controller: the polyline start pose -> waypoints
        sent through FollowPath (start = ground-truth position at the start of the task)."""
        wp = self.waypoints()
        tg, xg, yg, _ = self.cols(self.gt_rows, 4)
        if len(tg) == 0:
            return wp
        i = int(np.searchsorted(np.asarray(tg), t0 if t0 is not None else tg[0]))
        i = min(max(i, 0), len(tg) - 1)
        return np.vstack([[xg[i], yg[i]], wp])

    # ----------------------------------------------------------------- metrics
    def compute_metrics(self) -> dict:
        t0, t1 = self.window()
        m = dict(self.low_level_metrics(t0, t1))
        tg, xg, yg, _ = self.cols(self.gt_rows, 4)
        tc, vc, wc = self.cols(self.cmd_rows, 3)
        wp = self.waypoints()
        m.update(mid_level_indices(tg, xg, yg, tc, vc, wc, self.local_plans, self.cfg, t0, t1,
                                   ref_path=self.reference_path(t0)))
        ts, rs = self.cols(self.scan_rows, 2)
        goal = tuple(wp[-1]) if wp.shape[0] else None
        q = path_quality(tg, xg, yg, goal, ts, rs, self.cfg, t0, t1,
                         final_status=self.final_status,
                         t_collision=self.cols(self.col_rows, 2)[0],
                         collision=self.cols(self.col_rows, 2)[1])
        q.pop("reference_length", None)   # no reference path in category 2
        q.pop("length_ratio", None)
        m.update(q)
        m["JT2"] = self.total(m.get("J1"), m.get("J2"))
        return m

    def write_extra_csv(self, out_dir: str):
        super().write_extra_csv(out_dir)
        self.write_csv(os.path.join(out_dir, "waypoints.csv"), ["x", "y"], self.waypoints().tolist())

    # ------------------------------------------------------------------ figures
    def _create_figures(self):
        from matplotlib.gridspec import GridSpec

        self.fig1, self.ax1 = self.create_low_level_figure(
            "Category 2: Low Level Control. Linear Velocities of Left and Right Wheels")
        self.fig2 = self.plt.figure(figsize=(17, 10))
        gs = GridSpec(3, 2, width_ratios=[3.6, 1.4], figure=self.fig2)
        self.ax_xy = self.fig2.add_subplot(gs[0, 0])
        self.ax_u = self.fig2.add_subplot(gs[1, 0])
        self.ax_e = self.fig2.add_subplot(gs[2, 0])
        self.ax_t2 = self.fig2.add_subplot(gs[:, 1])
        self.ax_t2.axis("off")

    def _draw(self):
        t0, t1 = self.window()
        self.draw_low_level(self.fig1, self.ax1, t0)
        m = self.compute_metrics()
        lb = self.last_bench
        rows1 = [("Control Params", "Values")]
        if lb is not None:
            rows1 += [("Kp", f"{lb.kp:.2f}"), ("Ki", f"{lb.ki:.2f}"), ("Kd", f"{lb.kd:.2f}"),
                      ("Anti-windup", "ON" if lb.enable_antiwindup else "OFF")]
        rows1 += [("Performance Index", ""), ("SAE1", self.fmt(m.get("SAE1"))),
                  ("SCI1", self.fmt(m.get("SCI1"))), ("J1", self.fmt(m.get("J1")))]
        self.draw_table(self.ax1["table"], rows1)
        self.fig1.tight_layout()

        # Trajectory, commands and tracking error
        tg, xg, yg = self.crop(t1, *self.cols(self.gt_rows, 4)[:3])
        wp = self.waypoints()
        for ax in (self.ax_xy, self.ax_u, self.ax_e):
            ax.cla()
        self.ax_xy.set_title("Category 2: Mid Level Control. Trajectory tracking (ground truth)")
        self.ax_xy.plot(xg, yg, color="#2c41a0", label="Robot")
        if self.local_xy.shape[0]:
            self.ax_xy.plot(self.local_xy[:, 0], self.local_xy[:, 1], ".", color="#ff7f0e",
                            markersize=3, label=f"{self.controller.upper()} local path")
        self.ax_xy.scatter(wp[:, 0], wp[:, 1], marker="x", color="#a50f15", label="Waypoints")
        self.ax_xy.set_xlabel("x [m]")
        self.ax_xy.set_ylabel("y [m]")
        fit_xy(self.ax_xy, wp, np.column_stack([xg, yg]) if len(xg) else None)
        self.ax_xy.legend(loc="best")
        self.ax_xy.grid(True)

        tc, vc, wc = self.crop(t1, *self.cols(self.cmd_rows, 3))
        self.ax_u.plot(tc - t0, vc, color="#1f77b4", label=r"$v$ [m/s]")
        self.ax_u.plot(tc - t0, wc, "--", color="#9467bd", label=r"$\omega$ [rad/s]")
        self.ax_u.set_ylabel("Command")
        self.ax_u.legend(loc="upper right")
        self.ax_u.grid(True)

        if tg.size:
            ref = self.reference_path(t0)
            tge, xge, yge = self.crop(t1, *self.cols(self.gt_rows, 4)[:3])
            if len(tge) and len(ref) >= 2:
                e = dist_points_polyline(np.asarray(xge), np.asarray(yge), ref)
                self.ax_e.plot(np.asarray(tge) - t0, e, color="#2ca02c",
                               label=f"Mid-level error, {self.controller.upper()} (distance to the reference path)")
        self.ax_e.set_xlabel("Time [s]")
        self.ax_e.set_ylabel("Error [m]")
        self.ax_e.legend(loc="upper right")
        self.ax_e.grid(True)
        for ax in (self.ax_u, self.ax_e):
            ax.set_xlim(left=0)

        rows2 = [(f"Controller: {self.controller.upper()}", ""), (r"v_max [m/s]", f"{self.cfg.v_max:.2f}"),
                 (r"w_max [rad/s]", f"{self.cfg.w_max:.2f}"), ("d_ref [m]", f"{self.cfg.d_ref:.2f}"),
                 ("Performance Index", ""),
                 ("SAE2", self.fmt(m.get("SAE2"))), ("SCI2", self.fmt(m.get("SCI2"))),
                 ("J2", self.fmt(m.get("J2"))), ("JT2 = (J1+J2)/2", self.fmt(m.get("JT2"))),
                 ("Path quality", ""), ("Path length [m]", self.fmt(m.get("path_length"))),
                 ("Min obstacle dist. [m]", self.fmt(m.get("min_obstacle_dist"))),
                 ("Contacts", self.fmt(m.get("contacts"))), ("Success", self.fmt(m.get("success")))]
        self.draw_table(self.ax_t2, rows2)
        self.fig2.tight_layout()

    def update_plot(self):
        if self.done:
            return
        self._draw()
        for f in (self.fig1, self.fig2):
            f.canvas.draw_idle()
            f.canvas.flush_events()

    def save_figures(self, out_dir: str):
        if self.fig1 is None:
            self._create_figures()
        self._draw()
        self.save_fig(self.fig1, os.path.join(out_dir, "category_2_low_level.png"))
        self.save_fig(self.fig2, os.path.join(out_dir, "category_2_mid_level.png"))


def main():
    rclpy.init()
    node = PlotterC2()
    run_recorder(node, node.plt if node.plot else None)


if __name__ == "__main__":
    main()
