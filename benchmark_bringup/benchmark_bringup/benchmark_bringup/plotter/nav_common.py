# +-------------------------------------------------------------------------+
# |                       Benchmark control simulator                       |
# |                                                                         |
# | Copyright (C) 2025  Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | Common recorder for categories 2 and 3 (navigation signals).            |
# +-------------------------------------------------------------------------+
"""Navigation recorder shared by the plotters of categories 2 and 3.

Logged signals (wall-clock timestamps, from the message header when available):
  * ground-truth pose  (/base_pose_ground_truth, MVSim)  -> used by the metrics
  * estimated pose     (/amcl_pose)                     -> logged for reference
  * mid-level command  (/cmd_vel_nav, controller output)
  * minimum LiDAR range (/laser1, r >= lidar_min_range) -> distance to obstacles
  * collision flag     (/collision, MVSim ground truth)  -> contacts
  * local path of the controller (param local_path_topic, nav_msgs/Path)
                                                       -> mid-level reference (SAE2)
"""

from __future__ import annotations

import math
import os
from typing import List

import numpy as np
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from benchmark_bringup.metrics import low_level_indices, total_index
from benchmark_bringup.plotter.common import BenchmarkRecorder


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def prediction_error_series(rec, t1):
    """(t, e) of the mid-level prediction error for the figures."""
    from benchmark_bringup.metrics import prediction_error
    t0, _ = rec.window()
    tg, xg, yg, _ = rec.cols(rec.gt_rows, 4)
    return prediction_error(tg, xg, yg, rec.local_plans, rec.cfg, t0, t1)


def fit_xy(ax, *xy, margin: float = 1.0):
    """x-y axes limited to the data. axis('equal') in the wide panel stretched the
    x range to ~75 m and the trajectory became a thin line; here the axes keep the
    data range (scales of x and y may differ: read the tick labels)."""
    pts = [np.asarray(a, dtype=float).reshape(-1, 2) for a in xy if a is not None and len(a)]
    pts = [a[np.all(np.isfinite(a), axis=1)] for a in pts]
    pts = [a for a in pts if a.size]
    if not pts:
        return
    a = np.vstack(pts)
    ax.set_xlim(a[:, 0].min() - margin, a[:, 0].max() + margin)
    ax.set_ylim(a[:, 1].min() - margin, a[:, 1].max() + margin)
    ax.set_aspect("auto")


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavRecorder(BenchmarkRecorder):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.gt_rows: List[list] = []     # t, x, y, yaw
        self.est_rows: List[list] = []    # t, x, y, yaw
        self.cmd_rows: List[list] = []    # t, v, w
        self.scan_rows: List[list] = []   # t, min_range
        self.col_rows: List[list] = []    # t, collision (0/1)
        self.local_xy = np.zeros((0, 2))
        self.local_plans: List[tuple] = []  # (t_reception, Nx2) local paths of the controller

        self.create_subscription(Odometry, "/base_pose_ground_truth", self._cb_gt, 50)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._cb_est, 20)
        self.create_subscription(Twist, "/cmd_vel_nav", self._cb_cmd, 50)
        self.create_subscription(LaserScan, "/laser1", self._cb_scan, 10)
        self.create_subscription(Path, self.local_path_topic, self._cb_local_path, 10)
        self.create_subscription(Bool, "/collision", self._cb_col, 50)
        self.create_timer(0.2, self.check_termination)

    # --------------------------------------------------------------- callbacks
    def _stamp(self, header) -> float:
        t = stamp_to_sec(header.stamp)
        return t if t > 0 else self.now()

    def _cb_gt(self, msg: Odometry):
        if self.done:
            return
        p = msg.pose.pose
        self.gt_rows.append([self._stamp(msg.header), p.position.x, p.position.y,
                             yaw_from_quat(p.orientation)])

    def _cb_est(self, msg: PoseWithCovarianceStamped):
        if self.done:
            return
        p = msg.pose.pose
        self.est_rows.append([self._stamp(msg.header), p.position.x, p.position.y,
                              yaw_from_quat(p.orientation)])

    def _cb_cmd(self, msg: Twist):
        if self.done:
            return
        self.cmd_rows.append([self.now(), msg.linear.x, msg.angular.z])

    def _cb_scan(self, msg: LaserScan):
        if self.done:
            return
        r = np.asarray(msg.ranges, dtype=float)
        # The LiDAR is inside the robot: returns closer than lidar_min_range hit the
        # robot's own structure and are discarded (as on the real robot).
        r_low = max(msg.range_min, self.cfg.lidar_min_range, 1e-3)
        valid = np.isfinite(r) & (r >= r_low) & (r <= msg.range_max)
        rmin = float(np.min(r[valid])) if np.any(valid) else float("nan")
        self.scan_rows.append([self._stamp(msg.header), rmin])

    def _cb_col(self, msg: Bool):
        if self.done:
            return
        self.col_rows.append([self.now(), 1.0 if msg.data else 0.0])

    def _cb_local_path(self, msg: Path):
        # Reception time: the stamp of the message is the planning time for some
        # controllers (TEB, DWB) but the global-plan time for others (RPP, MPPI).
        if self.done:
            return
        xy = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses]).reshape(-1, 2)
        # Some controllers publish their path in the robot frame (RPP in Humble:
        # /received_global_plan in base_link). Express it in the world frame with
        # the latest ground-truth pose (with fake localization map = odom = world).
        frame = msg.header.frame_id.lstrip("/")
        if xy.shape[0] and frame not in ("map", "odom", "world", ""):
            if not self.gt_rows:
                return
            _, x0, y0, th = self.gt_rows[-1]
            c, s_ = math.cos(th), math.sin(th)
            xy = np.column_stack((x0 + c * xy[:, 0] - s_ * xy[:, 1],
                                  y0 + s_ * xy[:, 0] + c * xy[:, 1]))
        self.local_xy = xy
        if xy.shape[0]:
            self.local_plans.append((self.now(), xy))

    # ----------------------------------------------------------------- helpers
    @staticmethod
    def cols(rows: List[list], n: int):
        if not rows:
            return [np.zeros(0) for _ in range(n)]
        a = np.asarray(rows, dtype=float)
        return [a[:, i] for i in range(n)]

    @staticmethod
    def crop(t1, t, *arrays):
        """Keep the samples up to the end of the evaluation window (figures)."""
        t = np.asarray(t, dtype=float)
        m = t <= t1
        return [t[m]] + [np.asarray(a)[m] for a in arrays]

    def window(self):
        """Evaluation window in wall-clock time: [STARTED, terminal event or now]."""
        t0 = self.t0_wall
        if t0 is None:
            t0 = self.gt_rows[0][0] if self.gt_rows else self.now()
        t1 = self.t_end_wall if self.t_end_wall is not None else self.now()
        t1 = min(t1, t0 + self.max_time)
        return t0, t1

    def low_level_metrics(self, t0: float, t1: float) -> dict:
        c = self.bench_columns()
        if not c["t_wall"]:
            return {}
        return low_level_indices(c["t_wall"], c["error_l"], c["error_r"],
                                 c["torque_l"], c["torque_r"], self.cfg, t0, t1)

    def write_extra_csv(self, out_dir: str):
        self.write_csv(os.path.join(out_dir, "pose_ground_truth.csv"), ["t_wall", "x", "y", "yaw"], self.gt_rows)
        self.write_csv(os.path.join(out_dir, "pose_estimated.csv"), ["t_wall", "x", "y", "yaw"], self.est_rows)
        self.write_csv(os.path.join(out_dir, "cmd_vel_nav.csv"), ["t_wall", "v", "w"], self.cmd_rows)
        self.write_csv(os.path.join(out_dir, "scan_min_range.csv"), ["t_wall", "min_range"], self.scan_rows)
        self.write_csv(os.path.join(out_dir, "collision.csv"), ["t_wall", "collision"], self.col_rows)
        rows = []
        for k, (t, xy) in enumerate(self.local_plans):
            rows += [[k, t, x, y] for x, y in xy]
        self.write_csv(os.path.join(out_dir, "local_plans.csv"), ["plan_id", "t_wall", "x", "y"], rows)

    # ------------------------------------------------------------ low-level fig
    def create_low_level_figure(self, title: str):
        from matplotlib.gridspec import GridSpec

        fig = self.plt.figure(figsize=(17, 10))
        gs = GridSpec(4, 2, width_ratios=[3.6, 1.4], figure=fig)
        axes = {
            "L": fig.add_subplot(gs[0, 0]), "R": fig.add_subplot(gs[1, 0]),
            "tau": fig.add_subplot(gs[2, 0]), "dist": fig.add_subplot(gs[3, 0]),
            "table": fig.add_subplot(gs[:, 1]),
        }
        axes["dist_r"] = axes["dist"].twinx()
        axes["table"].axis("off")
        fig._bench_title = title
        return fig, axes

    def draw_low_level(self, fig, axes, t0: float):
        c = self.bench_columns()
        if not c["t_wall"]:
            return
        _, t1 = self.window()
        keep = np.asarray(c["t_wall"]) <= t1      # up to the end of the run
        c = {k: (np.asarray(v)[keep] if len(v) == keep.size else v) for k, v in c.items()}
        t = np.asarray(c["t_wall"]) - t0
        for k in ("L", "R", "tau", "dist", "dist_r"):
            axes[k].cla()
        axes["L"].set_title(fig._bench_title)
        axes["L"].plot(t, c["sp_vel_l"], color="#d62728", label=r"sp $v_L$")
        axes["L"].plot(t, c["act_vel_l"], "--", color="#2c41a0", label=r"$v_L$")
        axes["L"].set_ylabel(r"Left $v$ [m/s]")
        axes["R"].plot(t, c["sp_vel_r"], color="#af1fb4", label=r"sp $v_R$")
        axes["R"].plot(t, c["act_vel_r"], "--", color="#0ee3ff", label=r"$v_R$")
        axes["R"].set_ylabel(r"Right $v$ [m/s]")
        axes["tau"].plot(t, c["torque_l"], color="#67bd72", label=r"$\tau_L$")
        axes["tau"].plot(t, c["torque_r"], "--", color="#a06a2c", label=r"$\tau_R$")
        axes["tau"].set_ylabel(r"Torque $\tau$ [N m]")
        axes["dist"].plot(t, np.degrees(c["pitch_rad"]), color="#08519c", label="Slope")
        axes["dist_r"].step(t, c["sector"], where="post", linewidth=2.5, color="#a50f15",
                            label="Terrain sector")
        axes["dist"].set_ylabel(r"Slope [$\degree$]")
        axes["dist_r"].set_ylabel("Sector")
        axes["dist_r"].yaxis.set_label_position("right")
        axes["dist_r"].yaxis.tick_right()
        axes["dist_r"].set_yticks([0, 1, 2, 3])
        axes["dist_r"].set_yticklabels(["-", "S1", "S2", "S3"])
        axes["dist"].set_xlabel("Time [s]")
        for k in ("L", "R", "tau", "dist"):
            axes[k].grid(True)
            axes[k].set_xlim(left=0)
            if k != "dist":
                axes[k].legend(loc="upper right")

    def draw_table(self, ax, rows):
        from matplotlib.table import Table

        ax.cla()
        ax.axis("off")
        table = Table(ax, bbox=[0.05, 0.05, 0.9, 0.9])
        h = 1.0 / max(len(rows), 1)
        for i, (k, v) in enumerate(rows):
            for j, txt in enumerate([k, v]):
                cell = table.add_cell(i, j, width=[0.62, 0.32][j], height=h, text=txt,
                                      loc="left" if j == 0 else "right")
                cell.set_edgecolor("#CCCCCC")
                cell.set_fontsize(11)
                cell.set_text_props(weight="bold" if v == "" or i == 0 else "normal")
        ax.add_table(table)

    @staticmethod
    def fmt(v) -> str:
        if v is None:
            return "-"
        if isinstance(v, (int, np.integer)):
            return str(int(v))
        return "-" if (isinstance(v, float) and math.isnan(v)) else f"{v:.4f}"

    @staticmethod
    def total(*vals) -> float:
        return total_index(*vals)
