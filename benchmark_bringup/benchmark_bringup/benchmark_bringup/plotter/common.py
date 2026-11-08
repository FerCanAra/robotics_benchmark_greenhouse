# +-------------------------------------------------------------------------+
# |                       Benchmark control simulator                       |
# |                                                                         |
# | Copyright (C) 2025  Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | Common recorder for the benchmark plotters: raw data logging with       |
# | consistent timestamps, run metadata and end-of-run handling.            |
# +-------------------------------------------------------------------------+
"""Base class shared by the three plotters.

Every plotter:
  * logs the raw signals of the run (no metric is computed from truncated buffers);
  * timestamps every sample with the wall clock (MVSim runs in real time and does
    not publish /clock) and, for the low-level signals, also with the simulated time;
  * synchronises the start of the evaluation window with the category script,
    which publishes a STARTED event on /benchmark/status;
  * at the end of the run writes the CSV files, metadata, metrics and figures to
    <results_dir>/category_<N>/<run_name>/.
"""

from __future__ import annotations

import csv
import json
import os
import time
from datetime import datetime
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

from benchmark_msg.msg import BenchmarkParams

from benchmark_bringup.metrics import MetricsConfig, sector_from_mu

STATUS_TOPIC = "/benchmark/status"
TERMINAL_STATES = ("SUCCEEDED", "FAILED", "CANCELED", "TIMEOUT", "FINISHED")

BENCHMARK_HEADER = [
    "t_wall", "sim_time", "sp_vel_l", "sp_vel_r", "act_vel_l", "act_vel_r",
    "error_l", "error_r", "torque_l", "torque_r", "pitch_rad", "mu", "sector",
    "torque_slope", "max_torque",
]


def status_qos() -> QoSProfile:
    """Latched (transient local) QoS so late subscribers receive past events."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def setup_matplotlib(plot: bool):
    import matplotlib

    matplotlib.use("TkAgg" if plot else "Agg")
    import matplotlib as mpl
    import matplotlib.pyplot as plt

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
    if plot:
        plt.ion()
    return plt


class BenchmarkRecorder(Node):
    """Base node: parameters, status events, low-level logging and run output."""

    CATEGORY = 0

    def __init__(self, node_name: str):
        super().__init__(node_name)

        dyn = ParameterDescriptor(dynamic_typing=True)
        defaults = {
            "plot": True,
            "auto_shutdown": False,
            "results_dir": "result",
            "run_name": "",
            "seed": -1,
            "payload": 0.0,
            "terrain_slope": "false",
            "change_terrain": "false",
            "localization": "fake",
            "controller": "teb",
            "planner": "",
            "local_path_topic": "/local_plan",
            "max_time": 180.0,        # [s] termination rule for categories 2-3
            "t_end": 55.0,            # [s] max. evaluation window of category 1 (simulated time)
            "t_settle": 2.0,          # [s] category 1: the window closes t_settle after PROFILE_END
            "v_wheel_max": 1.0,
            "v_max": 0.5,
            "w_max": 1.0,             # [rad/s] (set by the launch argument w_max)
            "d_ref": 2.0,
            "fs": 10.0,
            "horizon": 1.0,           # [s] prediction horizon of SAE2
            "lidar_min_range": 0.5,
            "goal_tolerance": 0.3,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v, dyn)

        def p(name):
            return self.get_parameter(name).value

        self.plot = str(p("plot")).lower() == "true"
        self.auto_shutdown = str(p("auto_shutdown")).lower() == "true"
        # Matplotlib backend: interactive only when plotting (batch runs use Agg)
        self.plt = setup_matplotlib(self.plot)
        self.results_dir = str(p("results_dir"))
        self.run_name = str(p("run_name")) or datetime.now().strftime("%Y%m%d_%H%M%S")
        self.meta = {
            "category": self.CATEGORY,
            "run_name": self.run_name,
            "seed": int(float(p("seed"))),
            "payload_kg": float(p("payload")),
            "terrain_slope": str(p("terrain_slope")).lower() == "true",
            "change_terrain": str(p("change_terrain")).lower() == "true",
            "localization": str(p("localization")),
            "controller": str(p("controller")),
            "planner": str(p("planner")),
            "local_path_topic": str(p("local_path_topic")),
            "date": datetime.now().isoformat(timespec="seconds"),
        }
        self.max_time = float(p("max_time"))
        self.controller = str(p("controller"))
        self.planner = str(p("planner"))
        self.local_path_topic = str(p("local_path_topic"))
        self.t_end = float(p("t_end"))
        self.t_settle = float(p("t_settle"))
        self.cfg = MetricsConfig(
            v_wheel_max=float(p("v_wheel_max")),
            v_max=float(p("v_max")),
            w_max=float(p("w_max")),
            d_ref=float(p("d_ref")),
            fs=float(p("fs")),
            horizon=float(p("horizon")),
            lidar_min_range=float(p("lidar_min_range")),
            goal_tolerance=float(p("goal_tolerance")),
        )

        # Run state
        self.events: List[dict] = []
        self.t0_wall: Optional[float] = None     # start of the evaluation window (wall clock)
        self.t0_sim: Optional[float] = None      # start of the evaluation window (sim time)
        self.t_end_wall: Optional[float] = None
        self.final_status = ""
        self.route = None                        # waypoints / goal published by the script
        self.done = False
        self._terminal_seen_at: Optional[float] = None
        self._finish_request: Optional[str] = None   # set by the ROS thread, served by the main thread
        self.plot_period = 1.0                        # [s] refresh period of the live figures

        # Low-level data (all categories)
        self.bench_rows: List[list] = []
        self.last_bench: Optional[BenchmarkParams] = None

        self.create_subscription(String, STATUS_TOPIC, self._cb_status, status_qos())
        self.create_subscription(BenchmarkParams, "/benchmark_params", self._cb_bench, 100)

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def now() -> float:
        return time.time()

    def rel(self, t_wall: float) -> float:
        return t_wall - (self.t0_wall if self.t0_wall is not None else t_wall)

    # ------------------------------------------------------------------ callbacks
    def _cb_status(self, msg: String):
        try:
            ev = json.loads(msg.data)
        except json.JSONDecodeError:
            ev = {"state": msg.data}
        ev.setdefault("t_wall", self.now())
        self.events.append(ev)
        state = str(ev.get("state", "")).upper()
        if state == "STARTED":
            self.t0_wall = float(ev.get("t_wall", self.now()))
            if ev.get("t0_sim") is not None:
                self.t0_sim = float(ev["t0_sim"])
            if ev.get("route") is not None:
                self.route = ev["route"]
            self.get_logger().info("Run started: evaluation window opened.")
        elif state in TERMINAL_STATES and not self.final_status:
            self.final_status = state
            self.t_end_wall = float(ev.get("t_wall", self.now()))
            self._terminal_seen_at = self.now()
            self.get_logger().info(f"Run finished with status {state}.")

    def _cb_bench(self, msg: BenchmarkParams):
        if self.done:
            return
        self.last_bench = msg
        self.cfg.tau_max = msg.max_torque if msg.max_torque > 0 else self.cfg.tau_max
        sector = sector_from_mu(msg.mu, [msg.mu_zone1, msg.mu_zone2, msg.mu_zone3])
        self.bench_rows.append(
            [
                self.now(), msg.sim_time, msg.sp_vel_l, msg.sp_vel_r, msg.act_vel_l,
                msg.act_vel_r, msg.error_l, msg.error_r, msg.torque_l, msg.torque_r,
                msg.pitch, msg.mu, sector, msg.torque_slope, msg.max_torque,
            ]
        )
        self.on_bench(msg)

    def on_bench(self, msg: BenchmarkParams):
        """Hook for subclasses (e.g. category 1 termination in simulated time)."""

    # ------------------------------------------------------------------ output
    def run_dir(self) -> str:
        d = os.path.join(self.results_dir, f"category_{self.CATEGORY}", self.run_name)
        os.makedirs(d, exist_ok=True)
        return d

    @staticmethod
    def write_csv(path: str, header: List[str], rows: List[list]):
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(rows)

    def bench_columns(self) -> Dict[str, list]:
        cols = {h: [] for h in BENCHMARK_HEADER}
        for r in self.bench_rows:
            for h, v in zip(BENCHMARK_HEADER, r):
                cols[h].append(v)
        return cols

    def finish(self, status: str = ""):
        """Close the run: compute metrics, write all the files and save figures."""
        if self.done:
            return
        self.done = True
        if status and not self.final_status:
            self.final_status = status
        if self.t_end_wall is None:
            self.t_end_wall = self.now()
        out = self.run_dir()
        self.write_csv(os.path.join(out, "benchmark.csv"), BENCHMARK_HEADER, self.bench_rows)
        self.write_extra_csv(out)
        metrics = self.compute_metrics()
        quality = self.run_quality()
        metrics["run_quality"] = quality
        if not quality["rtf_valid"]:
            self.get_logger().warn(
                f"Real-time factor of the run = {quality['real_time_factor']:.3f} "
                f"(max. data gap {quality['max_data_gap_s']:.2f} s): the simulation did "
                "not run in real time, the run should be discarded/repeated."
            )
        self.meta.update(
            {
                "final_status": self.final_status,
                "t0_wall": self.t0_wall,
                "t0_sim": self.t0_sim,
                "t_end_wall": self.t_end_wall,
                "run_quality": quality,
                "metrics_config": self.cfg.as_dict(),
            }
        )
        with open(os.path.join(out, "meta.json"), "w") as f:
            json.dump(self.meta, f, indent=2)
        with open(os.path.join(out, "events.json"), "w") as f:
            json.dump(self.events, f, indent=2)
        with open(os.path.join(out, "metrics.json"), "w") as f:
            json.dump(metrics, f, indent=2)
        try:
            self.save_figures(out)
        except Exception as e:  # figures must never prevent saving the data
            self.get_logger().warn(f"Could not save figures: {e}")
        self.get_logger().info(f"Results saved in: {os.path.abspath(out)}")
        self.get_logger().info("Metrics: " + json.dumps(metrics))

    def run_quality(self, rtf_tol: float = 0.05) -> dict:
        """Real-time factor (simulated time / wall-clock time) of the evaluation
        window, and the largest gap between consecutive /benchmark_params samples.
        MVSim runs against the wall clock, so a run is only valid if RTF ~ 1."""
        t0 = self.t0_wall if self.t0_wall is not None else -float("inf")
        t1 = self.t_end_wall if self.t_end_wall is not None else float("inf")
        rows = [r for r in self.bench_rows if t0 <= r[0] <= t1]
        if len(rows) < 2:
            return {"real_time_factor": float("nan"), "max_data_gap_s": float("nan"),
                    "rtf_valid": False, "n_samples": len(rows)}
        d_wall = rows[-1][0] - rows[0][0]
        d_sim = rows[-1][1] - rows[0][1]
        rtf = d_sim / d_wall if d_wall > 0 else float("nan")
        gap = max(b[0] - a[0] for a, b in zip(rows[:-1], rows[1:]))
        # Also the silence at the end of the window: if the data stop before the run
        # ends (simulator or recorder stalled), the run is not valid either.
        if self.t_end_wall is not None:
            gap = max(gap, self.t_end_wall - rows[-1][0])
        return {
            "real_time_factor": rtf,
            "max_data_gap_s": gap,
            "rtf_valid": bool(abs(rtf - 1.0) <= rtf_tol and gap < 0.5),
            "n_samples": len(rows),
        }

    # Hooks to be implemented by the subclasses
    def write_extra_csv(self, out_dir: str):
        pass

    def compute_metrics(self) -> dict:
        return {}

    def save_figures(self, out_dir: str):
        pass

    @staticmethod
    def save_fig(fig, path: str, dpi: int = 150):
        """Save a figure even if its window was closed during the run (the Tk canvas
        is then destroyed): fall back to an off-screen Agg canvas."""
        try:
            fig.savefig(path, dpi=dpi)
        except Exception:
            from matplotlib.backends.backend_agg import FigureCanvasAgg
            FigureCanvasAgg(fig)
            fig.savefig(path, dpi=dpi)

    def request_finish(self, status: str = ""):
        """Ask the main thread to close the run (finish() writes files and figures,
        so it must not run in the ROS executor thread)."""
        if self._finish_request is None:
            self._finish_request = status

    def update_plot(self):
        """Live figures (implemented by the plotters). Called from the main thread."""

    def check_termination(self):
        """Categories 2-3: end at a terminal status (+1 s) or after max_time."""
        if self.done or self._finish_request is not None:
            return
        if self._terminal_seen_at is not None and self.now() - self._terminal_seen_at > 1.0:
            self.request_finish()
        elif self.t0_wall is not None and self.now() - self.t0_wall > self.max_time:
            self.t_end_wall = self.t0_wall + self.max_time
            self.request_finish("TIMEOUT")


def pump_gui(plt):
    """Process the pending GUI events of the open figures (draw_idle requests),
    without stealing the focus as plt.pause() does."""
    from matplotlib._pylab_helpers import Gcf

    for manager in Gcf.get_all_fig_managers():
        try:
            manager.canvas.flush_events()
        except Exception:
            pass


def run_recorder(node: BenchmarkRecorder, plt=None):
    """Run the recorder until the run is finished (and the windows are closed).

    The ROS callbacks run in a background executor thread, so drawing the live
    figures never delays the reception (and the timestamps) of the messages.
    Matplotlib and the end-of-run output run in the main thread."""
    import threading
    from rclpy.executors import SingleThreadedExecutor

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    def stop_ros():
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=2.0)

    last_plot = 0.0
    try:
        while rclpy.ok() and not node.done:
            if node._finish_request is not None:
                stop_ros()
                node.finish(node._finish_request)
                break
            if plt is not None and node.plot and time.time() - last_plot >= node.plot_period:
                last_plot = time.time()
                try:
                    node.update_plot()
                except Exception as e:  # a drawing error must not stop the logging
                    node.get_logger().warn(f"Live plot error: {e}")
            if plt is not None and node.plot:
                pump_gui(plt)
            time.sleep(0.05)
        # Keep the windows open until the user closes them (interactive runs)
        if node.done and plt is not None and node.plot and not node.auto_shutdown:
            # (plt.pause() would raise the windows to the front every 0.1 s and they
            # could not be minimised: only process their events)
            while plt.get_fignums():
                pump_gui(plt)
                time.sleep(0.05)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C: closing the run and saving the results...")
    finally:
        if spin_thread.is_alive():
            stop_ros()
        if not node.done:
            node.finish("INTERRUPTED")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if plt is not None:
            plt.close("all")
