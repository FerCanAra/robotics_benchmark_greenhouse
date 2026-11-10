#!/usr/bin/env python3
"""
Batch runner of the greenhouse benchmark.

Launches every (category x condition x seed) run without GUI, waits for it to
finish, checks that the simulation ran in real time (run_quality.rtf_valid) and that the
mid-level controller kept its rate (run_quality.controller_rate_valid),
repeats the invalid runs, measures the CPU/RAM of every process of the run
(R3.13) and writes a manifest with the software versions (R3.10).

Usage (from a terminal with the workspace sourced):
    source ~/code/simulator_control/install/setup.bash
    python3 run_batch.py --out ~/benchmark_results/v1.1.0 --categories 1 2 3 --seeds 10
    python3 run_batch.py --out ... --conditions nominal payload70 --seeds 10
    python3 run_batch.py --out ... --categories 2 --controllers teb mppi rpp dwb --conditions all --seeds 6
    python3 run_batch.py --out ... --categories 3 --planners theta smac navfn --conditions all --seeds 6
    python3 run_batch.py --out ... --dry-run           # only print the plan

The batch can be stopped (Ctrl+C) and restarted with the same command: the runs
that already have a valid metrics.json are skipped.

Results:  <out>/category_<N>/<label>_<condition>_s<seed>/
          label: pid (category 1), <controller> (category 2), <controller>-<planner> (category 3)   (CSVs, metrics.json, figures,
          resources.csv, batch_info.json)
          <out>/batch_manifest.json, <out>/batch_log.csv
"""

import argparse
import csv
import json
import os
import platform
import re
import signal
import subprocess
import sys
import time
from datetime import datetime

try:
    import psutil  # sudo apt install python3-psutil
except ImportError:
    psutil = None

# ---------------------------------------------------------------------------
# Experimental conditions (launch arguments). Edit / add conditions here.
# ---------------------------------------------------------------------------
CONDITIONS = {
    # name: payload [kg], terrain slope, change of terrain (soil sectors)
    "nominal":                 {"payload": "0",  "terrain_slope": "false", "change_terrain": "false"},
    "payload70":               {"payload": "70", "terrain_slope": "false", "change_terrain": "false"},
    "terrain":                 {"payload": "0",  "terrain_slope": "false", "change_terrain": "true"},
    "terrain_payload70":       {"payload": "70", "terrain_slope": "false", "change_terrain": "true"},
    "slope":                   {"payload": "0",  "terrain_slope": "true",  "change_terrain": "false"},
    "slope_payload70":         {"payload": "70", "terrain_slope": "true",  "change_terrain": "false"},
    "slope_terrain":           {"payload": "0",  "terrain_slope": "true",  "change_terrain": "true"},
    "slope_terrain_payload70": {"payload": "70", "terrain_slope": "true",  "change_terrain": "true"},
}

# Common launch arguments of every batch run
COMMON_ARGS = {
    "headless": "True",
    "use_rviz": "False",
    "plot": "False",
    "auto_shutdown": "True",
    "localization": "fake",
    "w_max": "1.0",       # [rad/s] angular limit of every mid-level controller
    "w_acc_max": "1.0",   # [rad/s^2]
}

# Max. wall-clock duration of one run, including the start-up of the stack [s]
TIMEOUT = {1: 120.0, 2: 300.0, 3: 300.0}   # task limit 180 s + start-up/shut-down

LAUNCH = ["ros2", "launch", "benchmark_bringup", "launch_benchmark_bringup.launch.py"]


# ---------------------------------------------------------------------------
def sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10).stdout.strip()
    except Exception:
        return ""


def environment_info():
    """Software versions, to document the experiments (README / Zenodo)."""
    def git(pkg):
        prefix = sh(f"ros2 pkg prefix {pkg}")
        # <ws>/install/<pkg> -> <ws>/src : look for the git repo of the package source
        src = sh(f"readlink -f $(ros2 pkg prefix {pkg})/share/{pkg}/package.xml")
        d = os.path.dirname(src) if src else ""
        commit = sh(f"git -C '{d}' rev-parse --short HEAD") if d else ""
        dirty = sh(f"git -C '{d}' status --porcelain") if d else ""
        return {"prefix": prefix, "source": d, "commit": commit, "uncommitted_changes": bool(dirty)}

    return {
        "date": datetime.now().isoformat(timespec="seconds"),
        "host": platform.node(),
        "os": sh("lsb_release -ds") or platform.platform(),
        "cpu": sh("lscpu | grep 'Model name' | sed 's/.*: *//'"),
        "gpu": sh("glxinfo -B 2>/dev/null | grep 'renderer string' | sed 's/.*: //'"),
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", "default (rmw_fastrtps_cpp)"),
        "mrpt_libbase": sh("dpkg-query -W -f='${Version}' ros-humble-mrpt-libbase"),
        "tf2_ros": sh("dpkg-query -W -f='${Version}' ros-humble-tf2-ros"),
        "nav2": sh("dpkg-query -W -f='${Version}' ros-humble-nav2-bringup"),
        "mvsim": git("mvsim"),
        "benchmark_bringup": git("benchmark_bringup"),
        "teb_local_planner": git("teb_local_planner"),
        "mrpt_ros_bridge": next((g for g in (git(n) for n in ("mrpt_libros_bridge", "mrpt_ros_bridge",
                                                                  "mrpt_ros2bridge")) if g["prefix"]), {}),
    }


# Max. fraction of control cycles that the mid-level controller may miss: above it,
# the computer could not run the controller at its rate and the run is not valid.
MAX_MISSED_CYCLES = 0.05
MISSED_RE = re.compile(r"Control loop missed its desired rate of ([0-9.]+)\s*Hz")


def annotate_controller_rate(run_dir, log_path):
    """Add the missed control cycles of the Nav2 controller to metrics.json."""
    mpath = os.path.join(run_dir, "metrics.json")
    try:
        m = json.load(open(mpath))
        meta = json.load(open(os.path.join(run_dir, "meta.json")))
    except Exception:
        return
    q = m.setdefault("run_quality", {})
    if "controller_rate_valid" in q or not os.path.isfile(log_path):
        return
    n_missed = 0
    rate = 0.0
    with open(log_path, errors="replace") as f:
        for line in f:
            r = MISSED_RE.search(line)
            if r:
                n_missed += 1
                rate = float(r.group(1))
    duration = (meta.get("t_end_wall") or 0) - (meta.get("t0_wall") or 0)
    frac = n_missed / (rate * duration) if rate > 0 and duration > 0 else 0.0
    q["controller_missed_cycles"] = n_missed
    q["controller_missed_fraction"] = frac
    q["controller_rate_valid"] = frac <= MAX_MISSED_CYCLES
    json.dump(m, open(mpath, "w"), indent=2)


def run_is_valid(run_dir):
    try:
        m = json.load(open(os.path.join(run_dir, "metrics.json")))
        q = m.get("run_quality", {})
        return bool(q.get("rtf_valid", False)) and bool(q.get("controller_rate_valid", True)), m
    except Exception:
        return False, None


class ResourceSampler:
    """CPU [%] and RSS [MB] of every process of the run, sampled every second."""

    def __init__(self, root_pid, path):
        self.root = psutil.Process(root_pid) if psutil else None
        self.f = open(path, "w", newline="") if psutil else None
        self.w = csv.writer(self.f) if self.f else None
        if self.w:
            self.w.writerow(["t_wall", "pid", "name", "cpu_percent", "rss_mb"])
        self.procs = {}

    def sample(self):
        if not self.root:
            return
        try:
            procs = [self.root] + self.root.children(recursive=True)
        except psutil.NoSuchProcess:
            return
        now = time.time()
        for q in procs:
            try:
                if q.pid not in self.procs:
                    # cpu_percent() measures since the previous call ON THE SAME Process
                    # object: keep the first object of each pid and always use it
                    self.procs[q.pid] = q
                    q.cpu_percent(None)  # first call initialises the counter (returns 0)
                    continue
                p = self.procs[q.pid]
                cmd = p.cmdline()
                name = os.path.basename(cmd[0]) if cmd else p.name()
                if name in ("python3", "python") and len(cmd) > 1:
                    name = os.path.basename(cmd[1])
                self.w.writerow([f"{now:.3f}", p.pid, name, f"{p.cpu_percent(None):.1f}",
                                 f"{p.memory_info().rss / 2**20:.1f}"])
            except (psutil.NoSuchProcess, psutil.AccessDenied, IndexError):
                pass

    def close(self):
        if self.f:
            self.f.close()


def stop_group(proc):
    """Ctrl+C to the whole launch (process group), then SIGKILL if needed."""
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    except ProcessLookupError:
        pass


def label(cat, ctrl, planner):
    return "pid" if cat == 1 else ctrl if cat == 2 else f"{ctrl}-{planner}"


def run_name(cat, ctrl, planner, cond, seed):
    return f"{label(cat, ctrl, planner)}_{cond}_s{seed:02d}"


def launch_run(out, cat, ctrl, planner, cond, seed, extra):
    name = run_name(cat, ctrl, planner, cond, seed)
    run_dir = os.path.join(out, f"category_{cat}", name)
    args = dict(COMMON_ARGS)
    args.update(CONDITIONS[cond])
    args.update(extra)
    args.update({"category": str(cat), "seed": str(seed), "results_dir": out, "run_name": name})
    if cat != 1:
        args["controller"] = ctrl
    if cat == 3:
        args["planner"] = planner
    cmd = LAUNCH + [f"{k}:={v}" for k, v in args.items()]

    os.makedirs(os.path.join(out, "logs"), exist_ok=True)
    log = open(os.path.join(out, "logs", f"c{cat}_{name}.log"), "w")
    t_start = time.time()
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    os.makedirs(run_dir, exist_ok=True)
    sampler = ResourceSampler(proc.pid, os.path.join(run_dir, "resources.csv"))
    status = "ok"
    try:
        while proc.poll() is None:
            sampler.sample()
            if time.time() - t_start > TIMEOUT[cat]:
                status = "timeout"
                break
            if os.path.exists(os.path.join(run_dir, "metrics.json")) and status == "ok":
                # results written: give the launch a few seconds to shut down by itself
                for _ in range(20):
                    if proc.poll() is not None:
                        break
                    time.sleep(0.5)
                break
            time.sleep(1.0)
    finally:
        sampler.close()
        stop_group(proc)
        log.close()
    annotate_controller_rate(run_dir, log.name)
    json.dump({"category": cat, "controller": "pid" if cat == 1 else ctrl,
               "planner": planner if cat == 3 else "", "condition": cond,
               "seed": seed, "launch_args": args,
               "command": " ".join(cmd), "wall_time_s": time.time() - t_start, "batch_status": status},
              open(os.path.join(run_dir, "batch_info.json"), "w"), indent=2)
    return run_dir, status


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="results folder (absolute or relative)")
    ap.add_argument("--categories", nargs="+", type=int, default=[1, 2, 3])
    ap.add_argument("--conditions", nargs="+", default=["nominal"], choices=list(CONDITIONS) + ["all"],
                    help="'all' = the 8 conditions")
    ap.add_argument("--controllers", nargs="+", default=["teb"], choices=["teb", "mppi", "rpp", "dwb"],
                    help="mid-level controllers of categories 2-3 (category 1 runs once: low-level PID)")
    ap.add_argument("--planners", nargs="+", default=["theta"], choices=["theta", "smac", "navfn"],
                    help="global planners of category 3")
    ap.add_argument("--seeds", type=int, default=6,
                    help="seeds 1..N (>= 6 so that the paired Wilcoxon test can reach p < 0.05)")
    ap.add_argument("--first-seed", type=int, default=1)
    ap.add_argument("--retries", type=int, default=3, help="repetitions of an invalid run")
    ap.add_argument("--arg", action="append", default=[], metavar="KEY:=VALUE",
                    help="extra launch argument for all the runs (e.g. --arg params_file_teb:=/path.yaml)")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    out = os.path.abspath(os.path.expanduser(a.out))
    extra = dict(s.split(":=", 1) for s in a.arg)
    if "all" in a.conditions:
        a.conditions = list(CONDITIONS)
    plan = [(c, ctl, pl, k, s) for c in a.categories
            for ctl in (["pid"] if c == 1 else a.controllers)
            for pl in (a.planners if c == 3 else [""])
            for k in a.conditions for s in range(a.first_seed, a.first_seed + a.seeds)]
    est = sum({1: 1.5, 2: 1.7, 3: 1.4}[c] for c, *_ in plan) / 60.0
    print(f"{len(plan)} runs -> {out}   (estimated duration ~{est:.1f} h)")
    if a.dry_run:
        for c, ctl, pl, k, s in plan:
            print(f"  category {c}  {label(c, ctl, pl):10s} {k:24s} seed {s}")
        return
    # The workspace must be sourced (otherwise every launch fails in 1 s)
    if subprocess.run(["ros2", "pkg", "prefix", "benchmark_bringup"], capture_output=True).returncode != 0:
        sys.exit("ERROR: package 'benchmark_bringup' not found. Source the workspace first:\n"
                 "    source ~/code/simulator_control/install/setup.bash")
    if psutil is None:
        print("WARNING: python3-psutil not installed: no CPU/RAM measurements (sudo apt install python3-psutil)")

    os.makedirs(out, exist_ok=True)
    man_path = os.path.join(out, "batch_manifest.json")
    manifest = json.load(open(man_path)) if os.path.exists(man_path) else {"batches": []}
    manifest["batches"].append({"environment": environment_info(), "categories": a.categories,
                                "conditions": {k: CONDITIONS[k] for k in a.conditions},
                                "controllers": a.controllers, "planners": a.planners,
                                "common_args": COMMON_ARGS, "extra_args": extra,
                                "seeds": [a.first_seed, a.first_seed + a.seeds - 1]})
    json.dump(manifest, open(man_path, "w"), indent=2)

    log_path = os.path.join(out, "batch_log.csv")
    new_log = not os.path.exists(log_path)
    logf = open(log_path, "a", newline="")
    logw = csv.writer(logf)
    if new_log:
        logw.writerow(["date", "category", "controller", "condition", "seed", "attempt", "batch_status",
                       "final_status", "rtf", "rtf_valid", "run_dir"])

    t_batch = time.time()
    for i, (c, ctl, pl, k, s) in enumerate(plan, 1):
        run_dir = os.path.join(out, f"category_{c}", run_name(c, ctl, pl, k, s))
        tag = f"c{c} {label(c, ctl, pl)} {k} s{s}"
        annotate_controller_rate(run_dir, os.path.join(out, "logs", f"c{c}_{os.path.basename(run_dir)}.log"))
        if run_is_valid(run_dir)[0]:
            print(f"[{i}/{len(plan)}] {tag}: already valid, skipped")
            continue
        for attempt in range(1, a.retries + 2):
            if os.path.isdir(run_dir):  # keep the invalid attempt (and its log) for inspection
                stamp = datetime.now().strftime('%H%M%S')
                os.rename(run_dir, f"{run_dir}_invalid_{stamp}")
                run_log = os.path.join(out, "logs", f"c{c}_{os.path.basename(run_dir)}.log")
                if os.path.isfile(run_log):
                    os.rename(run_log, run_log[:-4] + f"_invalid_{stamp}.log")
            print(f"[{i}/{len(plan)}] {tag} (attempt {attempt}) ...", flush=True)
            run_dir, status = launch_run(out, c, ctl, pl, k, s, extra)
            ok, m = run_is_valid(run_dir)
            q = (m or {}).get("run_quality", {})
            fs = json.load(open(os.path.join(run_dir, "meta.json"))).get("final_status", "") \
                if os.path.exists(os.path.join(run_dir, "meta.json")) else ""
            logw.writerow([datetime.now().isoformat(timespec="seconds"), c, label(c, ctl, pl), k, s, attempt,
                           status, fs,
                           q.get("real_time_factor", ""), ok, run_dir])
            logf.flush()
            print(f"      -> {status}, {fs or 'no results'}, RTF={q.get('real_time_factor', float('nan')):.3f}, "
                  f"missed cycles={100 * q.get('controller_missed_fraction', 0.0):.1f}%, "
                  f"{'VALID' if ok else 'INVALID'}")
            if ok:
                break
            time.sleep(5)
        time.sleep(3)  # let DDS clean up before the next launch
    logf.close()
    print(f"Batch finished in {(time.time() - t_batch) / 60:.1f} min. Next: python3 analyze_results.py {out}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nBatch interrupted: run the same command again to continue.")
        sys.exit(1)
