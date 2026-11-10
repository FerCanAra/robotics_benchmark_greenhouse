#!/usr/bin/env python3
"""
Offline analysis of a batch of benchmark runs (see run_batch.py).

    python3 analyze_results.py ~/benchmark_results/v1.1.0 [--horizon 1.0] [--baseline nominal]

Reads every <out>/category_<N>/<run>/ folder (invalid runs are ignored),
recomputes the metrics that depend on analysis choices from the saved CSVs, and
writes into <out>/analysis/:
    runs.csv                one row per valid run with all its metrics
    summary.csv             mean, std and n per category, controller and condition
    comparison_vs_<baseline>.csv  differences w.r.t. the baseline (controller,
                            condition): independent samples, Mann-Whitney U test
                            (if scipy exists)
    resources_summary.csv   mean CPU and max RAM per process (R3.13); --resources DIR adds runs
                            of other folders to this table only (their metrics are not used)
    appendix_tables.tex     full LaTeX tables (mean +- std, all configurations) for the appendix
    appendix_paired.tex     comparisons w.r.t. the baseline (differences and Mann-Whitney p)
    main_tables.tex         compact tables for the manuscript (Cat. 1 all conditions,
                            Cat. 2 and 3 nominal condition)
    resources_table.tex     CPU / RAM of the main processes (R3.13)
    figures/heatmap_cat2.(pdf|png), figures/heatmap_cat3.(pdf|png)
                            relative index of every controller/planner in every condition

Metrics added here:
    SAE2     category 2: distance from the robot position to the common reference
             path (start pose + waypoints, sent to every controller through
             FollowPath) / d_ref, always recomputed from the CSVs. SAE2_pred keeps
             the prediction error of the local planner (diagnostic only).
             Category 3: prediction error of TEB (distance from p(t+h) to the TEB
             local plan published at t / d_ref), recomputed only with --horizon.
    J*_rel   indices relative to the baseline (controller, condition), by default
             (teb, nominal) in category 2 and (teb-theta, nominal) in category 3:
             J1_rel = (SAE1/SAE1_ref + SCI1/SCI1_ref)/2,
             J2_rel = (SAE2/SAE2_ref + SCI2/SCI2_ref)/2,
             J3_rel = SAE3/SAE3_ref,  JT_rel = mean of the available levels.
             1 = as the baseline, < 1 better.
Only numpy is required (scipy optional, for the p-values).
"""

import argparse
import csv
import glob
import json
import math
import os
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "benchmark_bringup"))  # benchmark_bringup python package
from benchmark_bringup.metrics import dist_points_polyline, resample, resample_hold, time_average, time_grid  # noqa: E402

try:
    from scipy.stats import mannwhitneyu
except ImportError:
    mannwhitneyu = None

METRICS = {
    1: ["SAE1", "SCI1", "J1"],
    2: ["SAE1", "SCI1", "J1", "SAE2", "SAE2_pred", "SCI2", "J2", "path_length", "min_obstacle_dist",
        "contacts", "success", "time_to_goal"],
    3: ["SAE1", "SCI1", "J1", "SAE2", "SCI2", "J2", "SAE3", "SAE3_cur", "planned_path_length", "length_ratio",
        "min_obstacle_dist", "contacts", "success", "time_to_goal"],
}
REL = ["J1_rel", "J2_rel", "J3_rel", "JT_rel"]


def read_csv(path):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return list(csv.DictReader(f))


def controller_label(run_dir, meta):
    """pid (category 1), <controller> (category 2), <controller>-<planner> (category 3)."""
    info = os.path.join(run_dir, "batch_info.json")
    b = json.load(open(info)) if os.path.exists(info) else {}
    cat = int(meta.get("category", 0))
    if cat == 1:
        return "pid"
    ctl = b.get("controller") or meta.get("controller") or "teb"
    if cat == 3:
        return f"{ctl}-{b.get('planner') or meta.get('planner') or 'theta'}"
    return ctl


def condition_label(run_dir, meta):
    info = os.path.join(run_dir, "batch_info.json")
    if os.path.exists(info):
        return json.load(open(info))["condition"]
    lab = []
    if meta.get("terrain_slope"):
        lab.append("slope")
    if meta.get("change_terrain"):
        lab.append("terrain")
    if meta.get("payload_kg", 0):
        lab.append(f"payload{int(meta['payload_kg'])}")
    return "_".join(lab) or "nominal"


def sae2_route(run_dir, meta, d_ref, fs):
    """Category 2: distance from the robot position to the common reference path
    (start pose + waypoints of the STARTED event), normalised by d_ref."""
    gt = read_csv(os.path.join(run_dir, "pose_ground_truth.csv"))
    ev_path = os.path.join(run_dir, "events.json")
    if not gt or meta.get("t0_wall") is None or not os.path.exists(ev_path):
        return float("nan")
    route = next((e.get("route") for e in json.load(open(ev_path)) if e.get("state") == "STARTED"), None)
    if not route:
        return float("nan")
    t = np.array([float(r["t_wall"]) for r in gt])
    x = np.array([float(r["x"]) for r in gt])
    y = np.array([float(r["y"]) for r in gt])
    t0 = meta["t0_wall"]
    t1 = min(meta.get("t_end_wall") or t[-1], t[-1])
    i = min(int(np.searchsorted(t, t0)), len(t) - 1)
    ref = np.vstack([[x[i], y[i]], np.asarray([p[:2] for p in route], float)])
    tg = time_grid(t0, t1, fs)
    if tg.size < 2:
        return float("nan")
    return time_average(tg, dist_points_polyline(resample(t, x, tg), resample(t, y, tg), ref) / d_ref)


def cat3_plan_errors(run_dir, meta, d_ref, fs):
    """Category 3: (SAE2, SAE3) = time averages of the distance from the robot
    position to the global plan valid at each instant (mid level: the reference of
    the controller) and to the first plan of the task (high level), / d_ref."""
    gt = read_csv(os.path.join(run_dir, "pose_ground_truth.csv"))
    pl = read_csv(os.path.join(run_dir, "plans.csv"))
    nan = (float("nan"), float("nan"))
    if not gt or not pl or meta.get("t0_wall") is None:
        return nan
    plans = {}
    for r in pl:
        plans.setdefault(float(r["t_wall"]), []).append((float(r["x"]), float(r["y"])))
    plans = sorted((t, np.array(v)) for t, v in plans.items() if len(v) >= 2)
    if not plans:
        return nan
    t = np.array([float(r["t_wall"]) for r in gt])
    x = np.array([float(r["x"]) for r in gt])
    y = np.array([float(r["y"]) for r in gt])
    t0 = meta["t0_wall"]
    t1 = min(meta.get("t_end_wall") or t[-1], t[-1])
    tg = time_grid(t0, t1, fs)
    if tg.size < 2:
        return nan
    xg, yg = resample(t, x, tg), resample(t, y, tg)
    t_pl = np.array([p[0] for p in plans])
    idx = np.clip(np.searchsorted(t_pl, tg, side="right") - 1, 0, None)
    e2 = np.full(tg.size, np.nan)
    for k in np.unique(idx):
        sel = idx == k
        e2[sel] = dist_points_polyline(xg[sel], yg[sel], plans[k][1])
    first = next((p[1] for p in plans if p[0] >= t0 - 0.5), plans[0][1])  # as plotter_c3.first_plan
    e3 = dist_points_polyline(xg, yg, first)
    return time_average(tg, e2 / d_ref), time_average(tg, e3 / d_ref)


def sae2_horizon(run_dir, meta, h, d_ref, fs):
    """Distance from p_robot(t+h) to the TEB local plan published at t."""
    gt = read_csv(os.path.join(run_dir, "pose_ground_truth.csv"))
    tp = read_csv(os.path.join(run_dir, "local_plans.csv")) or read_csv(os.path.join(run_dir, "teb_local_plans.csv"))
    if not gt or not tp or meta.get("t0_wall") is None:
        return float("nan")
    t = np.array([float(r["t_wall"]) for r in gt])
    x = np.array([float(r["x"]) for r in gt])
    y = np.array([float(r["y"]) for r in gt])
    plans = {}
    for r in tp:
        plans.setdefault(int(r["plan_id"]), [float(r["t_wall"]), []])[1].append((float(r["x"]), float(r["y"])))
    plans = sorted((v[0], np.asarray(v[1])) for v in plans.values())
    t_pl = np.array([p[0] for p in plans])
    t0, t1 = meta["t0_wall"], meta.get("t_end_wall") or t[-1]
    tg = time_grid(t0, t1 - h, fs)
    if tg.size < 2:
        return float("nan")
    xr, yr = resample(t, x, tg + h), resample(t, y, tg + h)
    idx = np.clip(np.searchsorted(t_pl, tg, side="right") - 1, 0, None)
    e = np.empty(tg.size)
    for k in np.unique(idx):
        sel = idx == k
        e[sel] = dist_points_polyline(xr[sel], yr[sel], plans[k][1])
    return time_average(tg, e / d_ref)


def sci2_recompute(run_dir, meta, v_max, w_max, fs):
    """SCI2 with the given limits (if the run was recorded with other w_max/v_max)."""
    cmd = read_csv(os.path.join(run_dir, "cmd_vel_nav.csv"))
    if not cmd or meta.get("t0_wall") is None:
        return float("nan")
    k = list(cmd[0].keys())
    t = np.array([float(r[k[0]]) for r in cmd])
    v = np.array([float(r[k[1]]) for r in cmd])
    w = np.array([float(r[k[2]]) for r in cmd])
    tg = time_grid(meta["t0_wall"], meta.get("t_end_wall") or t[-1], fs)
    if tg.size < 2:
        return float("nan")
    u = np.sqrt((resample_hold(t, v, tg) / v_max) ** 2 + (resample_hold(t, w, tg) / w_max) ** 2) / math.sqrt(2.0)
    return time_average(tg, u)


def mean_std(v):
    v = np.asarray([x for x in v if x is not None and not (isinstance(x, float) and math.isnan(x))], float)
    if v.size == 0:
        return float("nan"), float("nan"), 0
    return float(v.mean()), float(v.std(ddof=1)) if v.size > 1 else 0.0, int(v.size)


# ---------------------------------------------------------------------------
#                 Outputs for the manuscript (compact) and appendix
# ---------------------------------------------------------------------------
COND_ORDER = ["nominal", "payload70", "terrain", "terrain_payload70", "slope", "slope_payload70",
              "slope_terrain", "slope_terrain_payload70"]
COND_LABEL = {"nominal": "Nominal", "payload70": "+70 kg", "terrain": "Terrain",
              "terrain_payload70": "Terrain +70 kg", "slope": "Slope", "slope_payload70": "Slope +70 kg",
              "slope_terrain": "Slope + terrain", "slope_terrain_payload70": "Slope + terrain +70 kg"}
NAME = {"pid": "PID", "teb": "TEB", "mppi": "MPPI", "rpp": "RPP", "dwb": "DWB",
        "theta": "Theta*", "smac": "Smac 2D", "navfn": "NavFn"}
CTRL_ORDER = ["teb", "mppi", "rpp", "dwb"]
PLAN_ORDER = ["theta", "smac", "navfn"]
TEX_METRIC = {"SAE2_pred": "SAE$_2^{pred}$", "SAE3_cur": "SAE$_3^{cur}$", "planned_path_length": "$L_{plan}$ [m]", "SAE1": "SAE$_1$", "SCI1": "SCI$_1$", "J1_rel": "$J_1^{rel}$", "SAE2": "SAE$_2$",
              "SCI2": "SCI$_2$", "J2_rel": "$J_2^{rel}$", "SAE3": "SAE$_3$", "J3_rel": "$J_3^{rel}$",
              "JT_rel": "$J_T^{rel}$", "success": "Success [\\%]", "time_to_goal": "$t_{goal}$ [s]",
              "min_obstacle_dist": "$d_{obs,min}$ [m]", "contacts": "Contacts", "length_ratio": "$L/L_{plan}$",
              "path_length": "$L$ [m]", "J1": "$J_1$", "J2": "$J_2$"}


def cond_label(k):
    return COND_LABEL.get(k, k.replace("_", " "))


def pretty(lab):
    """teb -> TEB, teb-smac -> Smac 2D (category 3 rows are planners)."""
    if "-" in lab:
        return NAME.get(lab.split("-", 1)[1], lab)
    return NAME.get(lab, lab.upper())


def cell(s, q, digits=3, pct=False):
    mu, sd = s.get(f"{q}_mean", float("nan")), s.get(f"{q}_std", float("nan"))
    if mu is None or math.isnan(mu):
        return "--"
    if pct:
        return f"{100 * mu:.0f}"
    return f"${mu:.{digits}f} \\pm {sd:.{digits}f}$"


def latex_longtable(f, caption, label, header, rows):
    """Long table that breaks across pages (appendix; needs \\usepackage{longtable})."""
    f.write("{\\footnotesize\\setlength{\\tabcolsep}{3pt}\n\\begin{longtable}{l" + "c" * (len(header) - 1) + "}\n")
    f.write(f"\\caption{{{caption}}}\\label{{{label}}}\\\\\n\\hline\n")
    f.write(" & ".join(header) + " \\\\\n\\hline\n\\endfirsthead\n\\hline\n")
    f.write(" & ".join(header) + " \\\\\n\\hline\n\\endhead\n\\hline\n\\endfoot\n")
    for r in rows:
        f.write(" & ".join(r) + " \\\\\n")
    f.write("\\end{longtable}\n}\n\n")


def latex_table(f, caption, label, header, rows):
    f.write("\\begin{table}[htbp]\n\\centering\n\\small\n")
    f.write(f"\\caption{{{caption}}}\n\\label{{{label}}}\n")
    # shrink only if wider than the column (needs graphicx)
    f.write("\\resizebox{\\ifdim\\width>\\linewidth\\linewidth\\else\\width\\fi}{!}{%\n")
    f.write("\\begin{tabular}{l" + "c" * (len(header) - 1) + "}\n\\hline\n")
    f.write(" & ".join(header) + " \\\\\n\\hline\n")
    for r in rows:
        f.write(" & ".join(r) + " \\\\\n")
    f.write("\\hline\n\\end{tabular}}\n\\end{table}\n\n")


def heatmap(path_base, row_labels, col_labels, values, title, cbar_label, notes=None):
    """Relative index (1 = baseline) on a diverging blue (better) - gray - red (worse)
    scale, symmetric in log(ratio); every cell is labelled with its value."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.colors import LinearSegmentedColormap, LogNorm
    except ImportError:
        print("(matplotlib not found: no heatmaps)")
        return
    v = np.asarray(values, float)
    finite = v[np.isfinite(v) & (v > 0)]
    if finite.size == 0:
        return
    m = max(np.max(np.abs(np.log(finite))), np.log(1.1))
    norm = LogNorm(vmin=math.exp(-m), vmax=math.exp(m))
    cmap = LinearSegmentedColormap.from_list("div", ["#2a78d6", "#f0efec", "#e34948"])
    cmap.set_bad("#ffffff")
    fig, ax = plt.subplots(figsize=(1.15 * len(col_labels) + 2.2,
                                    (0.75 if notes is not None else 0.55) * len(row_labels) + 1.6))
    im = ax.imshow(np.ma.masked_invalid(v), cmap=cmap, norm=norm, aspect="auto")
    ax.set_xticks(range(len(col_labels)), col_labels, rotation=30, ha="right", rotation_mode="anchor", fontsize=9)
    ax.set_yticks(range(len(row_labels)), row_labels, fontsize=10)
    ax.tick_params(length=0)
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.set_xticks(np.arange(-0.5, len(col_labels)), minor=True)
    ax.set_yticks(np.arange(-0.5, len(row_labels)), minor=True)
    ax.grid(which="minor", color="#fcfcfb", linewidth=2)  # 2 px gap between cells
    ax.tick_params(which="minor", length=0)
    for i in range(v.shape[0]):
        for j in range(v.shape[1]):
            if np.isfinite(v[i, j]):
                if notes is not None and notes[i][j]:
                    ax.text(j, i - 0.13, f"{v[i, j]:.2f}", ha="center", va="center", fontsize=9, color="#1a1a19")
                    ax.text(j, i + 0.22, notes[i][j], ha="center", va="center", fontsize=6.5, color="#3d3d3a")
                else:
                    ax.text(j, i, f"{v[i, j]:.2f}", ha="center", va="center", fontsize=9, color="#1a1a19")
    cb = fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02)
    cb.set_label(cbar_label, fontsize=9)
    cb.ax.tick_params(labelsize=8)
    cb.ax.minorticks_off()
    ticks = [t for t in (0.5, 0.75, 1.0, 1.25, 1.5, 2.0) if math.exp(-m) <= t <= math.exp(m)]
    cb.set_ticks(ticks, labels=[f"{t:g}" for t in ticks])
    ax.set_title(title, fontsize=11)
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(f"{path_base}.{ext}", dpi=200)
    plt.close(fig)


APPENDIX_GROUPS = {
    1: [["SAE1", "SCI1", "J1", "J1_rel"]],
    2: [["SAE1", "SCI1", "SAE2", "SCI2", "J2", "J2_rel", "JT_rel"],
        ["success", "time_to_goal", "path_length", "min_obstacle_dist", "contacts"]],
    3: [["SAE1", "SCI1", "SAE2", "SCI2", "SAE3", "J3_rel", "JT_rel"],
        ["success", "time_to_goal", "planned_path_length", "length_ratio", "min_obstacle_dist", "contacts"]],
}


def write_appendix_tables(out, summary):
    """All the configurations (mean +- std), split in two tables per category so that
    they fit the page width (longtable: they break across pages)."""
    order = {k: i for i, k in enumerate(COND_ORDER)}
    corder = {k: i for i, k in enumerate(["pid"] + CTRL_ORDER)}
    with open(os.path.join(out, "appendix_tables.tex"), "w") as f:
        f.write("% Full results: mean $\\pm$ std over the valid runs.\n"
                "% Requires the longtable and pdflscape packages (tables in landscape pages).\n"
                "\\begin{landscape}\n")
        for c in sorted({s["category"] for s in summary}):
            rows_c = sorted([s for s in summary if s["category"] == c],
                            key=lambda s: (corder.get(s["controller"].split("-")[0], 99), s["controller"],
                                           order.get(s["condition"], 99)))
            groups = APPENDIX_GROUPS.get(c, [METRICS[c] + REL])
            for g, cols in enumerate(groups, 1):
                cols = [q for q in cols if any(not math.isnan(s.get(f"{q}_mean", float("nan"))) for s in rows_c)]
                if not cols:
                    continue
                rows = []
                for s in rows_c:
                    rows.append([pretty(s["controller"]), cond_label(s["condition"]), str(s["n"])]
                                + [cell(s, q, 4 if q.startswith(("SAE", "SCI")) else 2, pct=(q == "success"))
                                   for q in cols])
                part = f" ({'indices' if g == 1 else 'path quality'})" if len(groups) > 1 else ""
                latex_longtable(f, f"Category {c}: full results{part}.", f"tab:app_cat{c}_{g}",
                                ["Config.", "Condition", "$n$"] + [TEX_METRIC.get(q, q.replace("_", "\\_")) for q in cols],
                                rows)
        f.write("\\end{landscape}\n")


def write_paper_outputs(out, runs, summary, paired, res, a):
    S = {(s["category"], s["controller"], s["condition"]): s for s in summary}
    conds = [k for k in COND_ORDER if any(s["condition"] == k for s in summary)]
    conds += sorted({s["condition"] for s in summary} - set(conds))
    figdir = os.path.join(out, "figures")
    os.makedirs(figdir, exist_ok=True)
    base3 = f"{a.baseline_controller}-"

    with open(os.path.join(out, "main_tables.tex"), "w") as f:
        f.write("% Compact tables for the manuscript (mean $\\pm$ std over the seeds).\n"
                "% J^rel = 1: as the baseline (nominal); < 1 better. Success in %.\n\n")
        # Category 1: every condition
        rows = [[cond_label(k), str(S[(1, "pid", k)]["n"]), cell(S[(1, "pid", k)], "SAE1", 4),
                 cell(S[(1, "pid", k)], "SCI1", 4), cell(S[(1, "pid", k)], "J1_rel", 2)]
                for k in conds if (1, "pid", k) in S]
        if rows:
            latex_table(f, "Category 1 (low level, PID): results in every condition.", "tab:res_cat1",
                        ["Condition", "$n$", TEX_METRIC["SAE1"], TEX_METRIC["SCI1"], TEX_METRIC["J1_rel"]], rows)
        # Category 2: nominal, one row per controller
        q2 = ["SAE2", "SCI2", "J2_rel", "success", "time_to_goal", "min_obstacle_dist", "contacts"]
        ctrls = [c for c in CTRL_ORDER if (2, c, a.baseline) in S] + \
                sorted({s["controller"] for s in summary if s["category"] == 2} - set(CTRL_ORDER))
        rows = []
        for c in ctrls:
            s = S.get((2, c, a.baseline))
            if s:
                rows.append([pretty(c), str(s["n"])] + [cell(s, q, 2 if q in ("J2_rel", "time_to_goal", "min_obstacle_dist", "contacts") else 3,
                                                          pct=(q == "success")) for q in q2])
        if rows:
            latex_table(f, f"Category 2 (mid level): controllers in the {cond_label(a.baseline).lower()} condition.",
                        "tab:res_cat2", ["Controller", "$n$"] + [TEX_METRIC[q] for q in q2], rows)
        # Category 3: nominal, one row per planner
        q3 = ["SAE3", "length_ratio", "J3_rel", "success", "time_to_goal", "min_obstacle_dist", "contacts"]
        labs3 = [base3 + p for p in PLAN_ORDER] + sorted({s["controller"] for s in summary if s["category"] == 3}
                                                          - {base3 + p for p in PLAN_ORDER})
        rows = []
        for lab in labs3:
            s = S.get((3, lab, a.baseline))
            if s:
                rows.append([pretty(lab), str(s["n"])] + [cell(s, q, 2 if q in ("J3_rel", "time_to_goal", "min_obstacle_dist", "contacts") else 3,
                                                            pct=(q == "success")) for q in q3])
        if rows:
            latex_table(f, f"Category 3 (high level, {NAME.get(a.baseline_controller, a.baseline_controller.upper())} "
                        f"as mid-level controller): planners in the {cond_label(a.baseline).lower()} condition.",
                        "tab:res_cat3", ["Planner", "$n$"] + [TEX_METRIC[q] for q in q3], rows)

    # Heatmaps of the relative index in every condition
    for c, q, labs, what in ((2, "J2_rel", ctrls, "controller"), (3, "J3_rel", labs3, "planner")):
        labs = [l for l in labs if any((c, l, k) in S for k in conds)]
        if not labs:
            continue
        vals = [[S.get((c, l, k), {}).get(f"{q}_mean", float("nan")) for k in conds] for l in labs]

        # Success and time to goal under every index: a controller that slows down or
        # stalls gets small prediction errors and small effort (low J), so the index
        # alone can look "better" while the task takes much longer or fails.
        def note(st):
            if not st:
                return ""
            n = int(st.get("n", 0))
            ns = int(round(st.get("success_mean", float("nan")) * n)) if n else 0
            t = st.get("time_to_goal_mean", float("nan"))
            return f"{ns}/{n} ok" + (f" · {t:.0f} s" if np.isfinite(t) else "")
        notes = [[note(S.get((c, l, k))) for k in conds] for l in labs]
        heatmap(os.path.join(figdir, f"heatmap_cat{c}"), [pretty(l) for l in labs], [cond_label(k) for k in conds],
                vals, f"Category {c}: {'$J_2^{rel}$' if c == 2 else '$J_3^{rel}$'} per {what} and condition "
                      f"(below: successful runs and mean time to goal)",
                "relative index (1 = baseline, < 1 better)", notes=notes)

    # Appendix: paired comparisons of the main indices
    keyq = ["SAE1", "SCI1", "SAE2", "SCI2", "SAE3", "J1_rel", "J2_rel", "J3_rel", "JT_rel", "success"]
    with open(os.path.join(out, "appendix_paired.tex"), "w") as f:
        f.write("% Differences w.r.t. the baseline of each category (independent samples, Mann-Whitney U): PID nominal (Cat. 1), "
                f"{a.baseline_controller.upper()} nominal (Cat. 2), {a.baseline_controller.upper()}+{a.baseline_planner} nominal (Cat. 3). "
                "Requires the longtable package.\n")
        for c in sorted({p["category"] for p in paired}):
            rows = []
            order = {k: i for i, k in enumerate(COND_ORDER)}
            for p in sorted(paired, key=lambda p: (p["controller"], order.get(p["condition"], 99),
                                                   keyq.index(p["metric"]) if p["metric"] in keyq else 99)):
                if p["category"] != c or p["metric"] not in keyq or (c == 1 and p["metric"] == "JT_rel"):
                    continue
                pv = p["p"]
                ptxt = "--" if math.isnan(pv) else (f"{pv:.3f}" + ("$^{*}$" if pv < 0.05 else ""))
                rows.append([pretty(p["controller"]), cond_label(p["condition"]), TEX_METRIC.get(p["metric"], p["metric"]),
                             str(p["n"]), f"${p['mean_diff']:+.4f} \\pm {p['se_diff']:.4f}$", ptxt])
            if rows:
                latex_longtable(f, f"Category {c}: differences w.r.t. the baseline (difference of means $\\pm$ "
                            f"standard error; two-sided Mann--Whitney $U$ test, $^{{*}}$ $p<0.05$).", f"tab:paired_cat{c}",
                            ["Config.", "Condition", "Metric", "$n$", "$\\Delta$ (mean $\\pm$ s.e.)", "$p$"], rows)

    # Resources (R3.13): main processes
    main_proc = ("mvsim_node", "controller_server", "planner_server", "bt_navigator", "amcl",
                 "velocity_smoother", "behavior_server", "smoother_server", "map_server", "waypoint_follower")
    rows = []
    for (cdir, name), v in sorted(res.items()):
        if not any(name.startswith(m) for m in main_proc) and "plotter" not in name and "cat" not in name:
            continue
        mu, sd, n = mean_std(v["cpu"])
        rows.append([cdir.replace("category_", "Cat. "), name.replace("_", "\\_"), str(n),
                     f"${mu:.1f} \\pm {sd:.1f}$", f"{np.mean(v['rss']):.0f}"])
    if rows:
        with open(os.path.join(out, "resources_table.tex"), "w") as f:
            latex_table(f, "Computational resources per process (mean CPU over each run, mean of the "
                        "peak RAM; 100\\% = one core).", "tab:resources",
                        ["Category", "Process", "Runs", "CPU [\\%]", "RAM [MB]"], rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results")
    ap.add_argument("--horizon", type=float, default=None,
                    help="recompute SAE2 with another prediction horizon h [s] (default: as recorded, 1 s)")
    ap.add_argument("--baseline", default="nominal", help="baseline condition")
    ap.add_argument("--resources", nargs="*", default=[],
                    help="extra result folders used ONLY for the CPU/RAM table (e.g. ~/benchmark_results/recursos_cat1)")
    ap.add_argument("--baseline-controller", default="teb", help="baseline controller (category 2)")
    ap.add_argument("--baseline-planner", default="theta", help="baseline planner (category 3, with --baseline-controller)")
    a = ap.parse_args()
    root = os.path.abspath(os.path.expanduser(a.results))
    out = os.path.join(root, "analysis")
    os.makedirs(out, exist_ok=True)

    # ------------------------------------------------------------ load runs
    runs, n_invalid = [], 0
    for mpath in sorted(glob.glob(os.path.join(root, "category_*", "*", "metrics.json"))):
        d = os.path.dirname(mpath)
        if "_invalid_" in os.path.basename(d):
            continue
        m = json.load(open(mpath))
        meta = json.load(open(os.path.join(d, "meta.json")))
        q = m.get("run_quality", {})
        if (not q.get("rtf_valid", False) or not q.get("controller_rate_valid", True)
                or not q.get("sim_rate_valid", True)):
            n_invalid += 1
            continue
        cfg = meta.get("metrics_config", {})
        r = {"category": int(meta["category"]), "controller": controller_label(d, meta),
             "condition": condition_label(d, meta),
             "seed": meta.get("seed", -1), "run": os.path.basename(d), "final_status": meta.get("final_status", ""),
             "rtf": m["run_quality"].get("real_time_factor")}
        r.update({k: v for k, v in m.items() if not isinstance(v, dict)})
        # Normalisation limits actually used by the controllers (launch arguments of the run)
        info = os.path.join(d, "batch_info.json")
        la = json.load(open(info)).get("launch_args", {}) if os.path.exists(info) else {}
        w_run = float(la.get("w_max", cfg.get("w_max", 1.0)))
        if r["category"] in (2, 3) and abs(w_run - cfg.get("w_max", w_run)) > 1e-9:
            r["SCI2"] = sci2_recompute(d, meta, cfg.get("v_max", 0.5), w_run, cfg.get("fs", 10.0))
            r["J2"] = 0.5 * (r.get("SAE2", float("nan")) + r["SCI2"])
        if r["category"] == 2:
            # SAE2 = distance to the common reference path (recomputed so that runs
            # recorded with the former definition are analysed in the same way);
            # the prediction error of the local planner is kept as SAE2_pred.
            if "SAE2_pred" not in r:
                r["SAE2_pred"] = r.get("SAE2", float("nan"))
            r["SAE2"] = sae2_route(d, meta, cfg.get("d_ref", 2.0), cfg.get("fs", 10.0))
            r["J2"] = 0.5 * (r["SAE2"] + r.get("SCI2", float("nan")))
        if r["category"] == 3:
            # SAE2 = distance to the current plan, SAE3 = distance to the first plan
            # (recomputed so that runs recorded with the former definitions are analysed
            # in the same way); the former values are kept as SAE2_pred and SAE3_cur.
            if "SAE2_pred" not in r:
                r["SAE2_pred"] = r.get("SAE2", float("nan"))
            if "SAE3_cur" not in r:
                r["SAE3_cur"] = r.get("SAE3", float("nan"))
            r["SAE2"], r["SAE3"] = cat3_plan_errors(d, meta, cfg.get("d_ref", 2.0), cfg.get("fs", 10.0))
            r["J2"] = 0.5 * (r["SAE2"] + r.get("SCI2", float("nan")))
            r["J3"] = r["SAE3"]
            r["JT3"] = (r.get("J1", float("nan")) + r["J2"] + r["J3"]) / 3.0
        runs.append(r)
    print(f"{len(runs)} valid runs ({n_invalid} invalid ignored)")
    if not runs:
        return

    # ---------------------------------------------- relative indices (baseline)
    groups = defaultdict(list)
    for r in runs:
        groups[(r["category"], r["controller"], r["condition"])].append(r)

    def base_key(c):
        lab = ("pid" if c == 1 else a.baseline_controller if c == 2
               else f"{a.baseline_controller}-{a.baseline_planner}")
        return (c, lab, a.baseline)

    ref = {}
    for c in {r["category"] for r in runs}:
        if base_key(c) in groups:
            rs = groups[base_key(c)]
            ref[c] = {q: mean_std([r.get(q) for r in rs])[0] for q in ("SAE1", "SCI1", "SAE2", "SCI2", "SAE3")}
    for r in runs:
        b = ref.get(r["category"])
        if not b:
            continue
        rel = lambda q: r.get(q, float("nan")) / b[q] if b.get(q) else float("nan")  # noqa: E731
        r["J1_rel"] = 0.5 * (rel("SAE1") + rel("SCI1"))
        if r["category"] >= 2:
            r["J2_rel"] = 0.5 * (rel("SAE2") + rel("SCI2"))
        if r["category"] == 3:
            r["J3_rel"] = rel("SAE3")
        levels = [r[q] for q in ("J1_rel", "J2_rel", "J3_rel") if q in r and not math.isnan(r[q])]
        r["JT_rel"] = float(np.mean(levels)) if levels else float("nan")

    # ------------------------------------------------------------- runs.csv
    keys = ["category", "controller", "condition", "seed", "run", "final_status", "rtf"]
    extra = sorted({k for r in runs for k in r} - set(keys))
    with open(os.path.join(out, "runs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys + extra)
        w.writeheader()
        for r in sorted(runs, key=lambda r: (r["category"], r["controller"], r["condition"], r["seed"])):
            w.writerow(r)

    # ---------------------------------------------------------- summary.csv
    summary = []
    for (c, ctl, k), rs in sorted(groups.items()):
        row = {"category": c, "controller": ctl, "condition": k, "n": len(rs)}
        for q in METRICS[c] + REL:
            mu, sd, n = mean_std([r.get(q) for r in rs])
            row[f"{q}_mean"], row[f"{q}_std"] = mu, sd
        summary.append(row)
    first = ["category", "controller", "condition", "n"]
    fields = first + sorted({k for r in summary for k in r} - set(first))
    with open(os.path.join(out, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(summary)

    # ------------------------------------------- comparison vs baseline
    # The plant arrangement is drawn at random in every run (it does not depend on
    # the seed), so the runs of two configurations are independent samples: the
    # difference of means, its standard error and a two-sided Mann-Whitney U test.
    paired = []
    for (c, ctl, k), rs in sorted(groups.items()):
        if (c, ctl, k) == base_key(c) or base_key(c) not in groups:
            continue
        base = groups[base_key(c)]
        for q in METRICS[c] + REL:
            x = np.array([float(r[q]) for r in rs if r.get(q) is not None and not math.isnan(float(r[q]))])
            y = np.array([float(r[q]) for r in base if r.get(q) is not None and not math.isnan(float(r[q]))])
            if len(x) < 2 or len(y) < 2:
                continue
            p = float("nan")
            if mannwhitneyu is not None and not (np.all(x == x[0]) and np.all(y == x[0])):
                try:
                    p = float(mannwhitneyu(x, y, alternative="two-sided").pvalue)
                except ValueError:
                    pass
            paired.append({"category": c, "controller": ctl, "condition": k, "metric": q,
                           "n": len(x), "n_baseline": len(y),
                           "mean_diff": float(x.mean() - y.mean()),
                           "se_diff": float(math.sqrt(x.var(ddof=1) / len(x) + y.var(ddof=1) / len(y))),
                           "p": p})
    with open(os.path.join(out, f"comparison_vs_{a.baseline_controller}_{a.baseline}.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["category", "controller", "condition", "metric", "n", "n_baseline",
                                          "mean_diff", "se_diff", "p"])
        w.writeheader()
        w.writerows(paired)

    # ------------------------------------------------------------ resources
    res = defaultdict(lambda: {"cpu": [], "rss": []})
    # Resources of the main folder plus the extra folders given with --resources.
    # Files whose CPU column is all zero (CPU sampling bug of run_batch.py before v3) are ignored
    # for the CPU; their RAM is still used.
    files = []
    for rd in [root] + [os.path.abspath(os.path.expanduser(x)) for x in a.resources]:
        files += glob.glob(os.path.join(rd, "category_*", "*", "resources.csv"))
    for p in files:
        if "_invalid_" in p:
            continue
        c = os.path.basename(os.path.dirname(os.path.dirname(p)))
        per_run = defaultdict(lambda: {"cpu": [], "rss": []})
        rows_p = read_csv(p) or []
        # old files: only the launch process (first pid) had CPU, every child read 0
        root_pid = rows_p[0]["pid"] if rows_p else None
        cpu_ok = any(float(r["cpu_percent"]) > 0 for r in rows_p if r["pid"] != root_pid)
        for r in rows_p:
            per_run[r["name"]]["cpu"].append(float(r["cpu_percent"]))
            per_run[r["name"]]["rss"].append(float(r["rss_mb"]))
        for name, v in per_run.items():
            if cpu_ok:
                res[(c, name)]["cpu"].append(np.mean(v["cpu"]))
            res[(c, name)]["rss"].append(np.max(v["rss"]))
    with open(os.path.join(out, "resources_summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["category", "process", "n_runs", "cpu_mean_percent", "cpu_std", "rss_max_mb_mean"])
        for (c, name), v in sorted(res.items()):
            mu, sd, n = mean_std(v["cpu"])
            w.writerow([c, name, n, f"{mu:.1f}", f"{sd:.1f}", f"{np.mean(v['rss']):.0f}"])

    # ------------------------------------------------------------ LaTeX
    write_appendix_tables(out, summary)
    write_paper_outputs(out, runs, summary, paired, res, a)
    print(f"Analysis written to {out}")
    if mannwhitneyu is None:
        print("(scipy not found: no p-values; sudo apt install python3-scipy)")


if __name__ == "__main__":
    main()
