#!/usr/bin/env python3
"""Turn recorded /terrain_map clouds into a per-axis density report.

For each variant's recorded bag this reads every /terrain_map message, counts points
(point count == width*height; the cloud is unorganized so height==1), drops a warmup
window, and reports per-message statistics over the steady-state window.

Why per-message median (not a cumulative total): /terrain_map is republished once per
laser frame over a fixed ~11x11 m vehicle-centered window, so a total just scales with
message count. The median point count per frame over a fixed time window is the density
signal. The replay is identical across variants (same bag, same rate, same poses), so the
same time window is a PAIRED comparison and Δ-vs-baseline cancels the scene/ego-motion
confound. Lower median = sparser / less noisy map.

Caveat by axis (printed in the report): for minRelZ / maxRelZ / minBlockPointNum the count
change is partly *mechanical* (they are inclusion predicates), so a drop is expected -- the
real question is whether genuine obstacles survive, which you confirm visually by replaying
the variant's bag into RViz. scanVoxelSize changes true cloud density; quantileZ changes the
ground estimate.
"""
import argparse
import json
import os
import statistics
import sys

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
except ImportError as e:
    sys.exit(f"ROS 2 python libs not found ({e}). Run inside the a2_ros_dev container.")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Analysis-time commentary per axis: how to read a count change.
AXIS_NOTE = {
    "scanVoxelSize":    "density (voxel leaf): fewer points = genuinely sparser/cleaner cloud.",
    "minBlockPointNum": "mechanical gate: higher rejects sparse cells (count drop expected); confirm obstacles persist.",
    "quantileZ":        "ground estimate: higher raises ground, suppresses low clutter (useSorting stays true).",
    "minRelZ":          "crop floor (mechanical): higher crops low points; too high erases the ground itself.",
    "maxRelZ":          "crop ceiling (mechanical): lower trims overhead clutter; too low clips body-height obstacles.",
    "disRatioZ":        "far-range crop: lower trims distant vertical spread admitted into the rolling map.",
}


def read_counts(bag_dir):
    """Return [(stamp_sec, point_count), ...] for /terrain_map in a recorded bag dir."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id="mcap"),
                ConverterOptions("cdr", "cdr"))
    out = []
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != "/terrain_map":
            continue
        msg = deserialize_message(data, PointCloud2)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        out.append((stamp, msg.width * msg.height))
    return out


def windowed_stats(counts, warmup):
    """Drop the first `warmup` seconds (sim time) and summarize the rest."""
    if not counts:
        return None
    t0 = min(s for s, _ in counts)
    window = [c for s, c in counts if s >= t0 + warmup]
    if not window:                      # warmup ate everything -> fall back to all frames
        window = [c for _, c in counts]
        truncated = True
    else:
        truncated = False
    window_sorted = sorted(window)
    n = len(window)
    return {
        "n_total": len(counts),
        "n_window": n,
        "median": statistics.median(window),
        "p25": window_sorted[int(0.25 * (n - 1))],
        "p75": window_sorted[int(0.75 * (n - 1))],
        "p95": window_sorted[int(0.95 * (n - 1))],
        "mean": sum(window) / n,
        "min": min(window),
        "max": max(window),
        "fallback_no_warmup": truncated,
    }


def fmt(v):
    if isinstance(v, float):
        return "%g" % v
    return str(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-dir", required=True, help="run dir holding manifest.json + per-variant bags")
    ap.add_argument("--warmup", type=float, default=13.0, help="seconds of warmup to drop (default 13)")
    args = ap.parse_args()

    manifest = json.load(open(os.path.join(args.run_dir, "manifest.json")))

    # Measure every variant bag once.
    stats = {}
    for ax in manifest["axes"]:
        for p in ax["points"]:
            v = p["variant"]
            if v in stats:
                continue
            bag = os.path.join(args.run_dir, v)
            if not os.path.isdir(bag):
                stats[v] = None
                continue
            try:
                stats[v] = windowed_stats(read_counts(bag), args.warmup)
            except Exception as e:                     # noqa: BLE001 - keep going on a bad bag
                print(f"  ! failed to read {v}: {e}", file=sys.stderr)
                stats[v] = None

    # ---- CSV ----
    csv_path = os.path.join(args.run_dir, "results.csv")
    with open(csv_path, "w") as f:
        f.write("axis,param,value,is_baseline,variant,n_window,median,p25,p75,p95,mean,min,max,delta_median_pct\n")
        for ax in manifest["axes"]:
            base_variant = next((p["variant"] for p in ax["points"]
                                 if p["variant"] == "baseline"), "baseline")
            base = stats.get(base_variant) or stats.get("baseline")
            base_med = base["median"] if base else None
            for p in ax["points"]:
                s = stats.get(p["variant"])
                is_base = "1" if p["variant"] == "baseline" else "0"
                if not s:
                    f.write(f"{ax['param']},{ax['param']},{fmt(p['value'])},{is_base},"
                            f"{p['variant']},0,,,,,,,,\n")
                    continue
                dpct = "" if (base_med in (None, 0)) else f"{(s['median']-base_med)/base_med*100:.1f}"
                f.write(f"{ax['param']},{ax['param']},{fmt(p['value'])},{is_base},{p['variant']},"
                        f"{s['n_window']},{s['median']:.0f},{s['p25']},{s['p75']},{s['p95']},"
                        f"{s['mean']:.0f},{s['min']},{s['max']},{dpct}\n")

    # ---- Markdown ----
    md = []
    md.append("# terrainAnalysis OFAT sweep results\n")
    md.append(f"- run dir: `{args.run_dir}`")
    md.append(f"- baseline: `{manifest['baseline_config']}`")
    md.append(f"- metric: per-frame point count of `/terrain_map`, steady-state window "
              f"(first **{args.warmup:g} s** dropped). Lower = sparser / less noisy.")
    md.append("- replay is identical across variants, so Δ-vs-baseline is a paired comparison.\n")

    base_global = stats.get("baseline")
    if base_global:
        md.append(f"**Baseline** steady-state median = **{base_global['median']:.0f}** pts/frame "
                  f"(IQR {base_global['p25']}–{base_global['p75']}, "
                  f"n={base_global['n_window']}).\n")

    for ax in manifest["axes"]:
        param = ax["param"]
        base = stats.get("baseline")
        base_med = base["median"] if base else None
        md.append(f"## `{param}`  (baseline {fmt(ax['baseline'])})")
        md.append(f"_{AXIS_NOTE.get(param, '')}_\n")
        md.append("| value | n | median pts | IQR (p25–p75) | p95 | Δ median vs base |")
        md.append("|---|---:|---:|---:|---:|---:|")
        for p in ax["points"]:
            s = stats.get(p["variant"])
            tag = " (baseline)" if p["variant"] == "baseline" else ""
            if not s:
                md.append(f"| {fmt(p['value'])}{tag} | 0 | — | — | — | _no output_ |")
                continue
            if base_med in (None, 0) or p["variant"] == "baseline":
                d = "—"
            else:
                d = f"{(s['median']-base_med)/base_med*100:+.1f}%"
            warn = " ⚠warmup" if s.get("fallback_no_warmup") else ""
            md.append(f"| {fmt(p['value'])}{tag} | {s['n_window']}{warn} | {s['median']:.0f} | "
                      f"{s['p25']}–{s['p75']} | {s['p95']} | {d} |")
        md.append("")

    md.append("## How to choose a value per axis\n")
    md.append("1. On each axis, a lower median means a sparser/cleaner map. Move along the axis "
              "until the median stops dropping meaningfully (diminishing returns) — that knee is "
              "the value worth keeping.")
    md.append("2. **Then verify visually**: replay that variant's bag into RViz and confirm real "
              "obstacles (walls, legs, low boxes) still appear — a low count can also mean you "
              "cropped/erased real terrain. `minRelZ`, `maxRelZ`, `minBlockPointNum` drop the count "
              "mechanically, so they need this check most.")
    md.append("3. Apply the chosen values together back into the `terrainAnalysis` block of "
              "`navigation_a2.yaml`, then run a combined confirmation pass.\n")
    md.append("> Variants with `_no output_` cropped the ground itself (e.g. `minRelZ` raised above "
              "the true ground level) — out of the safe range; ignore.")

    md_path = os.path.join(args.run_dir, "results.md")
    with open(md_path, "w") as f:
        f.write("\n".join(md) + "\n")

    # ---- console summary ----
    print(f"wrote {md_path}")
    print(f"wrote {csv_path}\n")
    for ax in manifest["axes"]:
        base = stats.get("baseline")
        base_med = base["median"] if base else None
        cells = []
        for p in ax["points"]:
            s = stats.get(p["variant"])
            if not s:
                cells.append(f"{fmt(p['value'])}=NA")
            else:
                cells.append(f"{fmt(p['value'])}={s['median']:.0f}")
        print(f"  {ax['param']:<16} median pts/frame: " + "  ".join(cells))


if __name__ == "__main__":
    main()
