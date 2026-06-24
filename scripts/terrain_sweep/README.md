# terrainAnalysis one-axis parameter sweep

Automatic **one-at-a-time (OFAT)** sweep of the `terrainAnalysis` node parameters that
control map noise/density, evaluated against a recorded bag. Each run varies **exactly one
parameter** away from the baseline (the `terrainAnalysis` block of
[`navigation_a2.yaml`](../../src/meta_packages/a2_ros/config/autonomy/navigation_a2.yaml)),
replays real LiDAR/odometry through the node, and measures the resulting `/terrain_map`
density so you can pick a value per axis.

The swept axes are exactly the *"Map is noisy or too dense"* recommendations from
[`tuning_guideline/CONFIG_TUNING_Terrain.md`](../../tuning_guideline/CONFIG_TUNING_Terrain.md):

| Axis | Direction | Grid (baseline **bold**) |
|---|---|---|
| `scanVoxelSize` | increase | **0.10**, 0.15, 0.20, 0.25, 0.30 |
| `minBlockPointNum` | increase | **20**, 30, 40, 50, 60 |
| `quantileZ` (useSorting=true) | 0.20–0.35 band | 0.20, **0.25**, 0.30, 0.35 |
| `minRelZ` | tighten up | **-1.0**, -0.8, -0.6, -0.4, -0.3 |
| `maxRelZ` | tighten down | **1.5**, 1.2, 1.0, 0.8, 0.6 |
| `disRatioZ` | lower | **0.20**, 0.15, 0.10, 0.05, 0.00 |

Edit `axes.yaml` to change the grids.

## Run it

Inside the `a2_ros_dev` container, with `terrain_analysis` built (`a2 build a2_robot`):

```bash
docker compose exec a2_ros_dev bash
scripts/terrain_sweep/run_sweep.sh
```

This generates variants, sweeps every one (~30 s–1 min each at rate 1.0), and writes a
report. Useful flags:

```bash
scripts/terrain_sweep/run_sweep.sh --rate 3            # faster playback (keep it constant!)
scripts/terrain_sweep/run_sweep.sh --bag bags/dataset_X
scripts/terrain_sweep/run_sweep.sh --gen-only          # just write variants/, don't run
scripts/terrain_sweep/run_sweep.sh --only scanVoxelSize__0.2   # one variant
```

Outputs land in `scripts/terrain_sweep/runs/<timestamp>/`:
- `results.md` / `results.csv` — per-axis density tables
- `<variant>/` — the recorded `/terrain_map` for that variant (replay into RViz to eyeball it)
- `<variant>.{node,play,rec}.log` — logs

You can also run the stages by hand: `gen_variants.py` → `run_sweep.sh --only …` →
`analyze.py --run-dir runs/<ts> --warmup 13`.

## How it works (and why each choice)

- **Full block per variant.** terrain_analysis is launched with the params file as its
  *only* source and declares every key with a baked-in C++ default, so an omitted key
  silently reverts to that default — and the C++ defaults differ from `navigation_a2.yaml`
  (e.g. `decayTime` 2.0 vs 10.0, `vehicleHeight` 1.5 vs 0.5). A single-key file would
  therefore change ~9 parameters at once. Each variant is a **full copy of the baseline
  block with one key changed** (same dead-key trap as
  [`docs/far_planner_tuning.md`](../../docs/far_planner_tuning.md) §1). The runner reads
  back a canary (`decayTime` must be `10.0`) to confirm the block loaded.
- **Isolated node.** Only `terrainAnalysis` is launched (not the full nav stack), so
  `/terrain_map` is unambiguously its output; `terrainAnalysisExt`/`localPlanner` never run.
- **Sim time.** The node runs with `use_sim_time:=true` and playback uses `--clock`; both
  are required or the time-based decay/voxel logic freezes.
- **Metric.** `/terrain_map` is an unorganized `PointXYZI` cloud, so point count =
  `width`. It is republished per laser frame over a fixed ~11×11 m vehicle-centered window.
  We drop a **warmup** (≥ `max(decayTime, 3·voxelTimeUpdateThre)` ≈ 13 s) and report the
  **per-frame median + IQR** over the steady-state window — not a cumulative total. Replay
  is identical across variants, so Δ-vs-baseline is a *paired* comparison.

### Why these ranges

Bounds were taken from `terrainAnalysis.cpp` and `vehicleHeight=0.5`:

- `scanVoxelSize` ≤ 0.30 m: the planar ground grid is fixed at 0.2 m; a leaf ≳0.35 m
  collapses each cell to 1–2 points and erases small obstacles.
- `minBlockPointNum` ≤ 60: above ~80 only the densest near cells publish, punching holes in
  far/thin terrain.
- `quantileZ` ≤ 0.35: at/above the median the ground latches onto obstacle returns and real
  obstacles vanish.
- `minRelZ` ≥ -0.3: ground sits ~-0.45 m below the frame; raising the floor above it crops
  the ground itself (those variants show `_no output_`).
- `maxRelZ` ≥ 0.6: below ~0.5 m it clips body-height obstacles.
- `disRatioZ` ≥ 0.0: negative inverts the crop.

## Reading the report

Lower median ⇒ sparser/cleaner map. On each axis, take the value where the median stops
dropping meaningfully, **then verify visually** — `minRelZ`/`maxRelZ`/`minBlockPointNum`
reduce the count *mechanically*, so a low number can mean you erased real terrain, not just
noise. Apply the chosen values together back into the `terrainAnalysis` block and re-confirm.
