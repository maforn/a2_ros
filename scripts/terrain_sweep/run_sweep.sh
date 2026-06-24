#!/usr/bin/env bash
#
# Automatic one-axis-at-a-time terrainAnalysis parameter sweep.
#
# For every variant produced by gen_variants.py this:
#   1. launches ONLY the terrainAnalysis node with that variant's params + use_sim_time
#   2. records /terrain_map to a per-variant mcap bag
#   3. replays the input bag (/registered_scan + /state_estimation + /tf) with --clock
#   4. tears the node + recorder down cleanly (params are read once at startup, so every
#      variant needs a fresh node)
# then runs analyze.py to turn the recorded /terrain_map clouds into a per-axis report.
#
# Run INSIDE the a2_ros_dev container (ROS 2 + workspace sourced):
#   docker compose exec a2_ros_dev bash
#   scripts/terrain_sweep/run_sweep.sh
#
# Common options:
#   --bag DIR        input bag (default: bags/bag_20260622_155851_nav_test)
#   --rate R         bag playback rate (default 1.0; keep identical across variants)
#   --warmup SEC     steady-state warmup dropped by analyze.py (default 13)
#   --play-timeout S hard cap on one bag playback before forced teardown (default 600)
#   --gen-only       just (re)generate variants, don't run
#   --no-analyze     run the sweep but skip analyze.py
#   --only NAME      run a single variant by name (e.g. baseline, scanVoxelSize__0.2)
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# ---- defaults ----
BAG="$REPO_ROOT/bags/bag_20260622_155851_nav_test"
RATE="1.0"
CLOCK_HZ="100"
SETTLE="2"      # let node + recorder discover before scans start
DRAIN="3"       # let the last /terrain_map frames be recorded after playback ends
WARMUP="13"     # >= max(decayTime=10, ~3*voxelTimeUpdateThre=6); passed to analyze.py
PLAY_TIMEOUT="600"   # hard cap (s) on one bag playback; backstop against a wedged play
GEN_ONLY=0
DO_ANALYZE=1
ONLY=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)        BAG="$2"; shift 2 ;;
    --rate)       RATE="$2"; shift 2 ;;
    --clock-hz)   CLOCK_HZ="$2"; shift 2 ;;
    --settle)     SETTLE="$2"; shift 2 ;;
    --drain)      DRAIN="$2"; shift 2 ;;
    --warmup)     WARMUP="$2"; shift 2 ;;
    --play-timeout) PLAY_TIMEOUT="$2"; shift 2 ;;
    --gen-only)   GEN_ONLY=1; shift ;;
    --no-analyze) DO_ANALYZE=0; shift ;;
    --only)       ONLY="$2"; shift 2 ;;
    -h|--help)    awk 'NR>1 && /^#/{sub(/^# ?/,"");print;next} NR>1{exit}' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

# ---- environment sanity ----
command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found -- run inside the a2_ros_dev container." >&2; exit 1; }
if ! ros2 pkg executables terrain_analysis 2>/dev/null | grep -q terrainAnalysis; then
  echo "terrain_analysis/terrainAnalysis not found. Build it first, e.g.: a2 build a2_robot" >&2
  exit 1
fi
[[ -d "$BAG" ]] || { echo "input bag not found: $BAG" >&2; exit 1; }

VARIANTS_DIR="$SCRIPT_DIR/variants"

# ---- 1. (re)generate variants ----
echo "==> generating variants"
python3 "$SCRIPT_DIR/gen_variants.py" --out "$VARIANTS_DIR" || exit 1
[[ "$GEN_ONLY" == 1 ]] && exit 0

RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$SCRIPT_DIR/runs/$RUN_ID"
mkdir -p "$RUN_DIR"
cp "$VARIANTS_DIR/manifest.json" "$RUN_DIR/manifest.json"
echo "==> run dir: $RUN_DIR"
echo "    bag=$BAG rate=$RATE clock=${CLOCK_HZ}Hz warmup=${WARMUP}s"

# Stop a background pid without ever blocking the sweep: SIGINT first (lets `ros2 bag
# record` flush its mcap), then a watchdog escalates to TERM/KILL if it ignores INT
# (e.g. a wedged rmw_zenoh teardown or a hung ros2 CLI). `wait` reaps the child so no
# zombies; the watchdog is cancelled the instant the child exits, so a clean stop fires
# no kills. This is the fix for the loop stalling instead of advancing to the next variant.
_stop() {
  local pid="$1" grace="${2:-6}"
  kill -0 "$pid" 2>/dev/null || { wait "$pid" 2>/dev/null; return; }
  kill -INT "$pid" 2>/dev/null
  ( sleep "$grace"; kill -TERM "$pid" 2>/dev/null; sleep 2; kill -KILL "$pid" 2>/dev/null ) 2>/dev/null &
  local watch=$!
  wait "$pid" 2>/dev/null
  kill "$watch" 2>/dev/null; wait "$watch" 2>/dev/null
}

cleanup() { pkill -INT -f 'terrainAnalysis --ros-args' 2>/dev/null; }
trap cleanup EXIT INT TERM

# Collect variant configs (baseline first for a stable reference ordering).
mapfile -t CONFIGS < <(ls "$VARIANTS_DIR"/baseline.yaml "$VARIANTS_DIR"/*.yaml 2>/dev/null | awk '!seen[$0]++')

run_variant() {
  local cfg="$1" name; name="$(basename "$cfg" .yaml)"
  local out_bag="$RUN_DIR/$name"

  pkill -INT -f 'terrainAnalysis --ros-args' 2>/dev/null; sleep 0.5

  ros2 run terrain_analysis terrainAnalysis --ros-args \
      -r __node:=terrainAnalysis --params-file "$cfg" -p use_sim_time:=true \
      >"$RUN_DIR/$name.node.log" 2>&1 &
  local node_pid=$!
  sleep "$SETTLE"

  # Verify the full block actually loaded: a canary baseline key must read its yaml
  # value (10.0), NOT the C++ default (2.0). Catches typo'd/dead keys silently.
  local canary
  canary="$(timeout 8 ros2 param get /terrainAnalysis decayTime 2>/dev/null | grep -oE '[0-9.]+' | tail -1)"
  if [[ -n "$canary" && "$canary" != "10.0" ]]; then
    echo "   WARNING: canary decayTime=$canary (expected 10.0) -- params may not have loaded!"
  fi

  ros2 bag record -s mcap -o "$out_bag" /terrain_map \
      >"$RUN_DIR/$name.rec.log" 2>&1 &
  local rec_pid=$!
  sleep 1

  timeout "$PLAY_TIMEOUT" ros2 bag play "$BAG" --clock "$CLOCK_HZ" --rate "$RATE" \
      --topics /registered_scan /state_estimation /tf /tf_static \
      >"$RUN_DIR/$name.play.log" 2>&1
  local rc=$?
  [[ $rc -eq 124 ]] && echo "   WARNING: bag play hit ${PLAY_TIMEOUT}s timeout -- forcing teardown"
  sleep "$DRAIN"

  _stop "$rec_pid" 8        # SIGINT + watchdog: graceful mcap flush, never blocks
  _stop "$node_pid" 6
  pkill -INT -f 'terrainAnalysis --ros-args' 2>/dev/null
  sleep 0.5
}

# ---- 2. sweep ----
total=${#CONFIGS[@]}; idx=0
for cfg in "${CONFIGS[@]}"; do
  name="$(basename "$cfg" .yaml)"
  idx=$((idx + 1))
  [[ -n "$ONLY" && "$name" != "$ONLY" ]] && continue
  echo "==> [$idx/$total] $name  ($(date +%H:%M:%S))"
  run_variant "$cfg"
done
echo "==> sweep complete"

# ---- 3. analyze ----
if [[ "$DO_ANALYZE" == 1 ]]; then
  echo "==> analyzing"
  python3 "$SCRIPT_DIR/analyze.py" --run-dir "$RUN_DIR" --warmup "$WARMUP"
  echo
  echo "Report: $RUN_DIR/results.md   (CSV: $RUN_DIR/results.csv)"
  echo "Visually compare a variant's terrain map in RViz with, e.g.:"
  echo "  ros2 bag play $RUN_DIR/scanVoxelSize__0.2 --clock 100   # then view /terrain_map"
fi
