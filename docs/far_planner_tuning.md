# FAR Planner tuning guide (A2)

How to tune the FAR Planner global visibility-graph planner for the Unitree A2, using
the upstream reference configs as a baseline.

- **Upstream reference (SMB/vehicle):**
  `external/far_planner/src/far_planner/config/default.yaml`
- **Upstream reference (Matterport indoor):**
  `external/far_planner/src/far_planner/config/matterport.yaml`
- **A2 override (what actually runs):**
  `src/meta_packages/a2_ros/config/autonomy/far_a2.yaml`

The A2 file is loaded by `navigation.launch.py` and `navigate_and_explore.launch.py`
(`parameters=[far_a2.yaml]`).

---

## 1. How parameter resolution actually works — read this first

`far_a2.yaml` is **not merged** with `default.yaml`. The launch files pass it as the
*only* parameter file for the node. Every parameter that is **not** listed in
`far_a2.yaml` falls back to the **C++ default baked into the node**, not to the value in
`default.yaml`.

The defaults live in `external/far_planner/src/far_planner/src/far_planner.cpp`
(`FARMaster::LoadParmasFromYaml`, around line 466). The node is created with default
`NodeOptions` (no `automatically_declare_parameters_from_overrides`), so:

> **A YAML key that does not exactly match a declared parameter name is silently
> ignored — no error, no effect.**

This has two practical consequences for `far_a2.yaml`:

### 1a. Two keys in `far_a2.yaml` are currently dead (silently ignored)

| Key as written in `far_a2.yaml` | Declared name in source | Result |
|---|---|---|
| `converge_distance: 0.5` | `g_planner/converge_distance` | ignored → uses code default **1.0** |
| `dynamic_obs_decay_time: 2.0` | `util/dynamic_obs_dacay_time` | ignored → uses code default **10.0** |

Both are missing the namespace prefix, and the second also has to match the upstream
misspelling **`dacay`**. As written, the robot treats the goal as reached at 1.0 m (not
0.5 m), and dynamic obstacles persist 10 s (not 2 s). See §5 for the corrected file.

### 1b. Several params silently inherit code defaults that differ from `default.yaml`

Because there is no merge, leaving a parameter out does **not** give you the
`default.yaml` value. Notable cases where the code default ≠ `default.yaml`:

| Parameter | code default (active for A2) | `default.yaml` | Effect of the gap |
|---|---|---|---|
| `is_opencv_visual` | **true** | false | OpenCV obstacle-image window enabled in a headless container |
| `g_planner/goal_adjust_radius` | **10.0** | 2.0 | goals snap to nearest reachable node within 10 m |
| `util/accept_max_align_angle` | **15.0°** | 4.0° | looser edge-alignment / redundant-edge pruning |
| `util/new_point_counter` | **10** | 5 | slower to register new obstacle points |
| `util/obs_inflate_size` | **2** | 1 | obstacles inflated by 2 voxels instead of 1 |
| `g_planner/reach_goal_vote_size` | **5** | 3 | needs more votes to declare goal reached |
| `g_planner/free_counter_thred` | **5** | 7 | — |
| `graph/node_finalize_thred` | **3** | 6 | nodes finalize on fewer observations |
| `graph/clear_dumper_thred` | **3** | 4 | nodes/edges pruned slightly sooner |
| `c_detector/resize_ratio` | **5.0** | 3.0 | contour image scaling |
| `c_detector/filter_count_value` | **5** | 3 | contour noise filter |
| `graph_msger/robot_id` | **0** | 1 | multi-robot graph id |

If you want any of these to behave like `default.yaml`, you must set them **explicitly**
in `far_a2.yaml`.

---

## 2. Parameter reference and A2 tuning

Columns: **code default** (active when omitted) · **default.yaml** · **matterport.yaml**
· **A2** (effective value once §1a fixes are applied). Bold A2 cells are values that
`far_a2.yaml` intentionally sets.

### Robot geometry / master (no namespace)

| Param | code | default | matterport | A2 | Meaning / tuning |
|---|---|---|---|---|---|
| `robot_dim` | 0.8 | 0.8 | 0.5 | **0.45** | Robot footprint diameter (m). Drives clearance/merge radii `kNearDist`, `kMatchDist` (=2·robot_dim+voxel), `kNavClearDist` (=robot_dim/2+voxel). Smaller → squeezes through tighter gaps but less safety margin. Set to A2 body diagonal. |
| `vehicle_height` | 0.75 | 0.75 | 0.5 | **0.5** | Standing height (m). Used in terrain/height feasibility of edges. Set to A2 stand height. |
| `sensor_range` | 30.0 | 20.0 | 15.0 | **10.0** | Max range (m) graph nodes are built/registered. Also caps `terrain_range`. Lower = smaller, denser graph near the robot; raise if the A2 lidar is trustworthy farther out. |
| `terrain_range` | 15.0 | 10.0 | 7.5 | 10.0¹ | Range for terrain analysis. `min(terrain_range, sensor_range)` — so capped to 10.0 here. |
| `local_planner_range` | 5.0 | 5.0 | 2.5 | 5.0 | Radius treated as the local planning region. Consider lowering toward the A2's actual local-planner horizon. |
| `voxel_dim` | 0.2 | 0.2 | 0.1 | **0.3** | Master leaf size (m) for clouds, contour image resolution, projection. Larger → coarser/cleaner contours, fewer spurious nodes, lower CPU; too large smears small gaps. A2 uses 0.3 to suppress noisy contours. |
| `main_run_freq` | 5.0 | 2.5 | 5.0 | **2.5** | Planner update rate (Hz). Lower = less CPU, slower reaction. |
| `visualize_ratio` | 1.0 | 0.75 | 0.4 | 1.0 | Fraction of markers visualized (cosmetic). |
| `is_static_env` | true | false | true | **false** | `true` freezes the graph once built (good for known/static maps). A2 keeps it dynamic so the graph updates as the world changes. |
| `is_pub_boundary` | true | false | false | **false** | Publish navigation-boundary polygons. Off for A2. |
| `is_viewpoint_extend` | true | true | true | true | Extend nodes to viewpoints around obstacles. |
| `is_opencv_visual` | true | false | false | true² | OpenCV obstacle-image display. **Recommend setting `false`** for headless containers (see §1b). |
| `is_attempt_autoswitch` | true | true | true | true | Auto-switch to "attemptable" navigation when no fully-free path exists. |
| `is_multi_layer` | false | false | false | false | Multi-floor graphs. |
| `world_frame` | map | map | map | map | Global frame. |

¹ inherited+capped, not set in `far_a2.yaml`. ² inherited code default; recommend overriding.

### Map handler (`map_handler/`)

| Param | code | default | matterport | Meaning |
|---|---|---|---|---|
| `floor_height` | 2.0 | 2.0 | 2.0 | Floor-to-ceiling height (m); sets `kTolerZ` height tolerance for edges and ceiling filtering. |
| `cell_length` | 5.0 | 5.0 | 2.5 | Map grid cell size (m). Smaller for tight indoor maps. |
| `map_grid_max_length` | 1000.0 | 1000.0 | 200.0 | Max map extent (m). |
| `map_grad_max_height` | 100.0 | 100.0 | 10.0 | Max height span (m). |

None are set for A2 → all use code defaults. For indoor A2 runs consider the
matterport-style smaller `cell_length`/extents.

### Utility (`util/`) — obstacle handling and graph geometry tolerances

| Param | code | default | matterport | Meaning / tuning |
|---|---|---|---|---|
| `angle_noise` | 15.0° | 15.0 | 15.0 | Angular noise tolerance; becomes `filter_dirs_margin` and the surface-direction RANSAC margin. Widens/narrows a node's "free direction" cone used to validate edges (§4). |
| `accept_max_align_angle` | 15.0° | 4.0 | 4.0 | Becomes `graph/connect_angle_thred` (`CONNECT_ANGLE_COS`). Controls convex-connect angle and **redundant-edge pruning** (§4). default/matterport tighten to 4°. |
| `terrain_free_Z` | 0.1 | 0.3 | 0.15 | Height band (m) treated as free/ground. |
| `obs_inflate_size` | 2 | 1 | 1 | Obstacle inflation in voxels. Larger = wider obstacle margins, fewer risky edges. |
| `new_intensity_thred` | 2.0 | 2.0 | 2.0 | New-point intensity threshold. |
| `new_point_counter` | 10 | 5 | 5 | Observations before a new obstacle point counts. |
| `dyosb_update_thred` | 4 | 4 | 4 | Dynamic-obstacle update count. |
| `dynamic_obs_dacay_time` | 10.0 | 2.0 | 10.0 | Seconds a dynamic obstacle persists before decay (**note `dacay` spelling**). A2 intends 2.0 (clear fast) — currently dead, see §1a. |
| `new_points_decay_time` | 2.0 | 0.2 | 1.0 | Seconds new points persist. |
| `nav_clear_dist` | 0.5 | — | — | Recomputed from `robot_dim` at load; YAML value is overwritten. |

### Dynamic graph (`graph/`) — V-graph edge/node retention (see §4)

| Param | code | default | matterport | Meaning |
|---|---|---|---|---|
| `connect_votes_size` | 10 | 10 | 10 | Temporal vote-window length for keeping a polygon (visibility) edge. |
| `clear_dumper_thred` | 3 | 4 | 4 | Consecutive "clear" counts before a node (and its edges) is pruned (×2 for navpoints). |
| `node_finalize_thred` | 3 | 6 | 6 | Observations to finalize node position/direction/frontier and invalidate terrain edges. |
| `filter_pool_size` | 12 | 12 | 12 | RANSAC pool size for node position/direction estimation. |
| `connect_angle_thred` | 10.0° | — | — | Overwritten by `util/accept_max_align_angle` at load. |
| `dirs_filter_margin` | 10.0° | — | — | Overwritten by `util/angle_noise` at load. |

### Graph planner (`g_planner/`) — goal handling and path smoothing

| Param | code | default | matterport | A2 | Meaning |
|---|---|---|---|---|---|
| `converge_distance` | 1.0 | 0.5 | 0.25 | 0.5³ | Distance (m) at which the goal is "reached". |
| `goal_adjust_radius` | 10.0 | 2.0 | 1.0 | 10.0 | Radius (m) to snap an unreachable goal to the nearest graph node. Large value = aggressive snapping; consider 1–2 m for A2. |
| `free_counter_thred` | 5 | 7 | 7 | 5 | Counts before a blocked path is treated as free again. |
| `reach_goal_vote_size` | 5 | 3 | 3 | 5 | Votes to confirm goal reached. |
| `path_momentum_thred` | 5 | 3 | 3 | **2** | Path "stickiness": higher keeps the current path longer (less switching/oscillation), lower replans more eagerly. A2 lowered to 2 for responsiveness. |

³ intended; currently dead — see §1a.

### Contour detector (`c_detector/`)

| Param | code | default | matterport | Meaning |
|---|---|---|---|---|
| `resize_ratio` | 5.0 | 3.0 | 3.0 | Obstacle-image upscaling for contour extraction. |
| `filter_count_value` | 5 | 3 | 3 | Min count to keep a contour pixel (noise filter). |
| `is_save_img` | false | false | false | Dump contour images to disk. |
| `img_folder_path` | "" | /path | /path | Output dir when saving. |

---

## 3. What was tuned for the A2, and why

Relative to the SMB-sized `default.yaml`, `far_a2.yaml` does three things:

1. **Shrinks the robot model** (`robot_dim` 0.8→0.45, `vehicle_height` 0.75→0.5) so the
   A2's smaller footprint can plan through gaps a full vehicle could not, and so terrain
   feasibility uses the correct stand height.
2. **Tightens sensing/compute** (`sensor_range` 30→10, `main_run_freq` 5→2.5,
   `voxel_dim` 0.2→0.3) for a dense, clean near-field graph at modest CPU.
3. **Makes the graph dynamic and the path responsive** (`is_static_env` false,
   `path_momentum_thred` →2), plus intended goal/obstacle tightening (`converge_distance`
   0.5, decay 2 s) that is currently inert due to the naming bug in §1a.

Everything else is **inherited code defaults**, which is why §1b matters: the active
behavior for goal snapping, obstacle inflation, OpenCV display, and several graph
thresholds comes from the C++ defaults, not from `default.yaml`.

---

## 4. Which parameters decide which V-graph edges are kept

Edge retention happens in `DynamicGraph::IsValidConnect`
(`src/dynamic_graph.cpp:226`). A candidate edge between two nav-nodes is kept only if it
passes **geometric gates** every cycle *and* accumulates enough **temporal votes**.

### 4a. Temporal voting — the primary keep/drop gate

Each cycle an edge that currently passes the gates gets a `+1` vote
(`RecordPolygonVote`); one that fails gets a vote removed (`DeletePolygonVote`). The edge
is considered "true" (kept) when the vote window holds a majority:

- `FARUtil::IsVoteTrue` returns true when `sum(votes) > floor(N/2)` (`utility.cpp:775`).
- The window length is **`graph/connect_votes_size`** (`votes_size`); for edges touching
  the robot/odom node the window is `ceil(votes_size/3)` so they form faster
  (`dynamic_graph.cpp:245`).

> **`graph/connect_votes_size`** is the main knob: larger → an edge must be observed
> free consistently over more cycles before it is trusted (stable graph, slow to add or
> drop edges); smaller → edges appear/disappear quickly (reactive but noisier).

Node/edge pruning is governed by **`graph/clear_dumper_thred`**: a node accrues a
clear-count and is merged/removed once it exceeds the threshold (×2 for navpoints,
`dynamic_graph.h:175`), taking its edges with it. **`graph/node_finalize_thred`** sets
how many observations finalize a node and how many bad terrain votes invalidate a
terrain edge (`dynamic_graph.cpp:321,361`). **`graph/filter_pool_size`** sets the RANSAC
pool that fixes node position/surface direction, which the direction gates below use.

### 4b. Geometric gates (must pass for a vote to be recorded)

Inside `IsValidConnect`, an edge only earns a vote when **all** of these hold
(`dynamic_graph.cpp:246`):

- **`IsConvexConnect`** — both nodes' free directions are mutually convex.
- **`IsInDirectConstraint`** — the edge leaves each node within its **free-direction
  cone**. Cone width is set by **`util/angle_noise`** (→ `filter_dirs_margin`, surface-dir
  RANSAC at `far_planner.cpp:604`).
- **`ContourGraph::IsNavNodesConnectFreePolygon`** — the **visibility check**: the edge
  must not cross an obstacle contour polygon. Contour fidelity is governed by
  **`voxel_dim`**, **`util/obs_inflate_size`**, and the `c_detector/*` params.
- **`IsOnTerrainConnect`** — terrain/height feasibility: rejects slopes > 45°, and
  height mismatches beyond `kMarginHeight` / `kTolerZ`. Driven by **`vehicle_height`**,
  **`map_handler/floor_height`**, **`voxel_dim`**, and **`robot_dim`** (via `kMatchDist`).

### 4c. Redundancy / direction pruning

Even a valid, voted edge is dropped if a **shorter edge exists in nearly the same
direction** (`IsSimilarConnectInDiection` → `IsAShorterConnectInDir`,
`dynamic_graph.cpp:373,440`). "Nearly the same direction" is `CONNECT_ANGLE_COS =
cos(kConnectAngleThred)`, and `kConnectAngleThred` is set from
**`util/accept_max_align_angle`** (`far_planner.cpp:605`). Wider angle → more aggressive
pruning of near-parallel edges (sparser graph); tighter angle (the 4° used by
`default.yaml`/`matterport.yaml`) → keeps more nearly-parallel edges.

**Summary — to change which V-graph edges are kept, tune:**

| Goal | Parameter(s) |
|---|---|
| More/less temporal stability before keeping an edge | `graph/connect_votes_size` |
| Prune stale nodes/edges faster/slower | `graph/clear_dumper_thred`, `graph/node_finalize_thred` |
| Allow edges leaving nodes at wider angles | `util/angle_noise` |
| More/less pruning of redundant near-parallel edges | `util/accept_max_align_angle` |
| Edge visibility / obstacle clearance strictness | `voxel_dim`, `util/obs_inflate_size`, `robot_dim`, `c_detector/*` |
| Edge terrain/height feasibility | `vehicle_height`, `map_handler/floor_height`, `voxel_dim` |

---

## 5. Recommended corrected `far_a2.yaml`

Fixes the two dead keys (§1a) and pins the three inherited defaults most likely to
surprise on the A2 (§1b): headless OpenCV display, aggressive goal snapping.

```yaml
far_planner:
  ros__parameters:
    # Robot dimensions - tuned for A2 (SMB defaults: robot_dim=0.8, vehicle_height=0.75)
    robot_dim:      0.45   # A2 body diagonal ~0.45 m
    vehicle_height: 0.5    # A2 standing height ~0.5 m

    # Sensor
    sensor_range:   10.0   # lidar cutoff 10 m

    # Planning
    main_run_freq:  2.5
    is_static_env:  false
    is_pub_boundary: false
    is_opencv_visual: false   # headless container - no X display

    # Contour detection
    voxel_dim: 0.3            # increase if contours are noisy

    # Goal (FIXED: namespaced so they actually apply)
    g_planner/converge_distance:  0.5
    g_planner/goal_adjust_radius: 2.0   # was inheriting code default 10.0
    g_planner/path_momentum_thred: 2

    # Dynamic obstacle decay (FIXED: correct namespace + upstream 'dacay' spelling)
    util/dynamic_obs_dacay_time: 2.0
```

> Verify after editing with `ros2 param get /far_planner g_planner/converge_distance`
> while the stack is running — a correctly-applied override reads back `0.5`, a dead key
> reads back the code default `1.0`.

---

## 6. Tuning workflow

1. Edit `src/meta_packages/a2_ros/config/autonomy/far_a2.yaml`.
2. `a2 nav` (or `a2 explore` / full stack), send a goal in RViz ('Goalpoint').
3. Watch the visibility graph + planned path in RViz; confirm overrides with
   `ros2 param get /far_planner <name>`.
4. Iterate. Geometry first (`robot_dim`, `voxel_dim`, `sensor_range`), then edge behavior
   (`graph/*`, `util/*` angles), then goal/path feel (`g_planner/*`).
