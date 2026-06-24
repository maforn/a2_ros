# FAR Planner Configuration and Tuning Guide

This guide explains the shipped FAR Planner YAML configs and how to tune them.
It covers:

- `src/far_planner/config/default.yaml`
- `src/far_planner/config/matterport.yaml`
- the small helper configs in `src/graph_decoder/config/default.yaml` and
  `src/boundary_handler/config/default.yaml`

The notes below are based on the current code paths in
`src/far_planner/src/far_planner.cpp` and the modules initialized from those
parameters. Paths in this guide are relative to the `far_planner/` package root.

## How Configs Are Loaded

Run the default profile:

```bash
roslaunch far_planner far_planner.launch
```

Run the Matterport profile:

```bash
roslaunch far_planner far_planner.launch config:=matterport
```

`far_planner.launch` loads:

```xml
$(find far_planner)/config/$(arg config).yaml
```

into the `/far_planner/` ROS namespace. It also starts RViz with:

```xml
$(find far_planner)/rviz/$(arg config).rviz
```

So if you create `config/my_robot.yaml`, also copy or create
`rviz/my_robot.rviz`, or edit the launch file to use an existing RViz file.

The launch also includes `graph_decoder/launch/decoder.launch`, which loads
`graph_decoder/config/default.yaml` into `/graph_decoder/`.

The boundary handler is launched separately with:

```bash
roslaunch boundary_handler boundary_handler.launch
```

That launch loads `boundary_handler/config/default.yaml`, then overrides file
paths from launch arguments.

## Shipped Profiles

| Setting group | `default.yaml` | `matterport.yaml` | Meaning |
| --- | ---: | ---: | --- |
| `voxel_dim` | `0.15` | `0.1` | Point cloud/filtering resolution. Matterport uses finer geometry. |
| `robot_dim` | `0.8` | `0.5` | Robot clearance footprint. Matterport assumes a smaller robot. |
| `vehicle_height` | `0.75` | `0.5` | Robot z offset above terrain. |
| `sensor_range` | `30.0` | `15.0` | Maximum surrounding map/graph radius. |
| `terrain_range` | `15.0` | `7.5` | Local terrain crop and scan grid radius. |
| `local_planner_range` | `5.0` | `2.5` | Lower-level waypoint/local planning horizon. |
| `MapHandler/cell_length` | `5.0` | `2.5` | Global map grid cell size. |
| `MapHandler/map_grid_max_length` | `1000.0` | `200.0` | Horizontal global map span. |
| `MapHandler/map_grad_max_height` | `100.0` | `10.0` | Vertical global map span. The key name is misspelled in code and YAML; keep this spelling. |
| `GPlanner/converge_distance` | `0.5` | `0.25` | Goal reached radius. |
| `GPlanner/goal_adjust_radius` | `2.0` | `1.0` | Radius for shifting a goal into free terrain. |

Use `default.yaml` for larger spaces and longer range sensors. Use
`matterport.yaml` as the starting point for compact indoor scenes.

## Important Derived Values

Some YAML values are not used directly. FAR Planner computes shared constants
from them:

| Derived value | Formula | Effect |
| --- | --- | --- |
| effective `terrain_range` | `min(terrain_range, sensor_range)` | Setting `terrain_range` above `sensor_range` has no extra effect. |
| `kLeafSize` | `voxel_dim` | Global cloud downsampling and many grid resolutions. |
| `kNearDist` | `robot_dim` | Near-node distance and trajectory/intermediate node spacing. |
| `kNavClearDist` | `robot_dim / 2 + voxel_dim` | Main navigation clearance margin. |
| `kMatchDist` | `robot_dim * 2 + voxel_dim` | Node/contour matching and terrain-connect checks. |
| `height_voxel_dim` | `voxel_dim * 2` | Terrain height map vertical resolution. |
| `cell_height` | `floor_height / 2.5` | Vertical map cell size. |
| `kTolerZ` | `floor_height - height_voxel_dim` | Same-layer vertical tolerance. |
| `kMarginDist` | `sensor_range - kMatchDist` | Margin for keeping/removing graph nodes. |
| `kMarginHeight` | `kTolerZ - cell_height / 2` | Terrain slope/height tolerance. |
| contour blur size | `round(kNavClearDist / voxel_dim)` | Obstacle image smoothing. |
| path momentum distance | `robot_dim / 2` | Movement threshold for path momentum logic. |
| frontier perimeter threshold | `kMatchDist * 4` | Small contours are not treated as frontiers. |

Important gotchas:

- `Util/nav_clear_dist` is read by the code, but then overwritten by
  `robot_dim / 2 + voxel_dim`. It is not present in the shipped YAML files.
- `Graph/connect_angle_thred` and `Graph/dirs_filter_margin` are read if added,
  but then overwritten by `Util/accept_max_align_angle` and `Util/angle_noise`.
- `GPlanner/free_counter_thred` is loaded into `gp_params_.free_thred`, but the
  current code does not use it.
- Angles in YAML are degrees. The code converts them to radians internally.

## Master Planner Parameters

| Parameter | Effect | Tune up when | Tune down when |
| --- | --- | --- | --- |
| `main_run_freq` | Main graph update loop rate and planning timer frequency. | The graph reacts too slowly and CPU is available. | CPU is high or sensor input is slower than the planner. |
| `voxel_dim` | Point cloud downsampling, scan/terrain grid resolution, contour image resolution. | CPU/memory is high, or small noise creates too many vertices. | Narrow passages or small obstacles are missed. |
| `robot_dim` | Robot clearance, node matching radius, graph spacing, frontier thresholds. | The robot plans too close to obstacles. | The planner refuses valid narrow passages. Only lower this if the robot really fits. |
| `vehicle_height` | Vertical offset from terrain to robot/navigation node height. | Planned z is too low relative to robot base. | Planned z is too high. |
| `sensor_range` | Surrounding map radius and contour extraction radius. | Long-range sensor data is reliable and you need earlier graph context. | Far points are noisy or runtime is high. |
| `terrain_range` | Local terrain crop radius and dynamic scan grid radius. Clamped to `sensor_range`. | Terrain/obstacle checks are too short. | Runtime is high or far terrain is unreliable. |
| `local_planner_range` | Waypoint projection horizon, boundary publication radius, local graph/terrain planning range. | The local planner can accept farther waypoints. | Local planner/controller needs closer waypoints. |
| `visualize_ratio` | RViz marker scale for graph direction markers. | Markers are hard to see. | RViz is cluttered. |
| `world_frame` | Planning frame for odom/cloud/goal transforms and marker frames. | Use the fixed map frame used by odometry and terrain clouds. | Do not use a drifting frame unless the rest of the stack also uses it. |

Boolean switches:

| Parameter | Effect |
| --- | --- |
| `is_viewpoint_extend` | Extends selected waypoints away from obstacles for better visibility. Disable if the robot over-shoots or the local planner already handles this well. |
| `is_multi_layer` | Enables multi-floor/layer checks. Enable for stairs, ramps, mezzanines, or stacked terrain. Keep false for simple single-floor maps. |
| `is_opencv_visual` | Opens the obstacle image display. Useful for debugging contour extraction, but costs UI/CPU. |
| `is_static_env` | If true, scan and local terrain callbacks for dynamic obstacle handling are ignored. Set false when moving obstacles matter. |
| `is_pub_boundary` | Publishes `/navigation_boundary` to the local planner. Only useful when custom boundaries are loaded/used. |
| `is_debug_output` | Enables ROS debug/warn style output instead of the compact terminal display. Useful while tuning. |
| `is_attempt_autoswitch` | Allows automatic switch from known-free navigation to attemptable navigation through unknown space. Disable for conservative known-map operation. |

## Map Handler Parameters

| Parameter | Effect | Tuning guidance |
| --- | --- | --- |
| `MapHandler/floor_height` | Determines vertical cell height, same-layer tolerance, and terrain height tolerance. | Increase for tall floors or rough vertical terrain. Decrease if separate floors are being merged. |
| `MapHandler/cell_length` | Global point cloud grid cell size. | Smaller cells give finer map bookkeeping but more cells. Larger cells reduce overhead but coarsen local extraction. |
| `MapHandler/map_grid_max_length` | Horizontal size of the global rolling/grid map. | Must cover the expected operating area. Larger values cost memory. |
| `MapHandler/map_grad_max_height` | Vertical size of the global map. | Must cover expected z travel. Keep the misspelled key name because the code expects it. |

The global grid dimensions are roughly:

```text
rows = ceil(map_grid_max_length / cell_length)
levels = ceil(map_grad_max_height / (floor_height / 2.5))
```

Large values can allocate many point cloud cells.

## Utility Parameters

| Parameter | Effect | Tuning guidance |
| --- | --- | --- |
| `Util/angle_noise` | Angular tolerance for noisy surface directions and graph direction filtering. | Increase if surface directions flicker from scan noise. Decrease if separate directions are being merged. |
| `Util/accept_max_align_angle` | Maximum alignment error for matching contour lines/connections. | Increase for noisier contours. Decrease for stricter geometry. |
| `Util/obs_inflate_size` | Obstacle inflation in voxels for dynamic scan grids and goal/free terrain checks. | Increase for safety around obstacles. Decrease only if valid narrow passages are blocked. |
| `Util/new_intensity_thred` | Threshold for classifying obstacle points as new after comparing current and previous clouds. | Increase to mark more partial-overlap points as new. Decrease to be stricter. |
| `Util/terrain_free_Z` | Splits terrain-analysis points into free and obstacle by intensity: intensity below this value is free. | Match this to your terrain-analysis output. If too many obstacles become free, lower it. If free terrain becomes obstacle, raise it. |
| `Util/dyosb_update_thred` | Minimum dynamic-obstacle point count before updating dynamic obstacle state. The YAML/code spelling is `dyosb`. | Increase to ignore small false detections. Decrease to react to small moving objects. |
| `Util/new_point_counter` | Number of nearby new points required before treating a graph point as near new observations. | Increase for stability in noisy clouds. Decrease if frontiers/new nodes are missed. |
| `Util/dynamic_obs_dacay_time` | Time, in seconds, to keep dynamic obstacle points. The YAML/code spelling is `dacay`. | Increase if obstacles vanish too quickly. Decrease if stale dynamic obstacles block paths. |
| `Util/new_points_decay_time` | Time, in seconds, to keep new obstacle points. | Increase for stable frontier detection. Decrease if stale new points cause graph clutter. |

## Dynamic Graph Parameters

| Parameter | Effect | Tune up when | Tune down when |
| --- | --- | --- | --- |
| `Graph/connect_votes_size` | Vote queue length for validating graph edges. Odom edges use about one third of this length. | Edges flicker because of noisy observations. | The graph reacts too slowly to visibility changes. |
| `Graph/clear_dumper_thred` | Failed re-evaluation count before clearing non-static nodes. Navpoints use twice this threshold. | Valid nodes are deleted too aggressively. | Bad nodes persist too long. |
| `Graph/node_finalize_thred` | Inlier/vote count before node position and surface direction are finalized. Also used in terrain/trajectory invalidation logic. | Node positions/directions are unstable. | Finalization is too slow or old nodes resist updates. |
| `Graph/filter_pool_size` | Sample queue size for RANSAC filtering of node positions and surface directions. | Observations are noisy and CPU is available. | Runtime is high or nodes adapt too slowly. |

## Corner Detector Parameters

| Parameter | Effect | Tuning guidance |
| --- | --- | --- |
| `CDetector/resize_ratio` | Upsamples the obstacle image before contour extraction. | Higher can smooth/improve contour geometry but costs CPU. Lower is faster but rougher. |
| `CDetector/filter_count_value` | Threshold for obstacle image binarization in dynamic environments only. | Increase if dynamic-mode images are too noisy. Decrease if obstacles disappear from the contour image. |
| `CDetector/is_save_img` | Saves obstacle images. | Enable only for debugging. |
| `CDetector/img_folder_path` | Save path for debug images. | Set to a writable folder before enabling image saving. |

## Graph Planner Parameters

| Parameter | Effect | Tuning guidance |
| --- | --- | --- |
| `GPlanner/converge_distance` | Distance from robot to goal/original goal that counts as reached. | Increase if the robot oscillates near the goal. Decrease if you need precise stopping. |
| `GPlanner/goal_adjust_radius` | Radius of the local free/obstacle grid used to adjust a clicked goal into free space. | Increase if goals near walls fail. Decrease if goals are shifted too far from the clicked point. |
| `GPlanner/free_counter_thred` | Loaded but currently unused. | Changing it should not affect current behavior. |
| `GPlanner/reach_goal_vote_size` | Vote queue length for validating connections to the goal node. | Increase if goal connections flicker. Decrease if goal connectivity reacts too slowly. |
| `GPlanner/path_momentum_thred` | Number of cycles the planner may keep a previous path/waypoint before switching. | Increase for smoother waypoint behavior. Decrease if the robot keeps following stale paths. |

## Graph Messager Parameters

| Parameter | Effect |
| --- | --- |
| `GraphMsger/robot_id` | ID used in published/saved visibility graph messages. ID `0` is reserved for graphs extracted from files according to the YAML comment. |

The graph messenger also inherits `world_frame`, graph vote size, pool size, and
distance margin from the main FAR Planner parameters.

## Graph Decoder Config

File: `src/graph_decoder/config/default.yaml`

| Parameter | Effect |
| --- | --- |
| `world_frame` | Frame used for decoded graph markers. Keep this aligned with FAR Planner's `world_frame`. |
| `visual_scale_ratio` | Marker scale multiplier for decoded graph visualization. |

This node subscribes to `/robot_vgraph`, the topic FAR Planner's graph
messenger publishes on. Note: `decoder.launch` contains a
`remap from="/planner_nav_graph"` line, but it has no effect because the node
never subscribes to a topic by that name.

## Boundary Handler Config

File: `src/boundary_handler/config/default.yaml`

| Parameter | Effect |
| --- | --- |
| `world_frame` | Frame used for boundary graph visualization/output. |
| `visual_scale_ratio` | Marker scale multiplier. |
| `height_tolz` | Vertical tolerance when extracting/associating boundary graph elements. |

`boundary_handler.launch` additionally sets:

| Launch arg | Default | Effect |
| --- | --- | --- |
| `boundary_file` | `boundary.ply` | Input polygon boundary file. |
| `traj_file` | `trajectory.txt` | Input trajectory file used to infer traversable side. |
| `graph_file` | `boundary_graph.vgh` | Output visibility graph file. |

The launch sets `folder_path` to `$(find far_planner)/data/`, so those files are
resolved under `src/far_planner/data/`.

## Symptom-Based Tuning

### Runtime or CPU Is Too High

Try these in order:

1. Increase `voxel_dim`.
2. Reduce `sensor_range` and `terrain_range`.
3. Reduce `main_run_freq`.
4. Increase `MapHandler/cell_length`.
5. Disable `is_opencv_visual`.
6. Keep `Graph/filter_pool_size` and vote sizes moderate.

### The Planner Misses Narrow Passages

Try:

1. Lower `voxel_dim`.
2. Lower `Util/obs_inflate_size`.
3. Check that `robot_dim` is not larger than the real required clearance.
4. Lower `CDetector/filter_count_value` if using dynamic mode and contours are missing.

Do not shrink `robot_dim` below the real robot footprint just to make paths
appear. That usually moves the collision problem downstream.

### The Robot Plans Too Close To Obstacles

Try:

1. Increase `robot_dim`.
2. Increase `Util/obs_inflate_size`.
3. Increase `GPlanner/goal_adjust_radius` if goals near walls remain unsafe.
4. Increase `voxel_dim` slightly if obstacle noise creates jagged contours.

### Graph Nodes or Edges Flicker

Try:

1. Increase `Graph/connect_votes_size`.
2. Increase `Graph/node_finalize_thred`.
3. Increase `Graph/filter_pool_size`.
4. Increase `Util/angle_noise` slightly for noisy surface directions.
5. Increase `Util/new_points_decay_time` if frontiers disappear between scans.

### Bad Nodes or Blocked Edges Persist Too Long

Try:

1. Decrease `Graph/clear_dumper_thred`.
2. Decrease `Graph/connect_votes_size`.
3. Decrease `Graph/node_finalize_thred`.
4. If dynamic obstacles are involved, decrease `Util/dynamic_obs_dacay_time`.

### Dynamic Obstacles Are Ignored

Check:

1. Set `is_static_env: false`.
2. Confirm `/scan_cloud` and `/terrain_local_cloud` are populated and in the
   same frame or transformable to `world_frame`.
3. Lower `Util/dyosb_update_thred`.
4. Increase `Util/dynamic_obs_dacay_time` if dynamic points vanish too quickly.

### False Dynamic Obstacles Appear

Try:

1. Increase `Util/dyosb_update_thred`.
2. Decrease `Util/dynamic_obs_dacay_time`.
3. Increase `voxel_dim` slightly to reduce scan noise.
4. Verify `terrain_free_Z` matches the terrain analysis intensity convention.

### Goal Handling Feels Wrong

If the robot stops too early, lower `GPlanner/converge_distance`.

If the robot oscillates near the goal, increase `GPlanner/converge_distance`.

If goals near obstacles fail, increase `GPlanner/goal_adjust_radius`.

If the clicked goal moves too far, decrease `GPlanner/goal_adjust_radius`.

If the robot keeps following an old path after the world changes, decrease
`GPlanner/path_momentum_thred`.

## Recommended Tuning Workflow

1. Start from the closest shipped profile: `default` for large spaces, `matterport`
   for small indoor spaces.
2. Match the robot and sensor first: `robot_dim`, `vehicle_height`,
   `voxel_dim`, `sensor_range`, `terrain_range`, and `world_frame`.
3. Match the environment scale: `MapHandler/floor_height`, `cell_length`,
   `map_grid_max_length`, and `map_grad_max_height`.
4. Run with `is_static_env: true` until the static graph and goal behavior are
   stable.
5. If needed, switch to `is_static_env: false` and tune dynamic-obstacle
   thresholds.
6. Tune graph stability with the `Graph/*` vote/finalization settings.
7. Tune user-facing navigation behavior with `GPlanner/*`.
8. Record `/runtime`, `/planning_time`, RViz graph quality, and goal success
   after each change. Change one or two related parameters at a time.

## Creating A New Profile

Example:

```bash
cp src/far_planner/config/default.yaml src/far_planner/config/my_robot.yaml
cp src/far_planner/rviz/default.rviz src/far_planner/rviz/my_robot.rviz
roslaunch far_planner far_planner.launch config:=my_robot
```

Then edit `my_robot.yaml` incrementally. Restart the launch after changing YAML;
the current node does not live-reload these parameters.
