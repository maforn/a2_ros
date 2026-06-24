# TARE Planner Configuration and Tuning Guide

This guide explains the ROS 2 configuration files in `src/tare_planner/config/`,
what each group of parameters changes, and how to tune them safely for new
robots or environments.

## Where the Configs Are Used

The scenario YAMLs are loaded by `src/tare_planner/launch/explore.launch`:

```bash
ros2 launch tare_planner explore.launch scenario:=garage
```

`explore.launch` loads `<package_share>/<scenario>.yaml` into the
`tare_planner_node` node. The wrapper launches, such as
`explore_garage.launch` and `explore_tunnel.launch`, just call `explore.launch`
with a fixed scenario name and optionally start the `navigationBoundary` node:

```bash
ros2 launch tare_planner explore_garage.launch use_boundary:=true rviz:=true
```

Config files are installed from `src/tare_planner/config/` into the package
share directory. After editing a config in source, rebuild or use a symlink
install so the launched package sees the change:

```bash
colcon build --packages-select tare_planner --symlink-install
source install/setup.bash
```

All scenario files use this structure:

```yaml
tare_planner_node:
  ros__parameters:
    kAutoStart: true
```

Keep new parameters under `tare_planner_node.ros__parameters`.

## Scenario Presets

Use the closest existing scenario as the starting point.

| Scenario | Intended scale | Main characteristics |
| --- | --- | --- |
| `garage` | Medium outdoor/structured spaces | Baseline. Uses terrain height, 48 m x 48 m local horizon, 12 m sensor range. |
| `campus` | Larger outdoor spaces | Similar to garage, but 15 m sensor range and line-of-sight lookahead disabled. |
| `forest` | Sparse outdoor spaces | Coarser rolling grid, larger surface leaf size, 15 m sensor range, high frontier thresholds to ignore small/noisy frontiers. |
| `indoor` | Indoor corridors/rooms | No terrain height, 30 m x 30 m horizon, 7.5 m sensor range, lower frontier cluster size. |
| `tunnel` | Long constrained corridors | Indoor-like scale, momentum enabled, higher frontier threshold than indoor. |
| `matterport` | Small, high-resolution indoor maps | 6 m x 6 m horizon, 3 m sensor range, very fine collision grid, smaller graph distances, coverage boundary on frontiers. |

## Important Derived Values

Some important behavior is not controlled by a single explicit parameter:

| Derived value | Formula or source | Effect |
| --- | --- | --- |
| Local planning horizon X/Y | `viewpoint_manager/number_x * viewpoint_manager/resolution_x`, same for Y | Physical size of the local viewpoint lattice. |
| Viewpoint count | `number_x * number_y * number_z` | Main runtime driver for local planning. Current configs use `number_z: 1`. |
| Grid-world cell size | `viewpoint_manager/number_x * viewpoint_manager/resolution_x / 5` | Global exploration subspace width. There is no YAML key for this directly. |
| Frontier extraction range | `local_horizon_half_size + 2 * kSensorRange` in X/Y, fixed `2` in Z | Larger sensor range also expands frontier extraction work. |
| Rolling occupancy range | `kPointCloudCellSize * kPointCloudManagerNeighborCellNum` in X/Y, `kPointCloudCellHeight * neighbor_num` in Z | Determines how much nearby occupancy is retained. |

The viewpoint manager currently sets `dimension_ = 2` in code, and all shipped
YAMLs set `viewpoint_manager/number_z: 1` and `resolution_z: 0.0`. Treat this
package as configured for ground/2D viewpoint planning unless you also change
the code.

## Tuning Workflow

1. Copy the nearest scenario file and change the launch scenario name, or edit
   the scenario you already launch.
2. Set scale first: `kSensorRange`, viewpoint `number_*`, and viewpoint
   `resolution_*`.
3. Tune collision conservatism: collision cloud leaf size, collision grid
   resolution, viewpoint collision margins, and point thresholds.
4. Tune exploration aggressiveness: frontier cluster thresholds and
   `kMinAddPointNum*`.
5. Tune waypoint smoothness last: lookahead distance, waypoint extension,
   momentum, and return-home distances.
6. Validate each change in RViz using the viewpoint candidates, viewpoints,
   local planning horizon, frontiers, collision cloud, and waypoints.

Change one group at a time. Many parameters interact through derived values,
so changing horizon size and thresholds together can make it hard to see which
change helped.

## Startup, Topics, and Waypoints

| Parameter | Effect | Tuning notes |
| --- | --- | --- |
| `sub_*_topic_`, `pub_*_topic_` | ROS topics used for inputs and outputs. | Change these when integrating with a different autonomy stack. |
| `kAutoStart` | Starts exploration without waiting for `/start_exploration`. | Set `false` when testing launch/integration before motion. |
| `kRushHome` | Enables final return-home behavior once the robot is near home. | Usually keep enabled for missions that should end at the start. |
| `kRushHomeDist` | Distance at which the planner considers itself near home for final behavior. | Increase for large outdoor maps; decrease for tight indoor maps. |
| `kAtHomeDistThreshold` | Distance threshold for being at home. | Keep near localization/waypoint accuracy, commonly around `0.5`. |
| `kNoExplorationReturnHome` | Prevents extra exploration behavior during return-home/final states. | Keep true for predictable mission finish. |
| `kUseTerrainHeight` | Uses the terrain map to set waypoint height. | Disable when no reliable terrain map exists, as in indoor/tunnel presets. |
| `kCheckTerrainCollision` | Checks terrain collision using terrain point intensity. | Keep true if terrain collision cloud is meaningful. |
| `kTerrainCollisionThreshold` | Terrain intensity threshold treated as collision. | Lower is more conservative; higher ignores weaker collision evidence. |
| `kKeyposeCloudDwzFilterLeafSize` | Downsamples registered scans before stacking. | Larger is faster/noisier; smaller preserves detail but costs CPU and memory. |
| `kLookAheadDistance` | Desired lookahead distance along the selected path. | Larger smooths motion but can cut corners; smaller reacts faster in tight spaces. |
| `kExtendWayPoint` | Extends the waypoint beyond the selected lookahead point. | Disable or reduce distances if waypoints overshoot in narrow spaces. |
| `kExtendWayPointDistanceBig` | Extension distance when the next path point is in line of sight. | Increase in open areas; decrease near clutter. |
| `kExtendWayPointDistanceSmall` | Extension distance when line of sight is limited. | Use smaller values indoors and in tunnels. |
| `kUseLineOfSightLookAheadPoint` | Forces lookahead updates to respect line of sight. | True is safer in clutter; false can be smoother in large open terrain. |
| `kUseMomentum` | Biases against frequent direction reversals. | Useful in tunnels/corridors; can delay turns into side rooms. |
| `kDirectionChangeCounterThr` | Number of direction changes before momentum activates. | Increase if momentum activates too eagerly. |
| `kDirectionNoChangeCounterThr` | Stable-direction count before momentum deactivates. | Increase if momentum drops too quickly. |
| `kResetWaypointJoystickAxesID` | Joystick axis used to reset waypoint. | Set to an available axis index, or negative to avoid joystick reset logic. |

## Frontier and Planning Environment

| Parameter | Effect | Tuning notes |
| --- | --- | --- |
| `kUseFrontier` | Enables rolling-grid frontier extraction. | If false, exploration relies on coverage/object-surface information only. |
| `kFrontierClusterTolerance` | Euclidean clustering tolerance for frontier points. | Larger merges nearby frontiers; smaller separates detailed openings. |
| `kFrontierClusterMinSize` | Minimum points for a frontier cluster to survive. | Raise to ignore noise; lower to catch narrow doors or small openings. |
| `kUseCoverageBoundaryOnFrontier` | Filters frontiers by received coverage boundary. | Useful when an external boundary constrains exploration. |
| `kUseCoverageBoundaryOnObjectSurface` | Filters object/surface coverage by boundary. | Enable only if boundary input is trusted. |
| `kSurfaceCloudDwzLeafSize` | Downsample leaf size for surface/coverage clouds. | Larger is faster and less detailed; smaller preserves small surfaces. |
| `kCollisionCloudDwzLeafSize` | Downsample leaf size for collision clouds. | Smaller detects tighter obstacles; larger reduces CPU. |
| `kKeyposeCloudStackNum` | Number of recent keypose clouds stacked. | Larger gives more context but costs memory and can keep stale obstacles. |
| `kPointCloudRowNum`, `kPointCloudColNum`, `kPointCloudLevelNum` | Point-cloud manager grid dimensions. | Increase for larger retained maps; reduce if memory is high. |
| `kMaxCellPointNum` | Max points per point-cloud manager cell. | Raise only if cells saturate and detail is being clipped. |
| `kPointCloudCellSize`, `kPointCloudCellHeight` | Physical size of point-cloud manager cells. | Larger cells cover more area per cell; smaller cells localize data better. |
| `kPointCloudManagerNeighborCellNum` | Neighbor cells used around the robot. | Larger retains/queries more nearby data, but increases runtime. |
| `kCoverCloudZSqueezeRatio` | Compresses Z before coverage KD-tree checks. | Larger makes vertical differences matter less in coverage matching. |

## Rolling Occupancy Grid

| Parameter | Effect | Tuning notes |
| --- | --- | --- |
| `rolling_occupancy_grid/resolution_x/y/z` | Occupancy grid cell resolution. | Smaller is more accurate but slower and larger. Coarser values are reasonable outdoors. |

The grid range is derived from point-cloud manager cell sizes and neighbor
count, so changing `kPointCloudCellSize`, `kPointCloudCellHeight`, or
`kPointCloudManagerNeighborCellNum` also changes how much occupancy history is
available around the robot.

## Keypose Graph

The keypose graph is the global roadmap used to connect explored areas.

| Parameter | Effect | Tuning notes |
| --- | --- | --- |
| `keypose_graph/kAddNodeMinDist` | Minimum distance before adding a new keypose node. | Larger creates a sparser graph; smaller is better for tight indoor maps. |
| `keypose_graph/kAddNonKeyposeNodeMinDist` | Minimum distance for intermediate/non-keypose nodes. | The planner later overrides this to half the smaller viewpoint resolution. |
| `keypose_graph/kAddEdgeConnectDistThr` | Connects a new keypose to nearby graph nodes within this distance. | Increase in open areas; decrease in clutter. |
| `keypose_graph/kAddEdgeToLastKeyposeDistThr` | Allows edge to last keypose if close enough. | Keep similar to connect distance unless odometry/keypose spacing is unusual. |
| `keypose_graph/kAddEdgeVerticalThreshold` | Max vertical difference for edge candidates. | Increase for ramps/stairs; decrease for flat indoor maps. |
| `keypose_graph/kAddEdgeCollisionCheckResolution` | Step size along an edge during collision checks. | Smaller is safer but slower. |
| `keypose_graph/kAddEdgeCollisionCheckRadius` | Radius used for graph edge collision checks. | Larger is more conservative. |
| `keypose_graph/kAddEdgeCollisionCheckPointNumThr` | Collision point count threshold for rejecting an edge. | Lower is more conservative; higher tolerates sparse/noisy collision returns. |

## Viewpoints and Local Coverage

| Parameter | Effect | Tuning notes |
| --- | --- | --- |
| `viewpoint_manager/number_x/y/z` | Number of candidate viewpoints in the local lattice. | Runtime grows with the product. Current configs are effectively 2D. |
| `viewpoint_manager/resolution_x/y/z` | Spacing between candidate viewpoints. | Smaller catches tight maneuvers; larger is faster and better for open spaces. |
| `kSensorRange` | Max range used for visibility and coverage. | Larger sees farther and extracts more frontier work; smaller is safer indoors. |
| `kNeighborRange` | Radius used when finding nearby viewpoints. | Should be a few viewpoint cells wide. Too small can disconnect local paths. |
| `kGreedyViewPointSampleRange` | Local coverage planner sample window. | Larger explores more candidates but costs runtime. |
| `kLocalPathOptimizationItrMax` | Max local path optimization iterations. | Increase only if path quality is poor and runtime budget allows it. |
| `kConnectivityHeightDiffThr` | Max height difference between connected viewpoints. | Increase for rough terrain; keep low on flat floors. |
| `kViewPointCollisionMargin` | XY collision safety margin around viewpoints. | Increase for larger robot footprint or noisy maps; decrease if planner is overly conservative. |
| `kViewPointCollisionMarginZPlus/ZMinus` | Vertical collision margin above/below viewpoint. | Tune to robot height and terrain noise. |
| `kCollisionGridResolutionX/Y/Z` | Internal grid resolution for viewpoint collision checks. | Smaller improves collision precision but can be expensive. |
| `kCollisionGridZScale` | Scales Z during collision correspondence. | Larger makes vertical separation count more during collision association. |
| `kCollisionPointThr` | Points needed to mark collision. | Lower is more conservative; higher ignores sparse noise. |
| `kLineOfSightStopAtNearestObstacle` | Stops line-of-sight tracing at nearest obstacle. | True is conservative; false can allow more viewpoints behind sparse clutter. |
| `kCheckDynamicObstacleCollision` | Tracks repeated collision observations for dynamic obstacles. | Enable for live obstacle-rich scenes; all current YAMLs set false. |
| `kCollisionFrameCountMax` | Frames before dynamic collision blocks a viewpoint. | Lower reacts faster; higher filters transient noise. |
| `kViewPointHeightFromTerrain` | Target viewpoint height above terrain. | Match sensor/robot height. Used when terrain height is enabled. |
| `kViewPointHeightFromTerrainChangeThreshold` | Max height adjustment change before updating. | Lower follows terrain more tightly; higher smooths height changes. |
| `kCoverageOcclusionThr` | Visibility/coverage occlusion threshold. | Lower is stricter; higher accepts more partially occluded points. |
| `kCoverageDilationRadius` | Dilation around covered points. | Larger marks more area covered; smaller requires denser inspection. |
| `kMinAddPointNumSmall` | Minimum newly covered points for local/global decisions. | Lower is more eager; higher avoids chasing low-value views. |
| `kMinAddPointNumBig` | Higher threshold for strong global cell value. | Increase in noisy/open scenes; decrease in tight scenes. |
| `kMinAddFrontierPointNum` | Minimum frontier points for a viewpoint/cell to matter. | Raise to suppress noise; lower to enter small openings. |

## Grid World

The grid world stores global exploration subspaces. It uses the derived cell
size described above, plus explicit Z height.

| Parameter | Effect | Tuning notes |
| --- | --- | --- |
| `kGridWorldXNum/YNum/ZNum` | Number of global subspace cells. | Larger covers bigger missions but increases initialization and memory. |
| `kGridWorldCellHeight` | Height of each global subspace cell. | Increase for tall outdoor spaces; decrease for low indoor maps. |
| `kGridWorldNearbyGridNum` | Number of nearby grid cells considered around the robot. | Larger allows more local/global options; smaller is faster. |
| `kCellExploringToCoveredThr` | Evidence count to move exploring cells to covered. | Lower finishes cells sooner. |
| `kCellCoveredToExploringThr` | Evidence count to reopen covered cells. | Lower reopens areas more easily. |
| `kCellExploringToAlmostCoveredThr` | Evidence count to mark exploring as almost covered. | Lower makes progress states advance sooner. |
| `kCellAlmostCoveredToExploringThr` | Evidence count to move almost-covered back to exploring. | Lower makes planner revisit partially covered cells more readily. |
| `kCellUnknownToExploringThr` | Evidence count to promote unknown cells to exploring. | Lower discovers cells sooner. |

## Visualization

Visualization parameters affect RViz markers only:

| Parameter | Effect |
| --- | --- |
| `kExploringSubspaceMarkerColor*` | Color and alpha of global/exploring subspace markers. |
| `kLocalPlanningHorizonMarkerColor*` | Color and alpha of local planning horizon marker. |
| `kLocalPlanningHorizonMarkerWidth` | Intended line width for the local planning horizon marker. |
| `kLocalPlanningHorizonHeight` | Height of the local planning horizon marker. |

## Symptom-Based Tuning

| Symptom | Try these changes |
| --- | --- |
| Planner is too slow | Reduce viewpoint `number_x/y`, increase viewpoint resolution, increase cloud leaf sizes, coarsen rolling/collision grid resolution, reduce `kSensorRange`, or reduce point-cloud grid dimensions. |
| Planner misses narrow obstacles | Decrease `kCollisionCloudDwzLeafSize`, decrease collision grid resolution, increase collision margins, or lower `kCollisionPointThr`. |
| Planner is too conservative or gets stuck near free space | Decrease collision margins, increase `kCollisionPointThr`, coarsen overly fine collision data, or reduce `kCoverageDilationRadius`. |
| Planner chases noisy frontiers | Increase `kFrontierClusterMinSize`, increase `kMinAddFrontierPointNum`, or coarsen `kSurfaceCloudDwzLeafSize`. |
| Planner ignores doorways/small side openings | Decrease `kFrontierClusterMinSize`, decrease `kMinAddFrontierPointNum`, decrease viewpoint resolution, or reduce `kSensorRange` indoors. |
| Waypoints jump too far ahead | Lower `kLookAheadDistance`, lower `kExtendWayPointDistanceBig/Small`, or disable `kExtendWayPoint`. |
| Motion oscillates in corridors | Enable `kUseMomentum`, increase `kDirectionChangeCounterThr`, or increase `kLookAheadDistance`. |
| Return home starts too late | Increase `kRushHomeDist`. |
| Memory is high | Reduce `kPointCloudRowNum/ColNum/LevelNum`, reduce `kMaxCellPointNum`, reduce `kGridWorldXNum/YNum/ZNum`, or reduce viewpoint count. |

## Config Gotchas

These are current-code details that affect tuning:

| Detail | Impact |
| --- | --- |
| `kCoveragePointCloudResolution` is declared with default `1.0` but is not present in the scenario YAMLs. | Add it to a YAML if you need to tune in-FOV thresholds in `ViewPointManager`. |
| `kGridWorldCellSize` is derived from the viewpoint grid. | Changing viewpoint number/resolution also changes global subspace width. |
| `matterport.yaml` contains un-namespaced `kKeyposeGraphCollisionCheckRadius` and `kKeyposeGraphCollisionCheckPointNumThr`. | The code reads the namespaced `keypose_graph/kAddEdgeCollisionCheckRadius` and `keypose_graph/kAddEdgeCollisionCheckPointNumThr`; the un-namespaced keys do not control the current readers. |
| `TAREVisualizer` reads `kLocalPlanningHorizonMarkerWidth` from the key `kLocalPlanningHorizonMarkerColorA`. | Changing `kLocalPlanningHorizonMarkerWidth` in YAML may not change the marker width until the code is fixed. |
| `TAREVisualizer` reads both X and Y marker resolution from `viewpoint_manager/resolution_x`. | This affects marker dimensions, not the actual viewpoint manager lattice. |

## Practical Starting Points

For a new indoor robot, start from `indoor.yaml`. Set `kSensorRange` to the
usable sensor range in clutter, keep `kUseTerrainHeight: false`, then tune
viewpoint resolution until local paths fit through doors without making runtime
too high.

For a new outdoor/rough-terrain robot, start from `garage.yaml` or
`campus.yaml`. Keep terrain height enabled if the terrain map is reliable, set
`kSensorRange` to the range where obstacle and surface returns are still
trustworthy, and tune collision margins to the robot footprint plus localization
error.

For a very small reconstructed map or simulation, start from `matterport.yaml`.
Keep the high-resolution collision settings only if runtime is acceptable; this
preset is intentionally much finer than the other scenarios.
