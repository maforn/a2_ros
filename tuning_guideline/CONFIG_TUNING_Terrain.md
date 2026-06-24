# Terrain Analysis Parameter Tuning Guide

This package runs the `terrainAnalysis` ROS 2 node from
`launch/terrain_analysis.launch`. The node subscribes to:

- `/state_estimation` (`nav_msgs/msg/Odometry`)
- `/registered_scan` (`sensor_msgs/msg/PointCloud2`)
- `/joy` (`sensor_msgs/msg/Joy`) for manual local map clearing
- `/map_clearing` (`std_msgs/msg/Float32`) for commanded local map clearing

It publishes `/terrain_map` (`sensor_msgs/msg/PointCloud2`) in the `map` frame.
Published point intensity is the point elevation relative to the estimated local
ground height. With `considerDrop=true`, intensity is the absolute elevation
difference, so both bumps and drops can appear.

The parameters are read once at node startup. Changing a parameter with
`ros2 param set` while the node is already running will not change behavior
unless the code is extended with a parameter update callback. For tuning, edit
`launch/terrain_analysis.launch` or pass startup parameters, then restart the
node.

## Processing Overview

At a high level, the node:

1. Crops incoming registered scan points by distance and relative height around
   the vehicle.
2. Stores recent points in a rolling terrain voxel map centered on the vehicle.
3. Periodically downsamples and prunes old points from each terrain voxel.
4. Estimates a ground height for each local planar grid cell.
5. Publishes points whose height above, or distance from, the local ground is
   within the configured vehicle-height band.
6. Optionally suppresses likely stale dynamic obstacles.
7. Optionally turns no-data cells into synthetic obstacle points.

Important fixed code constants:

- Terrain storage voxels are `1.0 m` cells in a `21 x 21` rolling map.
- The output uses the central `11 x 11` terrain voxel area.
- Planar elevation cells are `0.2 m` cells in a `51 x 51` local grid.

## Parameter Reference

| Parameter | Launch value | Unit | Effect | Tuning guidance |
| --- | ---: | --- | --- | --- |
| `scanVoxelSize` | `0.05` | m | PCL voxel-grid leaf size used when downsampling stored scan points. | Lower keeps more detail but costs more CPU and memory. Higher is smoother and faster but can erase small obstacles or terrain detail. |
| `decayTime` | `2.0` | s | Stored points older than this are removed when their terrain voxel updates, except points within `noDecayDis`. | Lower removes stale/moving objects faster. Higher stabilizes sparse maps but keeps ghost obstacles longer. |
| `noDecayDis` | `4.0` | m | Points inside this horizontal radius around the vehicle are kept regardless of age. Also controls the travel distance before `noDataObstacle` can activate after startup or clearing. | Increase if nearby terrain flickers. Decrease if stale nearby objects remain too long. |
| `clearingDis` | `8.0` | m | Radius removed from the stored map when clearing is triggered. `/map_clearing` overrides this value with the message data for that clear. | Increase to clear a wider local map. Decrease for more targeted clearing. |
| `useSorting` | `false` | bool | If true, ground height is selected from a sorted height quantile. If false, ground height is the minimum height sample in each cell. | Enable for noisy scans, vegetation, or mixed returns. Disable when the lowest return is the most reliable ground cue. |
| `quantileZ` | `0.25` | ratio | Ground-height quantile used only when `useSorting=true`. Values are clamped to the available sorted sample index. | Lower values behave closer to minimum height. Higher values raise the estimated ground and reduce false obstacle height, but can hide low obstacles. |
| `considerDrop` | `true` | bool | If true, uses absolute height difference from ground. If false, only points above estimated ground are considered. | Enable when negative obstacles, holes, curbs, or drop-offs matter. Disable if below-ground returns create false positives. |
| `limitGroundLift` | `false` | bool | If true and `useSorting=true`, caps the selected ground height to `min_z + maxGroundLift`. | Enable when obstacles or vegetation lift the ground estimate too high. |
| `maxGroundLift` | `0.15` | m | Maximum allowed lift above the minimum height sample when `limitGroundLift=true`. | Lower makes the ground estimate hug the lowest sample. Higher allows smoother ground on rough terrain. |
| `clearDyObs` | `true` | bool | Enables suppression of cells that look like stale dynamic obstacles. A cell is suppressed only from accumulated map points; any fresh return in the current scan that passes the angle test resets that cell's count, so currently observed obstacles are never cleared (see note below). | Enable for people, vehicles, or objects that move through the map. Disable if static obstacles disappear unexpectedly. |
| `minDyObsDis` | `0.3` | m | Dynamic-obstacle logic treats points closer than this as immediately eligible for suppression. Farther points must pass angle tests. | Lower if nearby valid obstacles are removed. Higher if close-range stale points persist. |
| `minDyObsAngle` | `0.0` | deg | Minimum vertical angle test used by dynamic-obstacle suppression. | Increase to make dynamic clearing less aggressive. Decrease to mark more points as clearable. |
| `minDyObsRelZ` | `-0.5` | m | Relative height reference used in the dynamic-obstacle angle test. | More negative makes the angle larger and clearing more aggressive. Less negative makes clearing more selective. |
| `absDyObsRelZThre` | `0.2` | m | Points near the vehicle-frame horizontal plane are eligible for dynamic suppression even if outside the configured vertical FOV. | Lower to avoid clearing valid flat/low structures. Higher to remove more near-horizontal stale points. |
| `minDyObsVFOV` | `-16.0` | deg | Lower vertical field-of-view bound for dynamic-obstacle visibility checks. | Match the lower vertical FOV of the sensor or the useful vertical band for clearing. |
| `maxDyObsVFOV` | `16.0` | deg | Upper vertical field-of-view bound for dynamic-obstacle visibility checks. | Match the upper vertical FOV of the sensor or narrow it if clearing is too aggressive. |
| `minDyObsPointNum` | `1` | points | Minimum dynamic-obstacle count needed before a planar cell is suppressed. | Increase to require more evidence before removing a cell. Decrease for more aggressive clearing. |
| `noDataObstacle` | `false` | bool | If true, cells with too few samples become synthetic obstacle points after the vehicle has moved at least `noDecayDis` since startup or clearing. | Enable for conservative navigation through unknown space. Disable when unknown cells should remain empty. |
| `noDataBlockSkipNum` | `0` | cells | Number of no-data boundary layers skipped before synthetic obstacle points are published. | Increase to avoid marking thin edge holes as obstacles. Use `0` for the most conservative unknown-space blocking. |
| `minBlockPointNum` | `10` | points | Minimum number of height samples required for a planar cell to publish real terrain points. Also defines the no-data threshold. | Increase to reject sparse noise. Decrease to fill sparse maps, with more risk of noisy terrain. |
| `vehicleHeight` | `1.5` | m | Maximum accepted elevation difference for published terrain points. Synthetic no-data points use this as intensity. | Set near the useful vertical obstacle band for the robot. Lower clips tall objects; higher keeps more vertical structures. |
| `voxelPointUpdateThre` | `100` | points | A terrain voxel is downsampled and pruned after this many new points are added. | Lower updates map cleanup more often but costs more CPU. Higher reduces CPU but can delay decay and downsampling. |
| `voxelTimeUpdateThre` | `2.0` | s | A terrain voxel is downsampled and pruned after this much time since its last update. | Lower removes stale points faster. Higher reduces CPU and can keep stale points longer. |
| `minRelZ` | `-2.5` | m | Minimum allowed point height relative to the vehicle for ground estimation and output. The initial scan crop expands this lower bound by `disRatioZ * distance`. | Lower to keep steep downhill ground or drop-offs. Raise to reject low outliers. |
| `maxRelZ` | `1.0` | m | Maximum allowed point height relative to the vehicle for ground estimation and output. The initial scan crop expands this upper bound by `disRatioZ * distance`. | Raise to keep taller obstacles or uphill terrain. Lower to reject overhead clutter, walls, or vegetation. |
| `disRatioZ` | `0.2` | m/m | Distance-based expansion of the relative-height crop during scan intake and stored-point pruning. | Increase for slopes, pose error, or far-range vertical uncertainty. Decrease to reject more far-range clutter. |

## Dynamic-Obstacle Clearing Detail

When `clearDyObs=true`, suppression runs in two passes:

1. **Accumulation pass** over the stored terrain cloud. Each point that clears
   `minDyObsDis` and the `minDyObsAngle` / VFOV tests increments a per-cell
   dynamic-obstacle counter. Points closer than `minDyObsDis` add
   `minDyObsPointNum` at once, so a single close return is enough to mark the
   cell. A cell is suppressed (its real points are withheld) once that counter
   reaches `minDyObsPointNum`.
2. **Rescue pass** over the *current* registered scan only. Any fresh point in
   the latest scan that passes the same `minDyObsAngle` test resets its cell's
   counter to `0`.

The practical effect is that dynamic clearing removes only *stale* points: a
cell that still produces returns in the current scan is never cleared, while a
cell whose returns have disappeared (a moving object that left) stays cleared.
This is why `clearDyObs` rarely removes valid static obstacles in front of the
sensor, but reliably erases ghosts left behind by moving objects.

## How To Tune

Start with the launch defaults, record or replay a representative bag, and view
`/terrain_map` colored by intensity. Change one parameter group at a time and
restart the node after each change.

Typical startup:

```bash
# From the ROS 2 workspace root:
colcon build --packages-select terrain_analysis
source install/setup.bash
ros2 launch terrain_analysis terrain_analysis.launch
```

If you edit `launch/terrain_analysis.launch` and launch the installed package,
rebuild the package before retesting. During heavy tuning, building the
workspace with `colcon build --symlink-install` avoids this extra copy step for
launch-file edits.

For quick one-off tests without editing the launch file:

```bash
ros2 run terrain_analysis terrainAnalysis --ros-args -p scanVoxelSize:=0.08 -p decayTime:=1.0
```

Manual clearing:

- Press joystick button index `5` on `/joy` to clear points within
  `clearingDis`.
- Publish a custom clearing radius on `/map_clearing`:

```bash
ros2 topic pub --once /map_clearing std_msgs/msg/Float32 "{data: 5.0}"
```

## Symptom-Based Adjustments

### Map is noisy or too dense

- Increase `scanVoxelSize`.
- Increase `minBlockPointNum`.
- Use `useSorting=true` and start with `quantileZ=0.2` to `0.35`.
- Tighten `minRelZ`, `maxRelZ`, or lower `disRatioZ` if clutter is outside the
  useful terrain band.

### Small obstacles disappear

- Decrease `scanVoxelSize`.
- Decrease `quantileZ` or set `useSorting=false`.
- Increase `maxRelZ` if the obstacles are clipped by relative height.
- Disable `clearDyObs` temporarily to check whether dynamic clearing is
  suppressing them.

### Moving objects leave ghost obstacles

- Lower `decayTime`.
- Lower `voxelPointUpdateThre` and/or `voxelTimeUpdateThre` so pruning happens
  more often.
- Keep `clearDyObs=true`.
- If ghosts remain, widen `minDyObsVFOV`/`maxDyObsVFOV`, lower
  `minDyObsAngle`, or increase `absDyObsRelZThre`.

### Static obstacles are incorrectly removed

- Set `clearDyObs=false` to confirm dynamic clearing is the cause.
- Increase `minDyObsPointNum`.
- Increase `minDyObsAngle`.
- Narrow `minDyObsVFOV` and `maxDyObsVFOV`.
- Lower `absDyObsRelZThre`.

### Ground estimate is too high

- Use `useSorting=false`, or lower `quantileZ`.
- Enable `limitGroundLift=true`.
- Reduce `maxGroundLift`.
- Increase `minBlockPointNum` if isolated obstacle returns are corrupting the
  ground estimate.

### Terrain has holes or flickers

- Decrease `minBlockPointNum`.
- Increase `decayTime`.
- Increase `noDecayDis`.
- Decrease `scanVoxelSize`.
- Increase `disRatioZ` if holes appear mainly on slopes or at distance.

### Unknown areas should block planning

- Set `noDataObstacle=true`.
- Keep `noDataBlockSkipNum=0` for the most conservative behavior.
- Increase `noDataBlockSkipNum` if obstacle blocks appear along valid map edges.
- Remember that no-data obstacles activate only after traveling at least
  `noDecayDis` since startup or the last clear.

### Drops or negative obstacles are missing

- Keep `considerDrop=true`.
- Lower `minRelZ` so below-vehicle points are not clipped.
- Increase `disRatioZ` if drop points are farther away or on sloped terrain.

## Suggested Tuning Order

1. Set the useful vertical crop first: `minRelZ`, `maxRelZ`, and `disRatioZ`.
2. Tune density and stability: `scanVoxelSize`, `minBlockPointNum`,
   `decayTime`, `voxelPointUpdateThre`, and `voxelTimeUpdateThre`.
3. Tune ground estimation: `useSorting`, `quantileZ`, `limitGroundLift`, and
   `maxGroundLift`.
4. Tune dynamic-object clearing only after static terrain looks correct:
   `clearDyObs` and the `minDyObs*` / `maxDyObs*` parameters.
5. Enable and tune `noDataObstacle` only if the downstream planner needs
   unknown space to be represented as occupied.
