# Local Planner Configuration and Tuning Guide

This package launches two ROS 2 nodes:

- `localPlanner`: reads sensor/map data and chooses a short collision-free path from precomputed path sets.
- `pathFollower`: follows `/path` and publishes `/cmd_vel`.

Most parameters are set in `launch/local_planner.launch`. This Jazzy branch uses ROS 2 XML launch syntax and `rclcpp` nodes. Parameters are read into local variables when the nodes start, so changing them normally requires restarting the launch file; `ros2 param set` will not change planner behavior unless the code is extended with parameter update callbacks. Runtime behavior can still be changed through topics such as `/joy`, `/way_point`, `/speed`, `/check_obstacle`, and `/stop`.

## Data Flow

1. `/state_estimation` provides vehicle pose.
2. `localPlanner` crops obstacle data around the vehicle.
   - With `useTerrainAnalysis=true`, it subscribes to `/terrain_map` from `terrain_analysis`.
   - With `useTerrainAnalysis=false`, it uses `/registered_scan`.
3. `localPlanner` scores precomputed paths from `paths/` against obstacles, goal direction, joystick direction, and speed-scaled horizon settings.
4. The best path is published on `/path`. If no path is valid, a one-point path at `(0, 0, 0)` is published.
5. `pathFollower` tracks `/path` and publishes `/cmd_vel`.

## Launch Arguments

These can be overridden directly with `ros2 launch`, for example:

```bash
ros2 launch local_planner local_planner.launch maxSpeed:=1.2 autonomySpeed:=0.8 goalX:=5.0 goalY:=0.0
```

| Argument | Default | Effect | Tuning Notes |
| --- | ---: | --- | --- |
| `sensorOffsetX` | `0.0` | X offset between the sensor frame and vehicle frame. Used in odometry compensation and the `sensor` to `vehicle` static TF. | Tune first. Wrong values shift obstacles and paths relative to the robot. |
| `sensorOffsetY` | `0.0` | Y offset between the sensor frame and vehicle frame. | Same as `sensorOffsetX`. |
| `cameraOffsetZ` | `0.0` | Z offset for the `sensor` to `camera` static TF only. | Does not affect planner scoring directly. |
| `twoWayDrive` | `true` | Allows reverse driving and forward/reverse switching in the follower. | Set `false` for platforms that must not drive backward. |
| `maxSpeed` | `2.0` | Shared speed scale for both nodes. `autonomySpeed` and `/speed` are divided by this to produce normalized planning speed. | Keep consistent with the real platform limit. Lower this before tuning aggressive behavior. |
| `autonomyMode` | `true` | Initial mode. In autonomy, target direction comes from the goal and speed comes from `autonomySpeed` or `/speed`. Joystick axis 2 can toggle the mode. | Use `false` for manual testing with joystick direction. |
| `autonomySpeed` | `2.0` | Initial autonomous target speed in m/s, clamped by `maxSpeed`. | Start low, then raise after obstacle and steering behavior are stable. |
| `joyToSpeedDelay` | `2.0` | After joystick input, `/speed` is ignored for this many seconds. `/speed` only applies in autonomy mode while the joystick stick is idle. Used by both nodes. | Increase if manual override should hold longer. Decrease if autonomy should resume quickly. |
| `goalX` | `0.0` | Initial goal X in the odometry/world frame. Updated by `/way_point`. | Useful for launch-time tests. |
| `goalY` | `0.0` | Initial goal Y in the odometry/world frame. Updated by `/way_point`. | Useful for launch-time tests. |

## Terrain-Analysis Input

In this branch, `localPlanner` is configured for the terrain-analysis pipeline:

- `useTerrainAnalysis=true` in `launch/local_planner.launch`.
- The planner subscribes to `/terrain_map`.
- `terrain_analysis` publishes `/terrain_map` in the `map` frame.
- The point `intensity` is elevation difference from local ground. With the current `terrain_analysis` default `considerDrop=true`, this is an absolute difference, so both bumps and drops can become obstacles.

This means `obstacleHeightThre=0.2` should be read as: block paths through terrain points more than about `0.2 m` above or below local ground. Keep the planner on `/terrain_map` for normal local planning; `/terrain_map_ext` is an extended visualization/global map product and is not what this launch currently feeds to the planner.

Recommended starting point for terrain-cloud planning:

- Keep `useTerrainAnalysis=true`.
- Keep `terrainVoxelSize=0.2`, matching the terrain-analysis grid scale.
- Keep `useCost=false` until hard obstacle blocking is reliable.
- Tune `obstacleHeightThre` first. Lower it if real obstacles or drops are missed; raise it if traversable bumps, grass, or shallow dips block too often.
- Enable `useCost=true` only when you want mild terrain height differences to bias path selection without fully blocking the path. Then tune `groundHeightThre`, `costHeightThre`, and `costScore`.

## `localPlanner` Parameters

### Path Assets and Vehicle Model

| Parameter | Default | Effect | Tuning Notes |
| --- | ---: | --- | --- |
| `pathFolder` | `$(find-pkg-share local_planner)/paths` | Folder containing `startPaths.ply`, `paths.ply`, `pathList.ply`, and `correspondences.txt`. | Change only when using a different precomputed path set. |
| `vehicleLength` | `0.6` | Vehicle length used by `checkRotObstacle` to reject unsafe rotation directions near obstacles. | Match the physical footprint if `checkRotObstacle=true`. |
| `vehicleWidth` | `0.6` | Vehicle width used by `checkRotObstacle`. | Match the physical footprint if `checkRotObstacle=true`. |
| `sensorOffsetX` | launch arg | Corrects odometry from sensor pose to vehicle pose. | Must match the TF offset. |
| `sensorOffsetY` | launch arg | Corrects odometry from sensor pose to vehicle pose. | Must match the TF offset. |
| `twoWayDrive` | launch arg | Allows planning toward directions behind the vehicle. | Disable for forward-only vehicles. |

### Obstacle Source and Filtering

| Parameter | Default | Effect | Tuning Notes |
| --- | ---: | --- | --- |
| `laserVoxelSize` | `0.05` | Voxel filter size for `/registered_scan` when `useTerrainAnalysis=false`. | Larger reduces CPU and noise but can miss thin obstacles. Smaller is more precise and heavier. |
| `terrainVoxelSize` | `0.2` | Voxel filter size for `/terrain_map`; also used to sample `/navigation_boundary` edges. | `0.2` matches the fixed planar cell size used by `terrain_analysis`. Lower only if the terrain output and CPU budget support it. |
| `useTerrainAnalysis` | `true` | Selects `/terrain_map` instead of `/registered_scan`. Terrain intensity is treated as obstacle or traversability cost. | Keep `true` for this setup. Use `false` only for raw scan obstacle blocking without terrain analysis. |
| `checkObstacle` | `true` | Enables obstacle checks. Toggled immediately by joystick axis 5. The `/check_obstacle` topic only takes effect in autonomy mode and after `joyToCheckObstacleDelay`. | Turn off only for controlled debugging. |
| `checkRotObstacle` | `false` | Adds a footprint-based check for obstacles close to the vehicle during rotation. | Enable if the robot clips nearby objects while turning in place or reversing. |
| `adjacentRange` | `4.25` | Radius around the robot used to crop obstacle data. Also acts as the maximum planning horizon before speed scaling. | Increase for faster driving or earlier obstacle reaction. Decrease for CPU reduction or tight indoor behavior. |
| `minRelZ` | `-0.5` | Lower relative Z crop for raw scans when `useTerrainAnalysis=false`. | Raise to ignore ground returns below the vehicle. |
| `maxRelZ` | `0.25` | Upper relative Z crop for raw scans when `useTerrainAnalysis=false`. | Lower to ignore high returns; raise if real obstacles are being cut out. |

### Obstacle Height and Cost

| Parameter | Default | Effect | Tuning Notes |
| --- | ---: | --- | --- |
| `obstacleHeightThre` | `0.2` | Terrain points above this intensity/height block paths. With `terrain_analysis` and `considerDrop=true`, intensity is absolute height/drop difference from local ground. In raw-scan mode, all accepted points block regardless of intensity. | Lower is more conservative. Raise if traversable bumps, grass, or shallow drops are blocking too often. |
| `groundHeightThre` | `0.1` | Terrain points above this, but below `obstacleHeightThre`, can penalize paths when `useCost=true`. | Lower makes mild terrain affect scoring. Raise to ignore small roughness. |
| `costHeightThre` | `0.1` | Converts terrain cost height into a penalty: higher point intensity lowers path score. | Lower makes rough terrain expensive faster. Higher makes cost gentler. |
| `costScore` | `0.02` | Minimum score multiplier for costly but not blocking terrain. | Lower strongly avoids rough terrain. Higher allows rough paths if direction is good. |
| `useCost` | `false` | Keeps sub-obstacle terrain points and uses them as path penalties. If `false`, terrain input keeps only hard obstacles. | Enable when terrain intensity is reliable and you want preference, not just blocking. |
| `pointPerPathThre` | `2` | A path is blocked only after this many obstacle hits. Boundaries publish repeated points to meet this threshold. | Lower catches sparse obstacles but is sensitive to noise. Raise to reduce false positives but may pass close to obstacles. |

### Direction and Path Scoring

| Parameter | Default | Effect | Tuning Notes |
| --- | ---: | --- | --- |
| `dirWeight` | `0.02` | Penalizes selected paths whose endpoint direction differs from joystick/goal direction. The score becomes stricter as this increases. | Lower if the planner needs more freedom around obstacles. Raise if it wanders instead of heading toward the command/goal. |
| `dirThre` | `90.0` | Maximum allowed direction difference, in degrees, for candidate path rotations. | Lower for straighter behavior. Raise if the planner fails to find paths around obstacles or needs broader turning choices. |
| `dirToVehicle` | `false` | Changes how `dirThre` is interpreted: relative to goal/joystick direction when `false`, relative to vehicle heading sectors when `true`. | Keep `false` for goal/joystick-driven behavior. Try `true` if you need a fixed vehicle-forward search cone. |

### Planning Horizon and Path Scale

| Parameter | Default | Effect | Tuning Notes |
| --- | ---: | --- | --- |
| `pathScale` | `1.25` | Scales the precomputed path geometry before publishing and collision checking. | Larger gives longer/wider path shapes and is useful at speed. Smaller helps in tight spaces. |
| `minPathScale` | `0.75` | Minimum scale used when speed scaling or fallback shrinking reduces `pathScale`. | Lower if the robot should squeeze through tight areas. Keep realistic for vehicle capability. |
| `pathScaleStep` | `0.25` | If no path is found, the planner reduces scale by this amount before reducing range. | Smaller gives finer fallback search but more computation. |
| `pathScaleBySpeed` | `true` | Uses `pathScale * normalized_speed`, clamped by `minPathScale`. | Good default. Disable for constant path shape independent of speed. |
| `minPathRange` | `1.0` | Minimum planning horizon. | Raise if the robot reacts too late at low speed. Lower if it cannot find short maneuvers in clutter. |
| `pathRangeStep` | `0.5` | If scale fallback fails, the planner shortens the horizon by this amount. | Smaller gives finer fallback. Larger exits blocked cases faster. |
| `pathRangeBySpeed` | `true` | Uses `adjacentRange * normalized_speed`, clamped by `minPathRange`. | Good for slow-speed tight maneuvers. Disable if you always want full horizon. |
| `pathCropByGoal` | `true` | Obstacle checking and free-path visualization are cropped near the goal plus `goalClearRange`. | Keep `true` for waypoint navigation. Disable if obstacles beyond the goal still need to influence the local path. |

### Autonomy and Goal Handling

| Parameter | Default | Effect | Tuning Notes |
| --- | ---: | --- | --- |
| `autonomyMode` | launch arg | In autonomy, `joyDir` is computed from vehicle-to-goal direction. | Joystick can still switch modes. |
| `autonomySpeed` | launch arg | Initial autonomy speed. | Set below `maxSpeed`; start conservatively. |
| `joyToSpeedDelay` | launch arg | Delay before `/speed` can replace joystick speed. | Same as launch argument. |
| `joyToCheckObstacleDelay` | `5.0` | Delay before `/check_obstacle` can replace joystick obstacle-check state. | Increase to prioritize recent manual safety choices. |
| `goalClearRange` | `0.5` | Extra distance beyond the goal used for obstacle checking when `pathCropByGoal=true`. | Increase if the final approach cuts too close to obstacles beyond the goal. Decrease if obstacles past the goal over-constrain the final approach. |
| `goalX` | launch arg | Initial goal X. | Updated by `/way_point`. |
| `goalY` | launch arg | Initial goal Y. | Updated by `/way_point`. |

## `pathFollower` Parameters

### Command Output and Geometry

| Parameter | Default | Effect | Tuning Notes |
| --- | ---: | --- | --- |
| `sensorOffsetX` | launch arg | Same vehicle pose correction as the planner. | Must match `localPlanner`. |
| `sensorOffsetY` | launch arg | Same vehicle pose correction as the planner. | Must match `localPlanner`. |
| `pubSkipNum` | `1` | Publishes every `pubSkipNum + 1` control loops. The loop runs at 100 Hz, so `1` gives about 50 Hz. | Use `0` for 100 Hz commands. Raise only if downstream command consumers need lower rate. |
| `twoWayDrive` | launch arg | Allows the follower to reverse when the path direction is more than 90 deg away. | Disable for forward-only vehicles. |

### Steering and Speed Control

| Parameter | Default | Effect | Tuning Notes |
| --- | ---: | --- | --- |
| `lookAheadDis` | `0.5` | Distance to advance along the path before choosing the tracking target. | Larger is smoother but cuts corners. Smaller tracks tightly but can oscillate. |
| `yawRateGain` | `7.5` | Proportional gain from heading error to yaw rate while moving. | Lower if oscillating. Raise if steering lags. |
| `stopYawRateGain` | `7.5` | Yaw gain used when current speed is near zero. | Raise for quicker in-place alignment; lower if it snaps too hard. |
| `maxYawRate` | `90.0` | Yaw-rate cap in deg/s. | Lower for smoother rotation. Raise only if the base can safely turn faster. |
| `maxSpeed` | launch arg | Maximum linear command scale in m/s. | Keep the same value as `localPlanner`. |
| `maxAccel` | `2.5` | Linear acceleration and deceleration limit in m/s^2, applied in the 100 Hz loop. | Lower for smoother starts/stops. Raise for more responsive speed changes. |
| `switchTimeThre` | `1.0` | Minimum seconds between forward/reverse direction switches. | Increase if the robot chatters between forward and reverse. Decrease if reversing feels delayed. |
| `dirDiffThre` | `0.1` | Heading-error threshold in radians required before accelerating. `0.1` rad is about 5.7 deg. | Raise if the robot never accelerates because alignment is too strict. Lower if it starts moving before it points along the path. |
| `stopDisThre` | `0.2` | Distance threshold near the tracking point where the follower stops accelerating; also used with `noRotAtGoal`. | Raise to stop earlier. Lower to approach closer, but watch for creep. |
| `slowDwnDisThre` | `0.85` | Distance from the path end where speed starts tapering down. | Larger starts slowing earlier. Smaller keeps speed until closer to the endpoint. |

### Inclination and Safety Behavior

| Parameter | Default | Effect | Tuning Notes |
| --- | ---: | --- | --- |
| `useInclRateToSlow` | `false` | Slows the robot after high roll/pitch angular velocity. | Enable on rough terrain if impacts or rocking should trigger a temporary slow-down. |
| `inclRateThre` | `120.0` | Roll/pitch angular velocity threshold in deg/s for slow-down. | Lower to trigger more often. Raise to avoid nuisance slow-downs. |
| `slowRate1` | `0.25` | First speed multiplier after an inclination-rate event. | Lower means stronger initial slow-down. |
| `slowRate2` | `0.5` | Second speed multiplier after `slowTime1`. | Lower means longer conservative recovery. |
| `slowTime1` | `2.0` | Duration of `slowRate1` in seconds. | Increase for longer initial slow-down. |
| `slowTime2` | `2.0` | Duration of `slowRate2` in seconds. | Increase for longer recovery. |
| `useInclToStop` | `false` | Stops the robot after roll or pitch exceeds `inclThre`. | Enable if tip risk is more important than continuous motion. |
| `inclThre` | `45.0` | Roll/pitch angle threshold in degrees for stop behavior. | Lower for conservative tip protection. |
| `stopTime` | `5.0` | Stop duration after an inclination stop event. | Increase to give the platform more time to settle. |
| `noRotAtStop` | `false` | In manual mode, prevents joystick yaw command when speed is zero. | Enable if in-place rotation is unsafe. |
| `noRotAtGoal` | `true` | Suppresses yaw command when the final path point is reached. | Disable if the robot should keep rotating to align at the goal. |
| `autonomyMode` | launch arg | Initial autonomy mode. | Must match intended behavior in `localPlanner`. |
| `autonomySpeed` | launch arg | Initial autonomous speed. | Same speed source as planner. |
| `joyToSpeedDelay` | launch arg | Delay before `/speed` can override joystick speed. | Same as planner. |

The follower also listens to `/stop` as `std_msgs/Int8`: values `>=1` force linear speed to zero, and values `>=2` also force yaw rate to zero.

## Recommended Tuning Order

1. Verify frames and offsets.
   - Confirm `/state_estimation` and the `sensor`, `vehicle`, and `camera` TF frames agree in RViz.
   - Tune `sensorOffsetX`, `sensorOffsetY`, and `cameraOffsetZ` before planner behavior.

2. Tune speed in a clear area.
   - Start with low `maxSpeed` and `autonomySpeed`, for example `0.5` to `1.0`.
   - Tune `lookAheadDis`, `yawRateGain`, `maxYawRate`, and `maxAccel` until following is smooth.

3. Tune obstacle detection.
   - Keep `useTerrainAnalysis=true`; the planner reads `/terrain_map` from `terrain_analysis`.
   - Adjust `obstacleHeightThre`, `groundHeightThre`, `useCost`, and voxel sizes while watching `/terrain_map` and `/free_paths`.
   - Use `pointPerPathThre` to balance noise rejection against obstacle sensitivity.

4. Tune planning horizon.
   - Increase `adjacentRange`, `pathRangeBySpeed`, and `minPathRange` for earlier decisions at speed.
   - Reduce `pathScale` or `minPathScale` if the robot cannot find maneuvers in tight spaces.

5. Tune direction preference.
   - Increase `dirWeight` or decrease `dirThre` if the planner wanders.
   - Decrease `dirWeight` or increase `dirThre` if it cannot route around obstacles.

6. Re-test goals and overrides.
   - Test `/way_point`, `/speed`, joystick mode switching, `/check_obstacle`, and `/stop`.
   - Keep `localPlanner` and `pathFollower` speed and drive-mode parameters synchronized.

## Common Symptoms

| Symptom | Likely Parameters |
| --- | --- |
| Obstacles appear shifted from the robot | `sensorOffsetX`, `sensorOffsetY`, TF setup |
| Planner publishes a one-point path and stops | In terrain mode: `obstacleHeightThre`, `pointPerPathThre`, `dirThre`, `pathScale`, `minPathScale`, `pathCropByGoal`, and whether `/terrain_map` is too dense or empty. In raw-scan mode also check local planner `minRelZ` and `maxRelZ`. |
| Planner is too conservative around grass or bumps | Raise `obstacleHeightThre`; raise `groundHeightThre`; disable `useCost`; raise `costScore`; raise `pointPerPathThre` |
| Robot passes too close to obstacles | Lower `obstacleHeightThre`; lower voxel sizes; lower `pointPerPathThre`; enable `checkRotObstacle`; verify path assets match vehicle size |
| Robot oscillates while following path | Increase `lookAheadDis`; lower `yawRateGain`; lower `maxYawRate`; lower `maxAccel` |
| Robot turns too slowly | Raise `yawRateGain`, `stopYawRateGain`, or `maxYawRate` |
| Robot never accelerates after turning | Increase `dirDiffThre` slightly |
| Robot starts moving before aligned | Decrease `dirDiffThre` |
| Robot chatters between forward and reverse | Increase `switchTimeThre`; consider `twoWayDrive=false` |
| Robot stops too early near the goal | Lower `stopDisThre`; lower `slowDwnDisThre`; tune `goalClearRange` |
| Robot drives too fast near the goal | Raise `slowDwnDisThre`; lower `autonomySpeed`; lower `maxAccel` |

## Quick Profiles

Place each `param` under the node that owns it: planner parameters under `localPlanner`, follower parameters such as `lookAheadDis` under `pathFollower`, and `arg` values near the top of the launch file. The Jazzy launch file uses ROS 2 XML `param` tags with inferred types, so the examples below match that style.

### Conservative Indoor Start

```xml
<arg name="maxSpeed" default="0.8"/>
<arg name="autonomySpeed" default="0.5"/>
<param name="adjacentRange" value="3.0" />
<param name="pathScale" value="1.0" />
<param name="minPathScale" value="0.75" />
<param name="obstacleHeightThre" value="0.10" />
<param name="checkRotObstacle" value="true" />
```

### Faster Open Area

```xml
<arg name="maxSpeed" default="2.0"/>
<arg name="autonomySpeed" default="1.5"/>
<param name="adjacentRange" value="5.0" />
<param name="pathScale" value="1.25" />
<param name="minPathRange" value="1.5" />
<param name="lookAheadDis" value="0.7" />
```

### Rough Terrain Preference

```xml
<param name="useTerrainAnalysis" value="true" />
<param name="useCost" value="true" />
<param name="groundHeightThre" value="0.05" />
<param name="obstacleHeightThre" value="0.18" />
<param name="costHeightThre" value="0.12" />
<param name="costScore" value="0.01" />
```
