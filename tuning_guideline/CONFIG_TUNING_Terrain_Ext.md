# Terrain Analysis Ext Parameter Tuning Guide

This package runs the `terrainAnalysisExt` ROS 2 node from
`launch/terrain_analysis_ext.launch`. It extends the local terrain map by
building a larger rolling terrain map from registered scans, optionally removing
disconnected or ceiling-like terrain, and then merging the original local
`/terrain_map` back into the near-field radius.

The node subscribes to:

- `/state_estimation` (`nav_msgs/msg/Odometry`)
- `/registered_scan` (`sensor_msgs/msg/PointCloud2`)
- `/terrain_map` (`sensor_msgs/msg/PointCloud2`) as the trusted local terrain map
- `/joy` (`sensor_msgs/msg/Joy`) for manual map clearing
- `/cloud_clearing` (`std_msgs/msg/Float32`) for commanded map clearing

It publishes `/terrain_map_ext` (`sensor_msgs/msg/PointCloud2`) in the `map`
frame. For points computed by this node, intensity is the absolute elevation
difference from the estimated local ground. Points copied from `/terrain_map`
inside `localTerrainMapRadius` keep their incoming intensity.

The parameters are read once at node startup. Changing a parameter with
`ros2 param set` while the node is already running will not change behavior
unless the code is extended with a parameter update callback. For tuning, edit
`launch/terrain_analysis_ext.launch` or pass startup parameters, then restart
the node.

## Processing Overview

At a high level, the node:

1. Crops incoming registered scan points by distance and relative height around
   the vehicle.
2. Stores recent points in a large rolling terrain voxel map centered on the
   vehicle.
3. Periodically downsamples and prunes old points from each terrain voxel.
4. Estimates a ground height for each local planar grid cell.
5. Optionally flood-fills terrain connectivity from the vehicle cell to reject
   disconnected or ceiling-like cells.
6. Publishes extended terrain points outside `localTerrainMapRadius`.
7. Merges the incoming `/terrain_map` inside `localTerrainMapRadius`.

Important fixed code constants:

- Terrain storage voxels are `2.0 m` cells in a `41 x 41` rolling map.
- Registered scan intake keeps points within about `42 m` horizontally.
- The extension stage uses the central `21 x 21` terrain voxel area.
- Planar elevation cells are `0.4 m` cells in a `101 x 101` local grid.
- Terrain connectivity expands through neighbors within `10` planar cells,
  equivalent to about `4.0 m` with the fixed planar voxel size.

## Parameter Reference

The `Launch value` column lists the value set in
`launch/terrain_analysis_ext.launch`, which is what the node uses when started
through the launch file. A few compiled-in code defaults differ from the launch
file (`quantileZ` is `0.25`, `lowerBoundZ` is `-1.5`, and `checkTerrainConn` is
`true` in the source), so running the bare `ros2 run terrainAnalysisExt` without
passing parameters will not match these defaults.

| Parameter | Launch value | Unit | Effect | Tuning guidance |
| --- | ---: | --- | --- | --- |
| `scanVoxelSize` | `0.1` | m | PCL voxel-grid leaf size used when downsampling stored scan points. | Lower keeps more detail but costs more CPU and memory. Higher smooths the map and improves speed, but can erase small terrain features. |
| `decayTime` | `10.0` | s | Stored points older than this are removed when their terrain voxel updates, except points within `noDecayDis`. | Lower removes stale obstacles faster. Higher keeps long-range terrain more stable in sparse scans. |
| `noDecayDis` | `0.0` | m | Points inside this horizontal radius around the vehicle are kept regardless of age. | `0.0` means all stored points can decay by time. Increase only if nearby extension points flicker and stale points are acceptable. |
| `clearingDis` | `30.0` | m | Radius removed from the stored extended map when clearing is triggered. `/cloud_clearing` overrides this value with the message data for that clear. | Increase to clear a wider extended map. Decrease for targeted cleanup. |
| `useSorting` | `false` | bool | If true, ground height is selected from a sorted height quantile. If false, ground height is the minimum height sample in each cell. | Enable for noisy scans, vegetation, or mixed returns. Disable when the lowest return is the most reliable ground cue. |
| `quantileZ` | `0.1` | ratio | Ground-height quantile used only when `useSorting=true`. Values are clamped to the available sorted sample index. | Lower values behave closer to minimum height. Higher values raise the estimated ground and can reduce false obstacle height, but may hide low obstacles. |
| `vehicleHeight` | `1.5` | m | Maximum accepted absolute elevation difference from estimated ground for extended terrain points. | Set to the useful vertical obstacle/terrain band for the robot. Lower clips tall structures; higher keeps more vertical clutter. |
| `voxelPointUpdateThre` | `100` | points | A terrain voxel is downsampled and pruned after this many new points are added. | Lower updates cleanup more often but costs more CPU. Higher reduces CPU but can delay decay and downsampling. |
| `voxelTimeUpdateThre` | `2.0` | s | A terrain voxel is downsampled and pruned after this much time since its last update. | Lower removes stale points faster. Higher reduces CPU and keeps points longer. |
| `lowerBoundZ` | `-2.5` | m | Minimum allowed point height relative to the vehicle. The crop expands this lower bound by `disRatioZ * distance`. | Lower to keep downhill terrain or drops. Raise to reject low outliers. |
| `upperBoundZ` | `1.0` | m | Maximum allowed point height relative to the vehicle. The crop expands this upper bound by `disRatioZ * distance`. | Raise for uphill terrain or taller obstacles. Lower to reject walls, overhangs, vegetation, or ceiling returns. |
| `disRatioZ` | `0.1` | m/m | Distance-based expansion of the relative-height crop during scan intake, pruning, ground estimation, and output. | Increase for slopes, pose error, or far-range vertical uncertainty. Decrease to reject more far-range clutter. |
| `checkTerrainConn` | `false` | bool | Enables connectivity filtering from the vehicle cell. Only connected cells are published, while large elevation jumps can be treated as ceiling/disconnected terrain. | Enable when ceilings, overpasses, or isolated high surfaces appear in `/terrain_map_ext`. Disable if valid disconnected terrain is being removed. |
| `terrainConnThre` | `0.5` | m | Maximum ground-height difference allowed between connected terrain cells during connectivity expansion. | Increase for rougher terrain or ramps. Decrease to make connectivity stricter and reject abrupt transitions. |
| `terrainUnderVehicle` | `-0.75` | m | Fallback ground height below the vehicle if the center planar cell has no samples during connectivity checking. | Make more negative if the vehicle center often has no returns and nearby terrain is lower. Make less negative if connectivity spreads from an unrealistically low seed. |
| `ceilingFilteringThre` | `2.0` | m | Elevation jump above which an unconnected neighboring cell is marked as ceiling-like during connectivity checking. | Lower to reject overhead structures more aggressively. Raise if high but valid terrain is being filtered. |
| `localTerrainMapRadius` | `4.0` | m | Radius around the vehicle where incoming `/terrain_map` is copied directly into `/terrain_map_ext`; this node computes only points outside this radius. | Increase to trust the local terrain-analysis map over a larger near-field area. Decrease to let the extended node replace more near-field terrain. |

## How To Tune

Start with the launch defaults, record or replay a representative bag, and view
`/terrain_map_ext` colored by intensity. Compare it with `/terrain_map` so you
can tell whether a problem comes from the local input map or from the extension
logic. Change one parameter group at a time and restart the node after each
change.

Typical startup:

```bash
# From the ROS 2 workspace root:
colcon build --packages-select terrain_analysis_ext
source install/setup.bash
ros2 launch terrain_analysis_ext terrain_analysis_ext.launch
```

If you edit `launch/terrain_analysis_ext.launch` and launch the installed
package, rebuild the package before retesting. During heavy tuning, building the
workspace with `colcon build --symlink-install` avoids this extra copy step for
launch-file edits.

Enable terrain connectivity from the launch argument:

```bash
ros2 launch terrain_analysis_ext terrain_analysis_ext.launch checkTerrainConn:=true
```

For quick one-off parameter tests without editing the launch file:

```bash
ros2 run terrain_analysis_ext terrainAnalysisExt --ros-args -p scanVoxelSize:=0.15 -p decayTime:=5.0
```

Manual clearing:

- Press joystick button index `5` on `/joy` to clear points within
  `clearingDis`.
- Publish a custom clearing radius on `/cloud_clearing`:

```bash
ros2 topic pub --once /cloud_clearing std_msgs/msg/Float32 "{data: 20.0}"
```

## Symptom-Based Adjustments

### Extended map is noisy or too dense

- Increase `scanVoxelSize`.
- Lower `upperBoundZ` or `disRatioZ` if high clutter is entering the map.
- Enable `checkTerrainConn` if the noise is disconnected from traversable
  terrain.
- Use `useSorting=true` with `quantileZ=0.1` to `0.25` if mixed returns distort
  the ground estimate.

### Long-range terrain has holes or flickers

- Increase `decayTime`.
- Increase `voxelTimeUpdateThre` only if CPU load is more important than quick
  cleanup.
- Decrease `scanVoxelSize` if sparse terrain detail is being downsampled away.
- Increase `disRatioZ` if holes appear mainly on slopes or at distance.
- Decrease `localTerrainMapRadius` only if the holes are in the overlap between
  `/terrain_map` and this extended output.

### Stale objects remain too long

- Lower `decayTime`.
- Lower `voxelPointUpdateThre` and/or `voxelTimeUpdateThre` so pruning happens
  more often.
- Use `/cloud_clearing` with an appropriate radius for immediate cleanup.
- Keep `noDecayDis` at `0.0` unless there is a strong reason to preserve nearby
  stale points.

### Ceiling, overpass, or high disconnected surfaces appear

- Set `checkTerrainConn=true`.
- Lower `ceilingFilteringThre` for more aggressive ceiling rejection.
- Lower `terrainConnThre` if connectivity is crossing abrupt height jumps.
- Lower `upperBoundZ` or `disRatioZ` if overhead returns are entering before
  connectivity filtering.

### Valid ramps or rough terrain are removed

- Increase `terrainConnThre`.
- Raise `ceilingFilteringThre`.
- Disable `checkTerrainConn` temporarily to confirm connectivity filtering is
  the cause.
- Increase `disRatioZ` or widen `lowerBoundZ`/`upperBoundZ` if the terrain is
  clipped before connectivity is checked.

### Near-field output does not match the base terrain map

- Check that `/terrain_map` is being published by the local terrain-analysis
  node.
- Increase `localTerrainMapRadius` to copy more of `/terrain_map` directly into
  `/terrain_map_ext`.
- Remember that points copied from `/terrain_map` keep their original intensity,
  while points computed by this node use absolute ground-relative elevation.

### Small terrain features disappear

- Decrease `scanVoxelSize`.
- Set `useSorting=false` or lower `quantileZ`.
- Increase `upperBoundZ` if the features are clipped by the relative-height
  filter.
- Increase `vehicleHeight` if features are removed after ground estimation.

## Suggested Tuning Order

1. Tune the vertical crop first: `lowerBoundZ`, `upperBoundZ`, and `disRatioZ`.
2. Tune map persistence and density: `scanVoxelSize`, `decayTime`,
   `voxelPointUpdateThre`, and `voxelTimeUpdateThre`.
3. Tune ground estimation: `useSorting` and `quantileZ`.
4. Set the handoff between local and extended terrain with
   `localTerrainMapRadius`.
5. Enable and tune terrain connectivity only after the basic extended map looks
   reasonable: `checkTerrainConn`, `terrainConnThre`, `terrainUnderVehicle`,
   and `ceilingFilteringThre`.
