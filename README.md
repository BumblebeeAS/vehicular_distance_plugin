# Vehicular Distance Plugin

Gazebo Sim system plugin that monitors the distance from a point fixed in a
vehicle's FLU frame to a target model. After the vehicle remains within the
configured threshold, it publishes a `gz.msgs.StringMsg` containing
`activate`.

## Build and load

Build the package after changing its C++ files, then source the workspace:

```bash
colcon build --packages-select vehicular_distance_plugin --symlink-install
source install/setup.bash
```

Use the following plugin declaration in an SDF model or world:

```xml
<plugin
  filename="libDistance_SensorSystem.so"
  name="Distance_Sensor::Distance_SensorSystem">
  ...
</plugin>
```

The package environment hook adds the installed `lib` directory to
`GZ_SIM_SYSTEM_PLUGIN_PATH` after the workspace is sourced.

## SDF parameters

| Parameter | Required | Description |
| --- | --- | --- |
| `vehicle_name` | Yes | Name of the monitored vehicle model. |
| `target_name` | Yes | Name of the target model. It must differ from `vehicle_name`. |
| `distance_threshold` | Yes | Distance in metres. Distances strictly less than this value are below the threshold; equality is above. |
| `gz_publish_topic` | Yes | Gazebo Transport topic on which the plugin publishes `activate`. |
| `minimum_time_within_threshold` | No | Continuous time in seconds required below the threshold before publishing. Defaults to `0.0`. Simulation pauses do not count. |
| `vehicle_offset_x` | No | Vehicle-local forward (`+X`) offset in metres. Defaults to `0.0`. |
| `vehicle_offset_y` | No | Vehicle-local left (`+Y`) offset in metres. Defaults to `0.0`. |
| `vehicle_offset_z` | No | Vehicle-local up (`+Z`) offset in metres. Defaults to `0.0`. |

The legacy `magnet_positionx`, `magnet_positiony`, and `magnet_positionz`
offset fields are also accepted when the corresponding `vehicle_offset_*` field
is absent.

## Behavior

1. The plugin locates the named vehicle and target models, waiting if either is
   created after the plugin is configured.
2. While unpaused, it calculates the vehicle offset in world
   coordinates using the vehicle orientation, then measures the Euclidean
   distance to the target model origin.
3. Below the threshold, the dwell timer starts. Returning to or above the
   threshold resets that timer.
4. Once the vehicle has remained below the threshold for
   `minimum_time_within_threshold`, the plugin publishes `activate` once.
5. The detector remains latched while the vehicle stays within the threshold.
   Leaving the threshold rearms it for a future entry.

## Pipe LED example

If the AUV stays within `0.3 m` of LED 1 for one simulation second, this
configuration publishes `activate` on the pipe's proximity topic.

```xml
<plugin
  filename="libDistance_SensorSystem.so"
  name="Distance_Sensor::Distance_SensorSystem">
  <vehicle_name>auv4</vehicle_name>
  <target_name>pipe_led_1</target_name>

  <!-- FLU offset from the AUV model origin -->
  <vehicle_offset_x>0.0</vehicle_offset_x>
  <vehicle_offset_y>0.0</vehicle_offset_y>
  <vehicle_offset_z>-0.5</vehicle_offset_z>

  <distance_threshold>0.3</distance_threshold>
  <minimum_time_within_threshold>1.0</minimum_time_within_threshold>

  <gz_publish_topic>/robotx/pipe_led/1/activate</gz_publish_topic>
</plugin>
```

Observe detections with Gazebo Transport:

```bash
gz topic -e -t /robotx/pipe_led/1/activate
```
