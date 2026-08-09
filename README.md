# Vehicular Distance Plugin

Gazebo Sim system plugin that monitors the distance from a point fixed in a
vehicle's FLU frame to a target model. It publishes `gz.msgs.StringMsg` values
for the below-threshold and above-threshold states. Monitoring can start
immediately or be gated by a Gazebo Transport topic.

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
| `output_topic` | No | Gazebo Transport topic for `gz.msgs.StringMsg`. Empty or omitted disables the plugin. |
| `below_threshold_message` | When enabled | String sent after the vehicle remains below the threshold for the configured dwell time. |
| `above_threshold_message` | When enabled | String sent when an active vehicle is at or above the threshold. |
| `minimum_time_within_threshold` | No | Continuous time in seconds required below the threshold before sending the below message. Defaults to `0.0`. Simulation pauses do not count. |
| `vehicle_offset_x` | No | Vehicle-local forward (`+X`) offset in metres. Defaults to `0.0`. |
| `vehicle_offset_y` | No | Vehicle-local left (`+Y`) offset in metres. Defaults to `0.0`. |
| `vehicle_offset_z` | No | Vehicle-local up (`+Z`) offset in metres. Defaults to `0.0`. |
| `gz_subscribe_topic` | No | Gazebo Transport topic used to activate the plugin. Empty or omitted means the plugin starts active. |
| `trigger_condition` | Required when `gz_subscribe_topic` is set | Exact `StringMsg.data()` value that activates monitoring. |

The legacy `magnet_positionx`, `magnet_positiony`, and `magnet_positionz`
offset fields are also accepted when the corresponding `vehicle_offset_*` field
is absent.

## Behavior

1. The plugin locates the named vehicle and target models, waiting if either is
   created after the plugin is configured.
2. While active and unpaused, it calculates the vehicle offset in world
   coordinates using the vehicle orientation, then measures the Euclidean
   distance to the target model origin.
3. At or above the distance threshold, it sends the above-threshold message
   once and continues monitoring.
4. Below the threshold, the dwell timer starts. Returning to or above the
   threshold resets that timer.
5. Once the vehicle has remained below the threshold for
   `minimum_time_within_threshold`, the plugin sends the below-threshold
   message once for that activation cycle.

With an activation topic, a message whose data does not match
`trigger_condition` deactivates and rearms the plugin. This makes it suitable
for a shared LED-control topic: for example, `steady_red` activates checking,
and a published `steady_green` result deactivates the instance again.

## Pipe LED example

This configuration makes `steady_red` activate distance monitoring for LED 1.
If the AUV stays within `0.3 m` for one simulation second, the plugin switches
the LED to green. Otherwise it keeps the LED red while continuing to monitor.

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

  <gz_subscribe_topic>/led_pipe_led_1/change_led_mode</gz_subscribe_topic>
  <trigger_condition>steady_red</trigger_condition>
  <output_topic>/led_pipe_led_1/change_led_mode</output_topic>
  <below_threshold_message>steady_green</below_threshold_message>
  <above_threshold_message>steady_red</above_threshold_message>
</plugin>
```

Activate it with Gazebo Transport:

```bash
gz topic -t /led_pipe_led_1/change_led_mode \
  -m gz.msgs.StringMsg \
  -p "data: 'steady_red'"
```
