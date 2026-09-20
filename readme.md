## Part 1 implementation

The waypoint manager:

- Loads and validates Point features from a GeoJSON `FeatureCollection`.
- Converts geographic coordinates to a local robot frame.
- Computes each waypoint orientation toward the following waypoint.
- Exposes services to load, retrieve, and reset waypoints.
- Saves a PNG visualization of the converted path.

### Build and run

From the ROS 2 workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select custom_interfaces paltech_assignment
source install/setup.bash
ros2 run paltech_assignment waypoint_manager
```

### Services

Load a GeoJSON file:

```bash
ros2 service call /set_waypoints custom_interfaces/srv/SetWaypoints \
"{file_path: '$HOME/ros2_ws/src/paltech_assignment/paltech_assignment/waypoints/waypoints.geojson'}"
```

Retrieve the converted waypoints:

```bash
ros2 service call /get_robot_waypoints custom_interfaces/srv/GetWaypoints "{}"
```

Reset all loaded waypoints:

```bash
ros2 service call /reset_waypoints std_srvs/srv/Trigger "{}"
```

The plot is saved as `waypoints_robot_frame.png` in the directory from which the node is launched.

### Assumptions

- GeoJSON coordinates follow the standard `[longitude, latitude]` order.
- All features must contain `Point` geometries with valid geographic ranges.
- A local equirectangular approximation is used with an Earth radius of
  `6,371,000 m`; it is intended for short distances around the robot origin.
- Local `x` points east and local `y` points north.
- The provided `Waypoint` message is reused for local coordinates:
  `longitude` stores `x`, `latitude` stores `y`, and `orientation` stores `yaw` in radians.
- Each yaw points toward the following waypoint. The final waypoint inherits the previous yaw because no following point exists.