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


## Part 2 implementation

The path planner generates weed positions and calculates a feasible route for a forward-only non-holonomic robot using Dubins paths.

### Approach

- The working area is modeled as a square of 1000 m².
- 500 initial weed positions are generated uniformly.
- 50% of the initial weeds are selected as cluster centers.
- Each cluster adds between 2 and 10 plants using a normal distribution with a standard deviation of 3 m.
- A fixed random seed (`42`) is used to make the result reproducible.
- The robot starts from a random position and orientation.
- The minimum turning radius is 2 m.

The waypoint ordering uses a greedy hybrid heuristic. The 10 nearest unvisited weeds are first selected using Euclidean distance. The planner then evaluates Dubins connections to those candidates and chooses the shortest valid one.

Eight possible terminal orientations are evaluated for each candidate. A connection is accepted only if:

- Its complete sampled trajectory remains inside the field.
- At least 2 m of forward clearance is available at the terminal pose.
- At least one complete left or right turning circle remains inside the field.

If none of the 10 nearest candidates is reachable, all remaining weeds are checked before stopping the planner.

### Assumptions and limitations

- The field is treated as a flat local Cartesian plane.
- Weeds are objectives and not obstacles.
- Clusters are resampled when a generated plant falls outside the field.
- Boundary validation uses trajectory samples separated by at most 0.25 m.
- Terminal orientation is not specified by the weed, so eight discrete headings are tested.
- The greedy heuristic provides a fast feasible route, but it does not guarantee a globally optimal  route.
- A weed is omitted when no valid sampled Dubins connection and safe terminal orientation can be found.

### Run

From the ROS 2 workspace:

```bash
source install/setup.bash
ros2 run paltech_assignment path_planning
```

The visualization is saved as `path_planning_result.png` in the current working directory.

For the fixed random seed (`42`), the planner generated 1893 weeds, visited 1878, and omitted 15. The observed execution time was approximately 6.3 seconds on the development system; execution time may vary depending on the hardware.