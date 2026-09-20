import rclpy
from rclpy.node import Node
from custom_interfaces.srv import GetWaypoints, SetWaypoints
from custom_interfaces.msg import Waypoint
from std_srvs.srv import Trigger
import json

from pathlib import Path
import math


class WaypointManager(Node):
    def __init__(self):
        super().__init__("waypoint_manager")
        self.get_waypoints_srv = self.create_service(
            GetWaypoints, "get_robot_waypoints", self.get_robot_waypoints_callback
        )
        self.set_waypoint_srv = self.create_service(
            SetWaypoints, "set_waypoints", self.set_waypoints_callback
        )
        self.reset_waypoint_srv = self.create_service(
            Trigger, "reset_waypoints", self.reset_waypoints_callback
        )
        self.robot_initial_geo = (47.740114, 10.322442)

        self.waypoint_list_geo = []  # needs to be an array of Waypoint() messages
        self.waypoint_list_robot_frame = []
        self.set_waypoint_client = self.create_client(SetWaypoints, "set_waypoints")

    def set_waypoints_callback(self, request, response):
        response.success = False

        try:
            file_path = Path(request.file_path).expanduser()

            with file_path.open("r", encoding="utf-8") as geojson_file:
                geojson_data = json.load(geojson_file)

            if geojson_data.get("type") != "FeatureCollection":
                raise ValueError("The GeoJSON root must be a FeatureCollection")

            features = geojson_data.get("features")

            if not isinstance(features, list) or not features:
                raise ValueError("The GeoJSON must contain at least one feature")

            loaded_waypoints = []
            for index, feature in enumerate(features):
                if not isinstance(feature, dict):
                    raise ValueError(f"Feature {index} is invalid")

                geometry = feature.get("geometry")

                if not isinstance(geometry, dict) or geometry.get("type") != "Point":
                    raise ValueError(f"Feature {index} is not a Point")

                coordinates = geometry.get("coordinates")

                if not isinstance(coordinates, list) or len(coordinates) < 2:
                    raise ValueError(f"Feature {index} has invalid coordinates")

                longitude = float(coordinates[0])
                latitude = float(coordinates[1])

                if not -180.0 <= longitude <= 180.0:
                    raise ValueError(f"Feature {index} has an invalid longitude")

                if not -90.0 <= latitude <= 90.0:
                    raise ValueError(f"Feature {index} has an invalid latitude")

                waypoint = Waypoint()
                waypoint.latitude = latitude
                waypoint.longitude = longitude
                waypoint.orientation = 0.0

                loaded_waypoints.append(waypoint)

            self.waypoint_list_geo = loaded_waypoints
            self.convert_waypoints_to_robot_frame()

            self.get_logger().info(
                f"Loaded {len(self.waypoint_list_geo)} geographic waypoints"
            )

            response.success = True

        except (OSError, json.JSONDecodeError, TypeError, ValueError) as error:
            self.get_logger().error(f"Could not load GeoJSON: {error}")

        return response

    def convert_waypoints_to_robot_frame(self):
        if not self.waypoint_list_geo:
            self.waypoint_list_robot_frame = []
            self.get_logger().warning("No geographic waypoints available to convert")
            return

        earth_radius = 6371000.0

        origin_latitude, origin_longitude = self.robot_initial_geo

        origin_latitude_rad = math.radians(origin_latitude)
        origin_longitude_rad = math.radians(origin_longitude)

        converted_waypoints = []

        for waypoint in self.waypoint_list_geo:
            latitude_rad = math.radians(waypoint.latitude)
            longitude_rad = math.radians(waypoint.longitude)

            average_latitude = (latitude_rad + origin_latitude_rad) / 2.0

            x = (
                earth_radius
                * (longitude_rad - origin_longitude_rad)
                * math.cos(average_latitude)
            )

            y = earth_radius * (latitude_rad - origin_latitude_rad)

            robot_waypoint = Waypoint()
            robot_waypoint.longitude = x
            robot_waypoint.latitude = y
            robot_waypoint.orientation = 0.0

            converted_waypoints.append(robot_waypoint)

        for index in range(len(converted_waypoints) - 1):
            current_waypoint = converted_waypoints[index]
            next_waypoint = converted_waypoints[index + 1]

            delta_x = next_waypoint.longitude - current_waypoint.longitude
            delta_y = next_waypoint.latitude - current_waypoint.latitude

            current_waypoint.orientation = math.atan2(delta_y, delta_x)

        if len(converted_waypoints) > 1:
            converted_waypoints[-1].orientation = converted_waypoints[-2].orientation

        self.waypoint_list_robot_frame = converted_waypoints

        self.get_logger().info(
            f"Converted {len(converted_waypoints)} waypoints to the robot frame"
        )

        for index, waypoint in enumerate(converted_waypoints):
            self.get_logger().info(
                f"Waypoint {index}: "
                f"x={waypoint.longitude:.2f} m, "
                f"y={waypoint.latitude:.2f} m, "
                f"yaw={waypoint.orientation:.3f} rad"
            )

    def plot_waypoints(self):
        self.get_logger().info(
            f" Task 3: Plot and save a graph of loaded waypoints in robot coordinate frame (png)"
        )
        # COMPLETE YOUR CODE HERE
        pass

    def get_robot_waypoints_callback(self, request, response):
        response.waypoints = self.waypoint_list_robot_frame
        response.success = bool(self.waypoint_list_robot_frame)

        return response

    def reset_waypoints_callback(self, request, response):
        response.waypoints = []

        return response

    def call_set_waypoints_geo(self, request):
        self.future = self.set_waypoint_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    waypoint_manager = WaypointManager()

    set_waypoints_msg = SetWaypoints.Request()
    set_waypoints_msg.file_path = "/your_path/waypoints.geojson"
    response = waypoint_manager.call_set_waypoints_geo(set_waypoints_msg)
    if response.success == True:
        waypoint_manager.convert_waypoints_to_robot_frame()
        waypoint_manager.plot_waypoints()
    else:
        print("No waypoints loaded")

    rclpy.spin(waypoint_manager)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
