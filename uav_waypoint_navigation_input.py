#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, OffboardControlMode, Timesync, VehicleOdometry
import math


class UAVWaypointNavigator(Node):
    def __init__(self, waypoints):
        super().__init__('uav_waypoint_navigator')

        # Create publishers
        self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )

        # Subscribers
        self.timesync_sub = self.create_subscription(
            Timesync, "/fmu/out/timesync", self.timesync_callback, 10
        )
        self.odometry_sub = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.odometry_callback, 10
        )

        self.timestamp = 0  # Time synchronization with PX4
        self.current_position = (0.0, 0.0, 0.0)  # Current position of the UAV
        self.current_waypoint_index = 0  # Index to track current waypoint
        self.waypoints = waypoints  # Waypoints entered by the user

        # Timer to send control commands
        self.create_timer(0.1, self.control_loop)

    def timesync_callback(self, msg):
        # Update timestamp to sync with PX4
        self.timestamp = msg.timestamp

    def odometry_callback(self, msg):
        # Update current position based on VehicleOdometry data
        self.current_position = (msg.position[0], msg.position[1], msg.position[2])

    def control_loop(self):
        # Send Offboard Control Mode
        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = self.timestamp
        offboard_mode.position = True
        offboard_mode.velocity = False
        offboard_mode.acceleration = False
        offboard_mode.attitude = False
        offboard_mode.body_rate = False
        self.offboard_control_mode_pub.publish(offboard_mode)

        # Check if waypoints are available
        if not self.waypoints:
            self.get_logger().info("No waypoints provided. Exiting navigation.")
            return

        # Get the current waypoint
        current_waypoint = self.waypoints[self.current_waypoint_index]

        # Send the current waypoint as a Trajectory Setpoint
        self.publish_trajectory_setpoint(current_waypoint)

        # Arm and set to Offboard mode
        self.arm_vehicle()
        self.set_offboard_mode()

        # Check if the UAV has reached the current waypoint
        if self.check_waypoint_reached(current_waypoint):
            self.get_logger().info(
                f"Waypoint {self.current_waypoint_index} reached: {current_waypoint}"
            )
            # Move to the next waypoint
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)

    def publish_trajectory_setpoint(self, waypoint):
        # Publish the desired trajectory setpoint (waypoint)
        waypoint_msg = TrajectorySetpoint()
        waypoint_msg.timestamp = self.timestamp
        waypoint_msg.position = [waypoint[0], waypoint[1], waypoint[2]]  # x, y, z
        waypoint_msg.yaw = 0.0  # Yaw angle (can be adjusted for orientation control)
        self.trajectory_setpoint_pub.publish(waypoint_msg)

    def check_waypoint_reached(self, waypoint, tolerance=0.5):
        """
        Check if the UAV has reached the current waypoint within a specified tolerance.

        Args:
            waypoint (tuple): The target waypoint (x, y, z).
            tolerance (float): The distance tolerance to consider the waypoint reached.

        Returns:
            bool: True if the waypoint is reached, False otherwise.
        """
        # Calculate the Euclidean distance between current position and target waypoint
        distance = math.sqrt(
            (self.current_position[0] - waypoint[0]) ** 2 +
            (self.current_position[1] - waypoint[1]) ** 2 +
            (self.current_position[2] - waypoint[2]) ** 2
        )

        return distance < tolerance

    def arm_vehicle(self):
        # Command to arm the vehicle
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def set_offboard_mode(self):
        # Command to switch to offboard mode
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        # Publish a vehicle command
        msg = VehicleCommand()
        msg.timestamp = self.timestamp
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
        self.get_logger().info(f"Published vehicle command: {command}")


def get_user_input_waypoints():
    """
    Function to get user input for waypoints.

    Returns:
        list of tuples: List of (x, y, z) waypoints.
    """
    waypoints = []
    print("Enter waypoints in the format: x y z (type 'done' when finished)")

    while True:
        user_input = input("Enter waypoint (or 'done' to finish): ")

        if user_input.lower() == 'done':
            break

        try:
            x, y, z = map(float, user_input.split())
            waypoints.append((x, y, z))
        except ValueError:
            print("Invalid input. Please enter coordinates in the format: x y z")

    return waypoints


def main(args=None):
    # Get waypoints from user input
    waypoints = get_user_input_waypoints()

    # If no waypoints are provided, exit the script
    if not waypoints:
        print("No waypoints entered. Exiting.")
        return

    rclpy.init(args=args)
    navigator = UAVWaypointNavigator(waypoints)
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
