#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, OffboardControlMode, VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import sys
import os


class UAVWaypointNavigator(Node):
    def __init__(self, uav_id, waypoints):
        # Node name is dynamic based on UAV ID
        node_name = f'px4_{uav_id}_waypoint_navigator'
        super().__init__(node_name)

        # Namespace for the UAV (e.g., uav1, uav2, uav3)
        self.namespace = f"/px4_{uav_id}"
        self.uav_id = int(uav_id)
        if self.uav_id == 0:
            self.namespace = f""
        # self.namespace = f"/px4_1"
        # self.namespace = f""

        # os.system('ros2 topic echo /px4_1/fmu/out/vehicle_status')

        # Create publishers with the namespace
        self.command_pub = self.create_publisher(VehicleCommand, f"{self.namespace}/fmu/in/vehicle_command", 10)
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, f"{self.namespace}/fmu/in/offboard_control_mode", 10
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f"{self.namespace}/fmu/in/trajectory_setpoint", 10
        )


        # Define a QoS profile compatible with the publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Update the subscriber with the new QoS profile
        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            f"{self.namespace}/fmu/out/vehicle_odometry",
            self.odometry_callback,
            qos_profile
        )

        self.timestamp = 0  # Time synchronization with PX4
        self.current_position = (0.0, 0.0, 0.0)  # Current position of the UAV
        self.current_waypoint_index = 0  # Index to track the current waypoint

        # Store waypoints passed via command-line arguments
        self.waypoints = waypoints

        # Timer to send control commands
        self.create_timer(0.1, self.control_loop)

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
        
        self.get_logger().info(f"Current vehicle distance from target: {distance}")
        self.get_logger().info(f"{self.namespace}/fmu/out/vehicle_odometry")

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
        # msg.target_system = 1
        msg.target_system = self.uav_id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
        self.get_logger().info(f"Published vehicle command: {command}")


def parse_waypoints(args):
    """ Parse the waypoints from command line arguments """
    waypoints = []
    try:
        # Expect waypoints in the form x1,y1,z1,x2,y2,z2,... as input
        for i in range(0, len(args), 3):
            waypoint = (float(args[i]), float(args[i+1]), float(args[i+2]))
            waypoints.append(waypoint)
    except (ValueError, IndexError):
        print("Invalid waypoint arguments. Ensure they are in the form x1,y1,z1,x2,y2,z2,...")
        sys.exit(1)
    return waypoints


def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Expect UAV ID and waypoints from command line arguments
    if len(sys.argv) < 5:
        print("Usage: uav_waypoint_navigation.py <uav_id> <x1> <y1> <z1> [<x2> <y2> <z2> ...]")
        sys.exit(1)

    uav_id = sys.argv[1]  # UAV ID from arguments

    
    waypoints = parse_waypoints(sys.argv[2:])

    # Instantiate the navigator node with UAV ID and parsed waypoints
    navigator = UAVWaypointNavigator(uav_id, waypoints)

    # Spin the node
    rclpy.spin(navigator)

    # Cleanup
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

