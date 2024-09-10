To simulate UAV waypoint navigation in ROS 2 Humble using Gazebo Fortress and PX4 SITL, you'll need to set up a simulation environment that integrates all these components. This guide will walk you through setting up the simulation, creating waypoints, and controlling the UAV to follow these waypoints.

### Overview of the Setup

1. **Gazebo Fortress**: Used as the 3D simulation environment.
2. **PX4 SITL**: Runs the PX4 autopilot in a simulated environment.
3. **MAVROS**: Provides communication between ROS 2 and PX4 using the MAVLink protocol.
4. **ROS 2 Node**: Controls the UAV's navigation by setting waypoints.

### Prerequisites

1. **ROS 2 Humble**: Install ROS 2 Humble from the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html).
2. **PX4 Autopilot**: Install the PX4 Autopilot. You can find installation instructions on the [PX4 official documentation](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html).
3. **MAVROS 2**: Install MAVROS for ROS 2 from source, as it may not be fully available via binaries for ROS 2 Humble.
4. **Gazebo Fortress**: Install Gazebo Fortress following the [installation guide](https://gazebosim.org/docs/fortress/install_ubuntu).

### Step 1: Launch PX4 SITL with Gazebo Fortress

PX4 provides ready-to-use launch files that start the SITL simulation with Gazebo.

```bash
cd ~/px4-Autopilot
make px4_sitl gazebo
```

Ensure that Gazebo Fortress is the default simulator for PX4. You may need to configure PX4's environment or launch files to use Gazebo Fortress.

### Step 2: Launch MAVROS

Start the MAVROS node to bridge ROS 2 with PX4:

```bash
ros2 launch mavros px4.launch.py fcu_url:=udp://:14540@
```

Ensure that the MAVROS node is successfully connected to the PX4 SITL. You should see messages indicating a successful connection.

### Step 3: Create ROS 2 Package for UAV Waypoint Navigation

Create a ROS 2 package to control the UAV.

```bash
ros2 pkg create uav_waypoint_navigation --build-type ament_python --dependencies rclpy mavros_msgs
```

### Step 4: Python Script for Waypoint Navigation (`uav_waypoint_navigation.py`)

Create a script inside your package to manage waypoints and control the UAV.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, WaypointReached
from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear
from mavros_msgs.msg import Waypoint

class UAVWaypointNavigator(Node):

    def __init__(self):
        super().__init__('uav_waypoint_navigator')

        # Subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.waypoint_reached_sub = self.create_subscription(WaypointReached, '/mavros/mission/reached', self.waypoint_reached_callback, 10)

        # Service clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.waypoint_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.waypoint_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')

        # Current state
        self.current_state = State()

        # Timer to set mode and arm vehicle
        self.timer = self.create_timer(1.0, self.timer_callback)

    def state_callback(self, msg):
        self.current_state = msg

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f'Waypoint {msg.wp_seq} reached')

    def timer_callback(self):
        # Set mode to OFFBOARD if not already set
        if self.current_state.mode != 'OFFBOARD':
            self.set_offboard_mode()

        # Arm the vehicle if not already armed
        if not self.current_state.armed:
            self.arm_vehicle()

    def set_offboard_mode(self):
        # Set the vehicle to OFFBOARD mode
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Set mode service not available')
            return

        request = SetMode.Request()
        request.custom_mode = 'OFFBOARD'
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().mode_sent:
            self.get_logger().info('OFFBOARD mode set successfully')
        else:
            self.get_logger().error('Failed to set OFFBOARD mode')

    def arm_vehicle(self):
        # Arm the vehicle
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Arming service not available')
            return

        request = CommandBool.Request()
        request.value = True
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info('Vehicle armed successfully')
        else:
            self.get_logger().error('Failed to arm vehicle')

    def clear_waypoints(self):
        # Clear existing waypoints
        if not self.waypoint_clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Waypoint clear service not available')
            return

        request = WaypointClear.Request()
        future = self.waypoint_clear_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info('Waypoints cleared successfully')
        else:
            self.get_logger().error('Failed to clear waypoints')

    def push_waypoints(self, waypoints):
        # Push waypoints to the PX4
        if not self.waypoint_push_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Waypoint push service not available')
            return

        request = WaypointPush.Request()
        request.waypoints = waypoints
        future = self.waypoint_push_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info('Waypoints pushed successfully')
        else:
            self.get_logger().error('Failed to push waypoints')

    def create_waypoints(self):
        # Define waypoints
        waypoints = []

        # Define the first waypoint
        waypoint1 = Waypoint()
        waypoint1.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        waypoint1.command = Waypoint.NAV_WAYPOINT
        waypoint1.is_current = True
        waypoint1.autocontinue = True
        waypoint1.param1 = 0  # Hold time
        waypoint1.param2 = 5  # Acceptance radius in meters
        waypoint1.param3 = 0  # Pass through
        waypoint1.param4 = float('nan')  # Yaw
        waypoint1.x_lat = 47.397742  # Latitude
        waypoint1.y_long = 8.545594  # Longitude
        waypoint1.z_alt = 10  # Altitude in meters
        waypoints.append(waypoint1)

        # Define a second waypoint
        waypoint2 = Waypoint()
        waypoint2.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        waypoint2.command = Waypoint.NAV_WAYPOINT
        waypoint2.is_current = False
        waypoint2.autocontinue = True
        waypoint2.param1 = 0  # Hold time
        waypoint2.param2 = 5  # Acceptance radius in meters
        waypoint2.param3 = 0  # Pass through
        waypoint2.param4 = float('nan')  # Yaw
        waypoint2.x_lat = 47.398242  # Latitude
        waypoint2.y_long = 8.546594  # Longitude
        waypoint2.z_alt = 15  # Altitude in meters
        waypoints.append(waypoint2)

        # Clear any existing waypoints
        self.clear_waypoints()

        # Push the new waypoints to the vehicle
        self.push_waypoints(waypoints)

def main(args=None):
    rclpy.init(args=args)
    navigator = UAVWaypointNavigator()
    navigator.create_waypoints()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Points in the Code

1. **Waypoint Management**: Waypoints are defined using the `Waypoint` message, which specifies position, altitude, and other navigation parameters.
2. **Mode Setting and Arming**: The script automatically sets the UAV to `OFFBOARD` mode and arms it, preparing it for waypoint navigation.
3. **Service Calls**: Uses ROS 2 service calls to interact with the PX4 SITL for setting modes, arming, clearing waypoints, and pushing new waypoints.

### Step 5: Build and Run the Package

1. **Build the package**:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select uav_waypoint_navigation
   source install/setup.bash
   ```

2. **Run the script**:

   ```bash
   ros2 run uav_waypoint_navigation uav_waypoint_navigation.py
   ```

### Summary

This setup allows you to control a UAV in a Gazebo Fortress simulation using ROS 2, PX4 SITL, and MAVROS. The UAV will follow the defined waypoints, demonstrating waypoint navigation in a simulated environment. Ensure that all dependencies are correctly installed and the communication between ROS 2, PX4, and MAVROS is functioning as expected for smooth operation.