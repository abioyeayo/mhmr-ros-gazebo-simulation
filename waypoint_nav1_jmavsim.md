To control PX4 SITL (Software In The Loop) waypoint navigation using ROS 2, you'll need to interact with the PX4 autopilot using MAVROS, a ROS package that acts as a bridge between ROS and MAVLink (the communication protocol used by PX4). The general workflow involves setting up a ROS 2 node that sends waypoint commands to the PX4 autopilot running in SITL.

Here's a step-by-step guide and a sample ROS 2 Python script (`waypoint_navigator.py`) to control PX4 SITL waypoint navigation:

### Prerequisites

1. **ROS 2 Installation**: Make sure you have ROS 2 installed. The example is based on ROS 2 Humble or later versions.
2. **PX4 Autopilot**: Install PX4 and ensure you can run SITL simulations.
3. **MAVROS**: You need the ROS 2 version of MAVROS (`mavros_msgs`). MAVROS can be built from source for ROS 2 since ROS 2 support is still in progress. Ensure that the appropriate plugins are working for communication.

### Setup

1. **Launch PX4 SITL:**

   ```bash
   cd ~/px4-Autopilot
   make px4_sitl jmavsim
   ```

2. **Run MAVROS Node**:

   Launch the MAVROS node configured for SITL with the appropriate parameters:

   ```bash
   ros2 launch mavros px4.launch.py fcu_url:=udp://:14540@
   ```

3. **Create a ROS 2 Package**:

   If you haven't already created a ROS 2 package, create one:

   ```bash
   ros2 pkg create waypoint_navigator --build-type ament_python --dependencies rclpy mavros_msgs
   ```

### ROS 2 Script: `waypoint_navigator.py`

Below is the script to control the PX4 SITL waypoint navigation:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State, Waypoint, WaypointList, WaypointReached
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.srv import WaypointPush, WaypointClear, CommandHome

class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        # Initialize clients, publishers, and subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.waypoint_reached_sub = self.create_subscription(WaypointReached, '/mavros/mission/reached', self.waypoint_reached_callback, 10)
        self.local_position_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_position_callback, 10)
        self.global_position_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.global_position_callback, 10)

        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.waypoint_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        self.waypoint_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        
        self.current_state = State()
        self.home_position_set = False

        # Create a timer to repeatedly call the state checking function
        self.timer = self.create_timer(2.0, self.timer_callback)

    def state_callback(self, msg):
        self.current_state = msg

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f'Waypoint {msg.wp_seq} reached')

    def local_position_callback(self, msg):
        pass

    def global_position_callback(self, msg):
        pass

    def timer_callback(self):
        # Check if the vehicle is connected and in OFFBOARD mode
        if self.current_state.mode != 'OFFBOARD':
            self.set_offboard_mode()

        # Check if the vehicle is armed
        if not self.current_state.armed:
            self.arm_vehicle()

    def set_offboard_mode(self):
        # Set the vehicle to OFFBOARD mode
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set Mode service not available')
            return
        request = SetMode.Request()
        request.custom_mode = 'OFFBOARD'
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().mode_sent:
            self.get_logger().info('OFFBOARD mode set successfully')
        else:
            self.get_logger().info('Failed to set OFFBOARD mode')

    def arm_vehicle(self):
        # Arm the vehicle
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available')
            return
        request = CommandBool.Request()
        request.value = True
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Vehicle armed successfully')
        else:
            self.get_logger().info('Failed to arm vehicle')

    def clear_waypoints(self):
        # Clear existing waypoints
        if not self.waypoint_clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint Clear service not available')
            return
        request = WaypointClear.Request()
        future = self.waypoint_clear_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Waypoints cleared successfully')
        else:
            self.get_logger().info('Failed to clear waypoints')

    def push_waypoints(self, waypoints):
        # Push waypoints to the PX4
        if not self.waypoint_push_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint Push service not available')
            return
        request = WaypointPush.Request()
        request.waypoints = waypoints
        future = self.waypoint_push_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Waypoints pushed successfully')
        else:
            self.get_logger().info('Failed to push waypoints')

    def create_waypoints(self):
        # Define a sequence of waypoints
        waypoints = []

        waypoint1 = Waypoint()
        waypoint1.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        waypoint1.command = Waypoint.NAV_WAYPOINT
        waypoint1.is_current = True
        waypoint1.autocontinue = True
        waypoint1.param1 = 0  # Hold time
        waypoint1.param2 = 0  # Acceptance radius
        waypoint1.param3 = 0  # Pass through
        waypoint1.param4 = 0  # Yaw
        waypoint1.x_lat = 47.397742  # Latitude
        waypoint1.y_long = 8.545594  # Longitude
        waypoint1.z_alt = 10         # Altitude
        waypoints.append(waypoint1)

        # Add more waypoints as needed
        # You can add more waypoints following the same format

        self.clear_waypoints()
        self.push_waypoints(waypoints)

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    navigator.create_waypoints()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Points of the Script

- **State Monitoring**: The script monitors the state of the PX4 (armed, offboard mode) and commands the vehicle accordingly.
- **Waypoint Creation**: The `create_waypoints()` function defines a sequence of waypoints that the PX4 will follow.
- **Waypoint Management**: Waypoints are pushed to PX4 via MAVROS services.
- **Service Calls**: The script uses ROS 2 service calls to set modes, arm the vehicle, and manage waypoints.

### Running the Script

1. Make the script executable:

   ```bash
   chmod +x waypoint_navigator.py
   ```

2. Run the script using:

   ```bash
   ros2 run waypoint_navigator waypoint_navigator.py
   ```

Make sure your ROS 2 environment is sourced and PX4 SITL is running correctly for this script to communicate with it properly.
