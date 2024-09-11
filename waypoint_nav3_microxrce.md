Using Micro XRCE-DDS Agent instead of MAVROS to communicate with PX4 SITL in a ROS 2 environment requires a different approach since Micro XRCE-DDS is the middleware that allows communication with microcontrollers or resource-constrained environments directly via the DDS-XRCE protocol, which PX4 supports natively. Here’s a detailed guide to setting up the system and controlling PX4 SITL using Micro XRCE-DDS Agent in Gazebo Fortress.

### Overview of the Setup

1. **PX4 SITL**: Simulates the UAV running the PX4 firmware.
2. **Gazebo Fortress**: 3D simulation environment for visualizing and testing the UAV.
3. **Micro XRCE-DDS Agent**: Acts as a bridge between the PX4 firmware and ROS 2, using the DDS-XRCE protocol.
4. **ROS 2 Node**: A custom ROS 2 node will be used to send waypoint commands to PX4 through the XRCE-DDS bridge.

### Prerequisites

1. **ROS 2 Humble**: Make sure ROS 2 Humble is installed and set up correctly. Installation instructions can be found on the [ROS 2 Humble documentation](https://docs.ros.org/en/humble/Installation.html).
2. **PX4 Autopilot**: PX4 should be installed on your machine. Follow the [PX4 installation guide](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html).
3. **Gazebo Fortress**: Gazebo Fortress should be installed on your system. Instructions can be found on the [Gazebo Fortress installation page](https://gazebosim.org/docs/fortress/install_ubuntu).
4. **Micro XRCE-DDS Agent**: Install Micro XRCE-DDS Agent, which enables DDS-XRCE communication between PX4 and your ROS 2 environment.

### Step 1: Install Micro XRCE-DDS Agent

To install Micro XRCE-DDS Agent, follow these steps:

1. Clone the Micro XRCE-DDS Agent repository:

    ```bash
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    ```

2. Run the Micro XRCE-DDS Agent:

    ```bash
    MicroXRCEAgent udp4 -p 8888
    ```

This command starts the agent and listens on UDP port 8888, which is the default port for PX4 XRCE-DDS communication.

### Step 2: Launch PX4 SITL with Gazebo Fortress

Launch PX4 SITL with Gazebo:

```bash
cd ~/px4-Autopilot
make px4_sitl gazebo
```

### Step 3: Create a ROS 2 Package for UAV Waypoint Navigation

Create a new ROS 2 package to control the UAV using DDS-XRCE:

In ~/ros2_ws/src folder, run the following in terminal

```bash
ros2 pkg create uav_waypoint_navigation --build-type ament_python --dependencies rclpy
```

### Step 4: Python Script for Waypoint Navigation (`uav_waypoint_navigation.py`)

Here's the Python script that sends waypoint commands to PX4 using DDS-XRCE instead of MAVROS:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, OffboardControlMode, Timesync

class UAVWaypointNavigator(Node):

    def __init__(self):
        super().__init__('uav_waypoint_navigator')

        # Create publishers
        self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)

        # Subscriber to sync time with PX4
        self.timesync_sub = self.create_subscription(Timesync, "/fmu/out/timesync", self.timesync_callback, 10)

        self.timestamp = 0  # Time synchronization with PX4

        # Start a timer to send control commands
        self.create_timer(0.1, self.control_loop)

    def timesync_callback(self, msg):
        # Update timestamp to sync with PX4
        self.timestamp = msg.timestamp

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

        # Send a Trajectory Setpoint (waypoint)
        waypoint = TrajectorySetpoint()
        waypoint.timestamp = self.timestamp
        waypoint.position = [0.0, 0.0, -5.0]  # Set desired x, y, z position (z is negative in NED frame)
        waypoint.yaw = 0.0  # Set desired yaw
        self.trajectory_setpoint_pub.publish(waypoint)

        # Arm and set to Offboard mode
        self.arm_vehicle()
        self.set_offboard_mode()

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

def main(args=None):
    rclpy.init(args=args)
    navigator = UAVWaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Components of the Script

1. **Time Synchronization**: Synchronizes time with PX4 using the `Timesync` message.
2. **Offboard Control Mode**: Sets the UAV into offboard control mode using the `OffboardControlMode` message.
3. **Waypoint Control**: Sends position commands using the `TrajectorySetpoint` message to guide the UAV to the desired waypoints.
4. **Vehicle Commands**: Arms the UAV and sets it into offboard mode using the `VehicleCommand` message.

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

This setup uses Micro XRCE-DDS Agent to interface directly with PX4 via the DDS-XRCE protocol, allowing you to send commands and waypoints without needing MAVROS. The provided ROS 2 node publishes control messages to set the UAV into offboard mode and guide it through specified waypoints. This approach is efficient for resource-constrained systems and leverages the native DDS-XRCE support in PX4, making it ideal for real-time UAV control scenarios.


----------------------------

The error "no executable found" usually occurs when the ROS 2 package has not been correctly built or the script hasn't been properly set up as an executable within the package. Here's how you can resolve this issue:

### Step-by-Step Guide to Fix the Error

1. **Ensure the Script is Marked as Executable**

   The script `uav_waypoint_navigation.py` needs to be executable. To make the script executable, run:

   ```bash
   chmod +x ~/ros2_ws/src/uav_waypoint_navigation/uav_waypoint_navigation/uav_waypoint_navigation.py
   ```

2. **Edit `setup.py` to Include the Executable**

   In your package's `setup.py`, ensure that the script is properly declared as an entry point. Here’s how the `setup.py` file should look:

   ```python
   from setuptools import setup

   package_name = 'uav_waypoint_navigation'

   setup(
       name=package_name,
       version='0.0.0',
       packages=[package_name],
       data_files=[
           ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='Your Name',
       maintainer_email='your.email@example.com',
       description='UAV waypoint navigation using ROS 2 and Micro XRCE-DDS Agent',
       license='Apache License 2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'uav_waypoint_navigation = uav_waypoint_navigation.uav_waypoint_navigation:main',
           ],
       },
   )
   ```

   Ensure the `entry_points` section correctly maps the script's location and the `main` function.

3. **Verify Package Structure**

   Your package should have the following structure:

   ```
   ~/ros2_ws/src/uav_waypoint_navigation/
   ├── package.xml
   ├── setup.py
   ├── resource/
   ├── uav_waypoint_navigation/
   │   ├── __init__.py
   │   └── uav_waypoint_navigation.py
   ```

   Make sure the `__init__.py` file exists in the `uav_waypoint_navigation` directory (it can be empty). This makes the directory recognizable as a Python package.

4. **Rebuild the Package**

   After making changes, rebuild your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select uav_waypoint_navigation
   source install/setup.bash
   ```

5. **Run the Script**

   Now, try running the script again:

   ```bash
   ros2 run uav_waypoint_navigation uav_waypoint_navigation
   ```

### Common Troubleshooting Tips

- **Check for Typos**: Ensure there are no typos in the `entry_points` section of `setup.py`.
- **Re-source the Environment**: After building, always source the workspace: `source install/setup.bash`.
- **Check Permissions**: Verify that the script has the correct executable permissions using `ls -l`.

These steps should help resolve the "no executable found" error and ensure that your ROS 2 node runs correctly.


-------------------------

Installing px4_msg first:

$ cd ~/ros2_ws/src/

$ git clone https://github.com/PX4/px4_msgs.git
$ git clone https://github.com/PX4/px4_ros_com.git

$ cd ..
$ source /opt/ros/humble/setup.bash
$ colcon build


Running the Example (in a new terminal)

$ cd ~/ros2_ws/
$ source /opt/ros/humble/setup.bash
$ source install/local_setup.bash

$ ros2 launch px4_ros_com sensor_combined_listener.launch.py

ROS 2 CLI
$ ros2 topic list
$ ros2 topic echo /fmu/out/vehicle_status


--------------------------

Adding square motion trajectory:

To add a square motion waypoint trajectory to the `uav_waypoint_navigation.py` script, we need to modify the `control_loop` function to include a sequence of waypoints that form a square path. The UAV will follow these waypoints one by one, completing the square trajectory.

### Updated `uav_waypoint_navigation.py` Script

Below is the updated script with square motion waypoints implemented. The waypoints are defined such that the UAV will move in a square pattern, maintaining a constant altitude.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, OffboardControlMode
# , Timesync


class UAVWaypointNavigator(Node):
    def __init__(self):
        super().__init__('uav_waypoint_navigator')

        # Create publishers
        self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )

        # # Subscriber to sync time with PX4
        # self.timesync_sub = self.create_subscription(
        #     Timesync, "/fmu/out/timesync", self.timesync_callback, 10
        # )

        self.timestamp = 0  # Time synchronization with PX4
        self.current_waypoint_index = 0  # Index to track current waypoint

        # Define the square waypoints (x, y, z) in meters
        self.waypoints = [
            (0.0, 0.0, -5.0),   # Start position
            (5.0, 0.0, -5.0),   # Move 5 meters to the east
            (5.0, 5.0, -5.0),   # Move 5 meters to the north
            (0.0, 5.0, -5.0),   # Move 5 meters to the west
            (0.0, 0.0, -5.0)    # Return to start position
        ]

        # Timer to send control commands
        self.create_timer(0.1, self.control_loop)

    # def timesync_callback(self, msg):
    #     # Update timestamp to sync with PX4
    #     self.timestamp = msg.timestamp

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
        # Placeholder for actual position check logic
        # Replace this function with appropriate feedback from PX4 to check if the UAV is close to the waypoint
        # For example, you can use feedback from PX4's position data to compare with the target waypoint
        # In a real scenario, implement logic to verify the UAV's position with a given tolerance
        return False

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


def main(args=None):
    rclpy.init(args=args)
    navigator = UAVWaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Key Changes and Explanation

1. **Square Waypoints Definition**:
   - The `self.waypoints` list contains waypoints that form a square trajectory. Each waypoint is a tuple of `(x, y, z)` in meters.
   - The UAV moves to these waypoints sequentially, forming a square pattern.

2. **Waypoint Tracking**:
   - `self.current_waypoint_index` keeps track of the current waypoint.
   - After reaching a waypoint, the script moves to the next waypoint using a modulo operation to loop back to the start.

3. **Trajectory Setpoint Publishing**:
   - The `publish_trajectory_setpoint()` function sends the current waypoint to the UAV, guiding it to the desired position.

4. **Waypoint Reached Check**:
   - The `check_waypoint_reached()` function is a placeholder to determine if the UAV has reached the waypoint. You need to replace this with real feedback logic from PX4 (e.g., comparing the current position to the target position).

### Additional Considerations

- **Position Feedback**: You may need to implement feedback to verify if the UAV has reached each waypoint, such as subscribing to PX4's position topic and comparing the current position to the waypoint coordinates with a tolerance.
- **Yaw Control**: Adjust the yaw value in `waypoint_msg.yaw` to control the orientation of the UAV at each waypoint.

This updated script will allow the UAV to navigate in a square pattern using the defined waypoints in a Gazebo simulation environment controlled by PX4 and Micro XRCE-DDS.


-----------------------
Added code to check if waypoint has been reached:

To add the functionality to check if the UAV has reached a waypoint, we need to subscribe to a topic that provides the current position of the UAV. PX4 publishes the vehicle position data on several topics, but since we are using Micro XRCE-DDS, we can subscribe to the `VehicleOdometry` topic, which provides the UAV's current position in NED (North, East, Down) coordinates.

### Updated `uav_waypoint_navigation.py` Script with Waypoint Reach Check

Here's the modified script with the logic to check if the UAV has reached the current waypoint. This implementation subscribes to `VehicleOdometry` to get the current position and compares it against the target waypoint with a specified tolerance.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, OffboardControlMode, VehicleOdometry #, Timesync 
import math


class UAVWaypointNavigator(Node):
    def __init__(self):
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
        # self.timesync_sub = self.create_subscription(
        #     Timesync, "/fmu/out/timesync", self.timesync_callback, 10
        # )
        self.odometry_sub = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.odometry_callback, 10
        )

        self.timestamp = 0  # Time synchronization with PX4
        self.current_position = (0.0, 0.0, 0.0)  # Current position of the UAV
        self.current_waypoint_index = 0  # Index to track current waypoint

        # Define the square waypoints (x, y, z) in meters
        self.waypoints = [
            (0.0, 0.0, -5.0),  # Start position
            (5.0, 0.0, -5.0),  # Move 5 meters to the east
            (5.0, 5.0, -5.0),  # Move 5 meters to the north
            (0.0, 5.0, -5.0),  # Move 5 meters to the west
            (0.0, 0.0, -5.0)   # Return to start position
        ]

        # Timer to send control commands
        self.create_timer(0.1, self.control_loop)

    # def timesync_callback(self, msg):
    #     # Update timestamp to sync with PX4
    #     self.timestamp = msg.timestamp

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


def main(args=None):
    rclpy.init(args=args)
    navigator = UAVWaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Key Changes and Explanation

1. **Position Feedback from `VehicleOdometry`**:
   - A subscription to the `VehicleOdometry` topic updates the UAV's current position (`self.current_position`).
   - This data is used to track the UAV's position in real-time.

2. **Waypoint Reach Check**:
   - The `check_waypoint_reached()` function calculates the Euclidean distance between the UAV's current position and the target waypoint.
   - If the distance is within a specified tolerance (0.5 meters by default), it considers the waypoint reached and moves on to the next one.

3. **Waypoint Progression**:
   - Once a waypoint is reached, the UAV moves to the next waypoint in the list using `self.current_waypoint_index`.

4. **Safety and Orientation Control**:
   - The UAV maintains a constant yaw of `0.0`. Adjust the yaw value if specific orientations are desired at each waypoint.

This script will now make the UAV navigate through the waypoints, checking when each waypoint is reached before proceeding to the next, forming a complete square trajectory.


------------------------
Added user input waypoint navigation:

Here's a Python script that allows a user to input a series of (x, y, z) waypoints via the terminal. The script prompts the user to enter the coordinates of each waypoint sequentially and then stores these waypoints in a list. Once the user finishes entering waypoints, the UAV will navigate through them using ROS 2.

This script assumes you have a similar structure as before, using the ROS 2 environment, `px4_msgs`, and the same UAV control setup.

### Script Overview

- **User Input**: The user can enter waypoints in the format `x y z` (e.g., `5 5 -5`).
- **Execution**: After inputting all waypoints, the UAV navigates through each waypoint sequentially.
- **Error Handling**: The script validates inputs to ensure they are valid floating-point numbers.

### `uav_waypoint_navigation_input.py`

Here's the complete script:

```python
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
```

### Key Features and Explanation

1. **User Input for Waypoints**:
   - The `get_user_input_waypoints()` function prompts the user to enter waypoints in the format `x y z`.
   - The user can enter as many waypoints as desired. Typing `done` finishes the input process.

2. **Handling Invalid Inputs**:
   - The script validates that the input consists of three floating-point numbers. If the input is invalid, it prompts the user again.

3. **Waypoint Navigation**:
   - The UAV navigates through the entered waypoints sequentially.
   - The `check_waypoint_reached()` function determines if the UAV has reached the current waypoint based on the Euclidean distance.

4. **ROS 2 Integration**:
   - The waypoints are published to the UAV in the form of `TrajectorySetpoint` messages.
   - The vehicle is armed and set to Offboard mode during the control loop.

### Usage Instructions

1. Save the script as `uav_waypoint_navigation_input.py` in your ROS 2 package directory.
2. Make sure the script is executable:
   ```bash
   chmod +x uav_waypoint_navigation_input.py
   ```
3. Build your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select uav_waypoint_navigation
   source install/setup.bash
   ```
4. Run the script:
   ```bash
   ros2 run uav_waypoint_navigation uav_waypoint_navigation_input.py
   ```
5. Follow the terminal prompts to enter waypoints and start the navigation.

This approach provides a flexible way to input custom trajectories for the UAV, making it easier to test different flight paths during simulation or real flight tests.


---------------------------------
fixing no execution found error:


The "no executable found" error typically occurs because ROS 2 cannot find the script as an executable in the package. This issue usually happens due to one of the following reasons:

1. The script is not marked as executable.
2. The script is not correctly installed as an executable within the ROS 2 package.
3. The script name does not match the entry in the package’s `setup.py`.

Here's a step-by-step guide to fixing this error:

### 1. Ensure the Script is Marked as Executable

First, make sure that the script `uav_waypoint_navigation_input.py` is marked as executable. You can do this by running:

```bash
chmod +x uav_waypoint_navigation_input.py
```

### 2. Modify the `setup.py` File

Ensure that your package’s `setup.py` file is properly configured to install your script as an executable. Your `setup.py` should have an entry point specifying the executable script.

Here's a sample `setup.py` configuration:

```python
from setuptools import setup

package_name = 'uav_waypoint_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='UAV waypoint navigation package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_waypoint_navigation = uav_waypoint_navigation.uav_waypoint_navigation_input:main',
        ],
    },
)
```

Make sure:
- `uav_waypoint_navigation_input:main` refers to the `main` function in `uav_waypoint_navigation_input.py`.
- The path matches your package structure.

### 3. Ensure Proper Directory Structure

Ensure that your Python script `uav_waypoint_navigation_input.py` is located within the `uav_waypoint_navigation` package directory. The directory structure should look like this:

```
ros2_ws/
└── src/
    └── uav_waypoint_navigation/
        ├── package.xml
        ├── setup.py
        └── uav_waypoint_navigation/
            └── uav_waypoint_navigation_input.py
```

### 4. Rebuild the Package

After making changes, rebuild the package:

```bash
cd ~/ros2_ws
colcon build --packages-select uav_waypoint_navigation
```

Source the workspace after building:

```bash
source install/setup.bash
```

### 5. Run the Executable

Try running the script again using:

```bash
ros2 run uav_waypoint_navigation uav_waypoint_navigation
```

### Troubleshooting Tips

- **Check Permissions**: If you still get errors, check the permissions of the script:
  ```bash
  ls -l uav_waypoint_navigation/uav_waypoint_navigation_input.py
  ```
  Ensure the output shows executable permissions, e.g., `-rwxr-xr-x`.
  
- **Verify Entry Points**: Ensure the script name and function match exactly with what is in `setup.py`.

Following these steps should resolve the "no executable found" error and allow you to run your ROS 2 Python script correctly. Let me know if you need further assistance!



-------------------

The error message `New publisher discovered on topic '/fmu/out/vehicle_odometry', offering incompatible QoS. No messages will be received from it. Last incompatible policy: RELIABILITY` indicates that the Quality of Service (QoS) settings between your subscriber and the publisher on the `/fmu/out/vehicle_odometry` topic are not compatible, specifically due to a mismatch in the `RELIABILITY` policy.

### Understanding QoS Compatibility Issue

In ROS 2, QoS settings determine how data is communicated between nodes. If the QoS policies between a publisher and a subscriber are not compatible, messages will not be exchanged. In this case, the specific issue is related to the `RELIABILITY` policy, which defines whether the communication is reliable (ensuring delivery) or best-effort (faster but not guaranteed).

### Steps to Fix the QoS Compatibility Issue

To resolve this, you need to adjust the QoS settings of your subscriber to match or be compatible with those of the publisher. Here's how you can modify the QoS settings for your `VehicleOdometry` subscription:

#### 1. Adjust QoS Settings in the Code

Update the QoS settings of your `VehicleOdometry` subscriber in your `uav_waypoint_navigation_input.py` script to match the publisher's settings. Here's how you can set it up:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Define a QoS profile compatible with the publisher
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Set to BEST_EFFORT to match the publisher's setting
    history=HistoryPolicy.KEEP_LAST,
    depth=10  # Set an appropriate queue size
)

# Update the subscriber with the new QoS profile
self.odometry_sub = self.create_subscription(
    VehicleOdometry,
    "/fmu/out/vehicle_odometry",
    self.odometry_callback,
    qos_profile
)
```

#### 2. Explanation of the QoS Settings

- **`ReliabilityPolicy.BEST_EFFORT`**: This policy allows for faster communication at the expense of message delivery guarantees, which matches the typical configuration used by UAV odometry publishers like PX4.
- **`HistoryPolicy.KEEP_LAST`**: Keeps the last `n` samples (defined by `depth`) in the queue.
- **`depth=10`**: Sets the depth of the queue to 10 messages, which is usually sufficient for UAV odometry data.

#### 3. Update Your Script

Ensure that the rest of the script remains consistent with these changes. Here's the relevant section of the code after applying the changes:

```python
# Inside the __init__ method of UAVWaypointNavigator
self.odometry_sub = self.create_subscription(
    VehicleOdometry,
    "/fmu/out/vehicle_odometry",
    self.odometry_callback,
    qos_profile  # Apply the QoS profile to match the publisher
)
```

### 4. Rebuild and Run the Package

After modifying the script, rebuild your package:

```bash
cd ~/ros2_ws
colcon build --packages-select uav_waypoint_navigation
source install/setup.bash
```

Run the script again:

```bash
ros2 run uav_waypoint_navigation uav_waypoint_navigation
```

### Additional Tips

- **Check PX4 Parameters**: Ensure the PX4 SITL environment is correctly configured to use `BEST_EFFORT` reliability if it's not explicitly stated in their documentation.
- **Inspect the Topic**: Use `ros2 topic info /fmu/out/vehicle_odometry -v` to inspect the QoS settings of the publisher for further adjustments if necessary.

By applying these changes, the subscriber should now match the QoS settings of the publisher, allowing your node to receive messages from the `/fmu/out/vehicle_odometry` topic.


----------------------

To show the current vehicle position in ROS 2, you can use the `ros2 topic echo` and `ros2 topic info` commands to inspect the relevant topic that publishes the vehicle's position data. In a PX4-Gazebo simulation setup, vehicle position information is typically published on topics like `/fmu/out/vehicle_odometry` or `/fmu/out/vehicle_local_position`, depending on your specific setup.

### Steps to Display Current Vehicle Position Using ROS 2

1. **Identify the Position Topic:**

   First, identify which topic is publishing the vehicle’s position data using the `ros2 topic list` command:

   ```bash
   ros2 topic list
   ```

   Look for topics that might contain odometry, position, or local position data, such as:
   
   - `/fmu/out/vehicle_odometry`
   - `/fmu/out/vehicle_local_position`
   - `/odom`
   - `/vehicle/odom` (or similar)

2. **Inspect Topic Information:**

   Use the `ros2 topic info` command to get information about the topic, including the message type. For example:

   ```bash
   ros2 topic info /fmu/out/vehicle_odometry
   ```

   This will display details like the message type, publishers, and subscribers associated with the topic.

3. **Display the Current Position Using `ros2 topic echo`:**

   Once you've identified the correct topic, use `ros2 topic echo` to display the current vehicle position in real-time. For example:

   ```bash
   ros2 topic echo /fmu/out/vehicle_odometry
   ```

   This command will output the message data being published on that topic. The message type is usually `px4_msgs/msg/VehicleOdometry` or `nav_msgs/msg/Odometry`.

4. **Interpret the Output:**

   The `ros2 topic echo` output will include fields for position, orientation, and other relevant data. For `px4_msgs/msg/VehicleOdometry`, look for the position fields `x`, `y`, and `z`. For `nav_msgs/msg/Odometry`, you can find the position under `pose.pose.position`.

   Example output for `/fmu/out/vehicle_odometry` might look like this:

   ```plaintext
   position:
     x: 5.0
     y: 10.0
     z: -3.0
   ```
   
   For `nav_msgs/msg/Odometry`, you might see:

   ```plaintext
   pose:
     pose:
       position:
         x: 5.0
         y: 10.0
         z: -3.0
   ```

### Summary

1. **Identify the topic** publishing vehicle position data with `ros2 topic list`.
2. **Inspect the topic** to confirm its message type using `ros2 topic info`.
3. **Echo the topic data** with `ros2 topic echo` to display the current position of the vehicle.

These steps will help you continuously monitor your UAV's position in the Gazebo simulator, allowing you to verify that your waypoint navigation is working as expected. Let me know if you need any more specific assistance with interpreting the data or adjusting your setup!

