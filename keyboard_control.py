# pip3 install pynput
# source /opt/ros/humble/setup.bash
# source ~/ros2_ws/install/setup.bash


import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand
from pynput import keyboard

class UAVControlNode(Node):
    def __init__(self):
        super().__init__('uav_control_node')
        
        # Create a publisher for the vehicle command
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/command', 10)
        
        # Create a VehicleCommand message instance
        self.cmd_msg = VehicleCommand()
        
        # Define the initial command parameters
        self.cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        self.cmd_msg.param1 = 1.0  # Set to a default mode (e.g., offboard)

        # Set up keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()
        
        # Timer to publish commands
        self.timer = self.create_timer(0.1, self.publish_cmd)

    def on_press(self, key):
        try:
            if key.char == 'w':  # Move forward
                self.cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_CHANGE_ALTITUDE
                self.cmd_msg.param1 = 1.0  # Increase altitude
            elif key.char == 's':  # Move backward
                self.cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_CHANGE_ALTITUDE
                self.cmd_msg.param1 = -1.0  # Decrease altitude
            elif key.char == 'a':  # Move left
                self.cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_CHANGE_ALTITUDE
                self.cmd_msg.param2 = 1.0  # Move left
            elif key.char == 'd':  # Move right
                self.cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_CHANGE_ALTITUDE
                self.cmd_msg.param2 = -1.0  # Move right
            elif key.char == 'q':  # Rotate counterclockwise
                self.cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_REPOSITION
                self.cmd_msg.param3 = 1.0  # Rotate counterclockwise
            elif key.char == 'e':  # Rotate clockwise
                self.cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_REPOSITION
                self.cmd_msg.param3 = -1.0  # Rotate clockwise
            elif key.char == 'x':  # Stop all movement
                self.cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
                self.cmd_msg.param1 = 0.0  # Stop all movement
        except AttributeError:
            pass

    def publish_cmd(self):
        self.cmd_pub.publish(self.cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    uav_control_node = UAVControlNode()
    
    try:
        rclpy.spin(uav_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        uav_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

