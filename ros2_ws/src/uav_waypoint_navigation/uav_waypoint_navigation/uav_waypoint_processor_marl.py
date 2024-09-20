#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, OffboardControlMode, VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import sys
import os
# from .WorldEnvOHE import WorldEnv
# import argparse
# import ast
# import torch
# from .CRL2_Swarm import Agent, CategoricalMasked
# import CRL2_Swarm as CRL


class UAVWaypointProcessorMARL(Node):
    def __init__(self):
        # Node name is dynamic based on UAV ID
        node_name = f'uav_waypoint_processor_marl'
        super().__init__(node_name)
        self.get_logger().info('UAV Waypoint Processor MARL Node Started')

        os.system('python3 src/uav_waypoint_navigation/uav_waypoint_navigation/runEnvironment.py')
    
    #*****************************************************************

    #     # Namespace for the UAV (e.g., uav1, uav2, uav3)
    #     self.namespace = f"/px4_{uav_id}"
    #     self.uav_id = int(uav_id)
    #     if self.uav_id == 0:
    #         self.namespace = f""
    #     # self.namespace = f"/px4_1"
    #     # self.namespace = f""

    #     # os.system('ros2 topic echo /px4_1/fmu/out/vehicle_status')

    #     # Create publishers with the namespace
    #     self.command_pub = self.create_publisher(VehicleCommand, f"{self.namespace}/fmu/in/vehicle_command", 10)
    #     self.offboard_control_mode_pub = self.create_publisher(
    #         OffboardControlMode, f"{self.namespace}/fmu/in/offboard_control_mode", 10
    #     )
    #     self.trajectory_setpoint_pub = self.create_publisher(
    #         TrajectorySetpoint, f"{self.namespace}/fmu/in/trajectory_setpoint", 10
    #     )


    #     # Define a QoS profile compatible with the publisher
    #     qos_profile = QoSProfile(
    #         reliability=ReliabilityPolicy.BEST_EFFORT,
    #         history=HistoryPolicy.KEEP_LAST,
    #         depth=10
    #     )

    #     # Update the subscriber with the new QoS profile
    #     self.odometry_sub = self.create_subscription(
    #         VehicleOdometry,
    #         f"{self.namespace}/fmu/out/vehicle_odometry",
    #         self.odometry_callback,
    #         qos_profile
    #     )

    #     self.timestamp = 0  # Time synchronization with PX4
    #     self.current_position = (0.0, 0.0, 0.0)  # Current position of the UAV
    #     self.current_waypoint_index = 0  # Index to track the current waypoint

    #     # Store waypoints passed via command-line arguments
    #     self.waypoints = waypoints

    #     # Timer to send control commands
    #     self.create_timer(0.1, self.control_loop)

    # def odometry_callback(self, msg):
    #     # Update current position based on VehicleOdometry data
    #     self.current_position = (msg.position[0], msg.position[1], msg.position[2])

    # def control_loop(self):
    #     # Send Offboard Control Mode
    #     offboard_mode = OffboardControlMode()
    #     offboard_mode.timestamp = self.timestamp
    #     offboard_mode.position = True
    #     offboard_mode.velocity = False
    #     offboard_mode.acceleration = False
    #     offboard_mode.attitude = False
    #     offboard_mode.body_rate = False
    #     self.offboard_control_mode_pub.publish(offboard_mode)

    #     # Get the current waypoint
    #     current_waypoint = self.waypoints[self.current_waypoint_index]

    #     # Send the current waypoint as a Trajectory Setpoint
    #     self.publish_trajectory_setpoint(current_waypoint)

    #     # Arm and set to Offboard mode
    #     self.arm_vehicle()
    #     self.set_offboard_mode()

    #     # Check if the UAV has reached the current waypoint
    #     if self.check_waypoint_reached(current_waypoint):
    #         self.get_logger().info(
    #             f"Waypoint {self.current_waypoint_index} reached: {current_waypoint}"
    #         )
    #         # Move to the next waypoint
    #         self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)

    # def publish_trajectory_setpoint(self, waypoint):
    #     # Publish the desired trajectory setpoint (waypoint)
    #     waypoint_msg = TrajectorySetpoint()
    #     waypoint_msg.timestamp = self.timestamp
    #     waypoint_msg.position = [waypoint[0], waypoint[1], waypoint[2]]  # x, y, z
    #     waypoint_msg.yaw = 0.0  # Yaw angle (can be adjusted for orientation control)
    #     self.trajectory_setpoint_pub.publish(waypoint_msg)

    # def check_waypoint_reached(self, waypoint, tolerance=0.5):
    #     """
    #     Check if the UAV has reached the current waypoint within a specified tolerance.

    #     Args:
    #         waypoint (tuple): The target waypoint (x, y, z).
    #         tolerance (float): The distance tolerance to consider the waypoint reached.

    #     Returns:
    #         bool: True if the waypoint is reached, False otherwise.
    #     """
    #     # Calculate the Euclidean distance between current position and target waypoint
    #     distance = math.sqrt(
    #         (self.current_position[0] - waypoint[0]) ** 2 +
    #         (self.current_position[1] - waypoint[1]) ** 2 +
    #         (self.current_position[2] - waypoint[2]) ** 2
    #     )
        
    #     self.get_logger().info(f"Current vehicle distance from target: {distance}")
    #     self.get_logger().info(f"{self.namespace}/fmu/out/vehicle_odometry")

    #     return distance < tolerance

    # def arm_vehicle(self):
    #     # Command to arm the vehicle
    #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    # def set_offboard_mode(self):
    #     # Command to switch to offboard mode
    #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    # def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
    #     # Publish a vehicle command
    #     msg = VehicleCommand()
    #     msg.timestamp = self.timestamp
    #     msg.param1 = param1
    #     msg.param2 = param2
    #     msg.command = command
    #     # msg.target_system = 1
    #     msg.target_system = self.uav_id + 1
    #     msg.target_component = 1
    #     msg.source_system = 1
    #     msg.source_component = 1
    #     msg.from_external = True
    #     self.command_pub.publish(msg)
    #     self.get_logger().info(f"Published vehicle command: {command}")

    #*****************************************************************


# def parse_waypoints(args):
#     """ Parse the waypoints from command line arguments """
#     waypoints = []
#     try:
#         # Expect waypoints in the form x1,y1,z1,x2,y2,z2,... as input
#         for i in range(0, len(args), 3):
#             waypoint = (float(args[i]), float(args[i+1]), float(args[i+2]))
#             waypoints.append(waypoint)
#     except (ValueError, IndexError):
#         print("Invalid waypoint arguments. Ensure they are in the form x1,y1,z1,x2,y2,z2,...")
#         sys.exit(1)
#     return waypoints







def convert_actions_to_dict(actions):
    # Define the action mapping
    action_map = {0: "up", 1: "right", 2: "down", 3: "left"}
    
    # Create a dictionary to map drone IDs to their actions
    action_dict = {}
    for i, action in enumerate(actions, 1):
        drone_name = f"drone_{i}"
        action_dict[drone_name] = action_map[action.item()]  # Convert tensor element to Pythopyn int and get action name
    
    return action_dict

def parse_tuple_list(arg):
    tuple_list=ast.literal_eval(arg)
    return tuple_list

def parse_args():
    parser = argparse.ArgumentParser(description="seed")
    parser.add_argument(
        "--track",
        type=lambda x: bool(strtobool(x)),
        default=True,
        nargs="?",
        const=True,
        help="if toggled, this experiment will be tracked with Weights and Biases",
    )
    
    parser.add_argument("--n_drones", type=int, help="The number of drones in sim.", default=3)
    parser.add_argument("--drone_locations", type=parse_tuple_list, default=[(0,0),(0,1),(0,2)])
    parser.add_argument("--n_humans", type=int, help="The number of humans in sim.", default=2)
    parser.add_argument("--human_locations", type=parse_tuple_list, default=[(2, 6), (6, 4)])
    parser.add_argument("--targets", type=parse_tuple_list, default=[(9, 9)])
    parser.add_argument("--max_x", type=int, default=10)
    parser.add_argument("--max_y", type=int, default=10)
    parser.add_argument(
        "--distinct-actor-critic",
        type=lambda x: bool(strtobool(x)),
        default=True,
        nargs="?",
        const=True,
        help="If true, actor and critic will be two separate NNs. If false, actor and critic will be one NN with separate output heads.",
    )
    parser.add_argument(
        "--action-masks",
        type=lambda x: bool(strtobool(x)),
        default=True,
        nargs="?",
        const=True,
        help="If true, invalid agent actions will be masked. If false, they will have no effect.",
    )
    return parser.parse_args()

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Instantiate the navigator node with UAV ID and parsed waypoints
    navigator = UAVWaypointProcessorMARL()

    # Spin the node
    rclpy.spin(navigator)

    # Cleanup
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # #device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    # device = torch.device("cpu")
    # args = parse_args()
    # env = WorldEnv(n_drones=3,drone_locations=[(0,0),(0,1),(0,2)],n_humans=2,human_locations=[(2, 6), (6, 4)],targets=[(9, 9)],max_x=10,max_y=10)
    # next_obs, infos = env.reset()
    
    # agent = Agent(4,args)
    # agent.actor.load_state_dict(torch.load('actor.model', weights_only=True))
    # agent.critic.load_state_dict(torch.load('critic.model', weights_only=True))

    # agent.eval()
    # episode_length = []
    # with torch.no_grad():
    #     dead_agents = []
    #     for step in range(500):
    #         try:
    #             obs = CRL.batchify_obs(next_obs, device)
    #         except ValueError as e:
    #             breakpoint()
    #             raise e
            
    #         masks = torch.tensor([env.get_action_masks(env.all_grids, env.agent_name_mapping[agt]) for agt in env.agents])
    #         actions, logprobs, _, values = agent.get_action_and_value(obs, args, action_masks=masks,device=device)
    #         actions_dict = convert_actions_to_dict(actions)
    #         next_obs, rewards, terms, truncs, infos = env.step(CRL.unbatchify(actions, env))
    #         print(actions_dict)
    #         env.show_grid(env.all_grids)

    #         for agt in (agt for agt in terms if terms[agt] and agt not in dead_agents):
    #             dead_agents.append(agt)
    #             env.agents.remove(agt)

    #         if (
    #                 all([terms[a] for a in terms])
    #                 or all([truncs[a] for a in truncs])
    #                 or terms == []
    #                 or env.agents == []
    #         ):
    #             end_step = step

    #             break

    # os.system('python3 runEnvironment.py')

    main()
    
