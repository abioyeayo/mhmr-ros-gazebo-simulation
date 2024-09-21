#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, OffboardControlMode, VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import sys
import os
import json
from time import sleep
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

        # Namespace for the UAV (e.g., uav1, uav2, uav3)
        self.namespace1 = f""
        self.namespace2 = f"/px4_1"
        self.namespace3 = f"/px4_2"

        # self.uav_id = int(uav_id)
        # if self.uav_id == 0:
        #     self.namespace = f""
        # # self.namespace = f"/px4_1"
        # self.namespace = f""

        # os.system('ros2 topic echo /px4_1/fmu/out/vehicle_status')

        # Create publishers with the namespace
        self.command_pub1 = self.create_publisher(VehicleCommand, f"{self.namespace1}/fmu/in/vehicle_command", 10)
        self.command_pub2 = self.create_publisher(VehicleCommand, f"{self.namespace2}/fmu/in/vehicle_command", 10)
        self.command_pub3 = self.create_publisher(VehicleCommand, f"{self.namespace3}/fmu/in/vehicle_command", 10)

        self.offboard_control_mode_pub1 = self.create_publisher(OffboardControlMode, f"{self.namespace1}/fmu/in/offboard_control_mode", 10)
        self.offboard_control_mode_pub2 = self.create_publisher(OffboardControlMode, f"{self.namespace2}/fmu/in/offboard_control_mode", 10)
        self.offboard_control_mode_pub3 = self.create_publisher(OffboardControlMode, f"{self.namespace3}/fmu/in/offboard_control_mode", 10)


        self.trajectory_setpoint_pub1 = self.create_publisher(TrajectorySetpoint, f"{self.namespace1}/fmu/in/trajectory_setpoint", 10)
        self.trajectory_setpoint_pub2 = self.create_publisher(TrajectorySetpoint, f"{self.namespace2}/fmu/in/trajectory_setpoint", 10)
        self.trajectory_setpoint_pub3 = self.create_publisher(TrajectorySetpoint, f"{self.namespace3}/fmu/in/trajectory_setpoint", 10)


        # Define a QoS profile compatible with the publisher
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,depth=10)

        # Update the subscriber with the new QoS profile
        self.odometry_sub1 = self.create_subscription(VehicleOdometry,f"{self.namespace1}/fmu/out/vehicle_odometry",self.odometry_callback1,qos_profile)
        self.odometry_sub2 = self.create_subscription(VehicleOdometry,f"{self.namespace2}/fmu/out/vehicle_odometry",self.odometry_callback2,qos_profile)
        self.odometry_sub3 = self.create_subscription(VehicleOdometry,f"{self.namespace3}/fmu/out/vehicle_odometry",self.odometry_callback3,qos_profile)

        self.timestamp = 0  # Time synchronization with PX4

        self.uavs_current_position = [
            [-7.0, -7.0, 0.0],  # UAV1
            [-9.0, -5.0, 0.0],  # UAV2
            [-9.0, -9.0, 0.0]   # UAV3
        ]

        # self.uavs_next_waypoint = [
        #     [-7.0, -7.0, -5.0],  # UAV1
        #     [-9.0, -5.0, -5.0],  # UAV2
        #     [-9.0, -9.0, -5.0]   # UAV3
        # ]

        self.uavs_next_waypoint = [
            [-1.0, -2.0, -5.5],  # UAV1
            [-2.0, -2.0, -6.5],  # UAV2
            [-3.0, -2.0, -7.5]   # UAV3
        ]

        os.system('python3 src/uav_waypoint_navigation/uav_waypoint_navigation/runEnvironment.py --drone_locations="[(1,1),(0,2),(0,0)]" --human_locations="[(4, 8), (6, 1)]"')

        waypoint_data = open('waypoint_data.txt', 'r')    # point test p(3.68, 3.40,1.05)
        uav_waypoints = waypoint_data.readlines()

        # Parse waypoints from marl model
        waypoint_index = 0
        for uav_waypoint in uav_waypoints:
            # print(uav_waypoint[0:11])
            uav_step_direction = ["hold","hold","hold"]
            if uav_waypoint[0:11] == "{'drone_1':":
                waypoint_index = waypoint_index + 1
                # print(str(waypoint_index) + '. ' + uav_waypoint)

                # Convert single quotes to double quotes JSON data to a Python object 
                uav_waypoint = uav_waypoint.replace("\'", "\"")
                uav_direction = json.loads(uav_waypoint)
                for x in range(0,3):
                    try:
                        uav_step_direction[x] = uav_direction["drone_" + str(x+1)]
                        if uav_step_direction[x] == "down":
                            self.uavs_next_waypoint[x][1] = self.uavs_next_waypoint[x][1] + 2
                        elif uav_step_direction[x] == "up":
                            self.uavs_next_waypoint[x][1] = self.uavs_next_waypoint[x][1] - 2
                        elif uav_step_direction[x] == "right":
                            self.uavs_next_waypoint[x][0] = self.uavs_next_waypoint[x][0] + 2
                        elif uav_step_direction[x] == "left":
                            self.uavs_next_waypoint[x][0] = self.uavs_next_waypoint[x][0] - 2
                        else:
                            print("Invalid step...")
                    except KeyError:
                        uav_step_direction[x] = "hold"
                print(uav_step_direction)
                print(self.uavs_next_waypoint)

                loop_counter = 0
                while True:
                    # Timer to send control commands
                    # self.create_timer(0.1, self.control_loop)
                    sleep(0.05)
                    self.control_loop()

                    # Check if the UAV has reached the current waypoint or move to next waypoint after 3 seconds i.e. 60 x 0.05
                    loop_counter = loop_counter + 1
                    uav_id = 1
                    if (self.check_waypoint_reached(uav_id)) or (self.check_waypoint_reached(uav_id+1)) or (self.check_waypoint_reached(uav_id+2)) or (loop_counter > 60):
                        self.get_logger().info(f"All UAVs waypoints reached")
                        # Move to the next waypoint
                        break
                
    
    #*****************************************************************

        

                
                # self.current_position = (0.0, 0.0, 0.0)  # Current position of the UAV
                # self.current_waypoint_index = 0  # Index to track the current waypoint

                # # Store waypoints passed via command-line arguments
                # self.waypoints = waypoints

        self.create_timer(0.1, self.control_loop)

                

    # Update current position based on VehicleOdometry data
    def odometry_callback1(self, msg):        
        self.uavs_current_position[0] = [msg.position[0], msg.position[1], msg.position[2]]
    def odometry_callback2(self, msg):
        self.uavs_current_position[1] = (msg.position[0], msg.position[1], msg.position[2])
    def odometry_callback3(self, msg):
        self.uavs_current_position[2] = (msg.position[0], msg.position[1], msg.position[2])

    def control_loop(self):
        # Send Offboard Control Mode
        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = self.timestamp
        offboard_mode.position = True
        offboard_mode.velocity = False
        offboard_mode.acceleration = False
        offboard_mode.attitude = False
        offboard_mode.body_rate = False
        self.offboard_control_mode_pub1.publish(offboard_mode)
        self.offboard_control_mode_pub2.publish(offboard_mode)
        self.offboard_control_mode_pub3.publish(offboard_mode)

        # Get the current waypoint
        # current_waypoint = self.waypoints[self.current_waypoint_index]
        current_waypoint1 = self.uavs_next_waypoint[0]
        current_waypoint2 = self.uavs_next_waypoint[1]
        current_waypoint3 = self.uavs_next_waypoint[2]

        # Send the current waypoint as a Trajectory Setpoint
        # self.publish_trajectory_setpoint(current_waypoint)
        self.publish_trajectory_setpoint()

        # Arm and set to Offboard mode
        self.arm_vehicle()
        self.set_offboard_mode()

        
                

    def publish_trajectory_setpoint(self):
        # Publish the desired trajectory setpoint (waypoint)
        waypoint_msg = TrajectorySetpoint()
        waypoint_msg.timestamp = self.timestamp
        # waypoint_msg.position = [waypoint[0], waypoint[1], waypoint[2]]  # x, y, z
        waypoint_msg.position = [self.uavs_next_waypoint[0][0], self.uavs_next_waypoint[0][1], self.uavs_next_waypoint[0][2]]  # UAV1 x, y, z
        waypoint_msg.yaw = 0.0  # Yaw angle (can be adjusted for orientation control)
        self.trajectory_setpoint_pub1.publish(waypoint_msg)
        waypoint_msg.position = [self.uavs_next_waypoint[1][0], self.uavs_next_waypoint[1][1], self.uavs_next_waypoint[1][2]]  # UAV2 x, y, z
        self.trajectory_setpoint_pub2.publish(waypoint_msg)
        waypoint_msg.position = [self.uavs_next_waypoint[2][0], self.uavs_next_waypoint[2][1], self.uavs_next_waypoint[2][2]]  # UAV3 x, y, z
        self.trajectory_setpoint_pub3.publish(waypoint_msg)

    def check_waypoint_reached(self, uav_id, tolerance=0.5):
        """
        Check if the UAV has reached the current waypoint within a specified tolerance.

        Args:
            waypoint (tuple): The target waypoint (x, y, z).
            tolerance (float): The distance tolerance to consider the waypoint reached.

        Returns:
            bool: True if the waypoint is reached, False otherwise.
        """
        # # Calculate the Euclidean distance between current position and target waypoint
        # distance = math.sqrt(
        #     (self.current_position[0] - waypoint[0]) ** 2 +
        #     (self.current_position[1] - waypoint[1]) ** 2 +
        #     (self.current_position[2] - waypoint[2]) ** 2
        # )

        distance = math.sqrt(
            (self.uavs_current_position[uav_id-1][0] - self.uavs_next_waypoint[uav_id-1][0]) ** 2 +
            (self.uavs_current_position[uav_id-1][1] - self.uavs_next_waypoint[uav_id-1][1]) ** 2 +
            (self.uavs_current_position[uav_id-1][2] - self.uavs_next_waypoint[uav_id-1][2]) ** 2
        )

        
        
        # self.get_logger().info(f"Current UAV{uav_id} distance from target: {distance}")
        # # self.get_logger().info(f"{self.namespace}/fmu/out/vehicle_odometry")

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
        # msg.target_system = self.uav_id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub1.publish(msg)
        msg.target_system = 2
        self.command_pub2.publish(msg)
        msg.target_system = 3
        self.command_pub3.publish(msg)

        # self.get_logger().info(f"Published vehicle command: {command}")

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







# def convert_actions_to_dict(actions):
#     # Define the action mapping
#     action_map = {0: "up", 1: "right", 2: "down", 3: "left"}
    
#     # Create a dictionary to map drone IDs to their actions
#     action_dict = {}
#     for i, action in enumerate(actions, 1):
#         drone_name = f"drone_{i}"
#         action_dict[drone_name] = action_map[action.item()]  # Convert tensor element to Pythopyn int and get action name
    
#     return action_dict

# def parse_tuple_list(arg):
#     tuple_list=ast.literal_eval(arg)
#     return tuple_list

# def parse_args():
#     parser = argparse.ArgumentParser(description="seed")
#     parser.add_argument(
#         "--track",
#         type=lambda x: bool(strtobool(x)),
#         default=True,
#         nargs="?",
#         const=True,
#         help="if toggled, this experiment will be tracked with Weights and Biases",
#     )
    
#     parser.add_argument("--n_drones", type=int, help="The number of drones in sim.", default=3)
#     parser.add_argument("--drone_locations", type=parse_tuple_list, default=[(0,0),(0,1),(0,2)])
#     parser.add_argument("--n_humans", type=int, help="The number of humans in sim.", default=2)
#     parser.add_argument("--human_locations", type=parse_tuple_list, default=[(2, 6), (6, 4)])
#     parser.add_argument("--targets", type=parse_tuple_list, default=[(9, 9)])
#     parser.add_argument("--max_x", type=int, default=10)
#     parser.add_argument("--max_y", type=int, default=10)
#     parser.add_argument(
#         "--distinct-actor-critic",
#         type=lambda x: bool(strtobool(x)),
#         default=True,
#         nargs="?",
#         const=True,
#         help="If true, actor and critic will be two separate NNs. If false, actor and critic will be one NN with separate output heads.",
#     )
#     parser.add_argument(
#         "--action-masks",
#         type=lambda x: bool(strtobool(x)),
#         default=True,
#         nargs="?",
#         const=True,
#         help="If true, invalid agent actions will be masked. If false, they will have no effect.",
#     )
#     return parser.parse_args()

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
    
