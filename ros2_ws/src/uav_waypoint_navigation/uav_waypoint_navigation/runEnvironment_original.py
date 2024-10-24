from WorldEnvOHE import WorldEnv
import argparse
import ast
import torch
from CRL2_Swarm import Agent, CategoricalMasked
import CRL2_Swarm as CRL
import sys


filename = 'waypoint_data.txt'
f = open(filename,'w')
f.write('Model Inference begins\n')
f.flush()
#f.close()

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
    parser.add_argument("--drone_locations", type=parse_tuple_list, default=None)
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

if __name__ == '__main__':
    #device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    device = torch.device("cpu")
    args = parse_args()
    env = WorldEnv(n_drones=args.n_drones,drone_locations=args.drone_locations,n_humans=args.n_humans,human_locations=args.human_locations,targets=args.targets,max_x=args.max_x,max_y=args.max_y)
    next_obs, infos = env.reset()
    
    agent = Agent(4,args)
    agent.actor.load_state_dict(torch.load('src/uav_waypoint_navigation/uav_waypoint_navigation/actor.model', weights_only=True))
    agent.critic.load_state_dict(torch.load('src/uav_waypoint_navigation/uav_waypoint_navigation/critic.model', weights_only=True))

    agent.eval()
    episode_length = []
    with torch.no_grad():
        dead_agents = []
        for step in range(500):
            try:
                obs = CRL.batchify_obs(next_obs, device)
            except ValueError as e:
                breakpoint()
                raise e
            
            masks = torch.tensor([env.get_action_masks(env.all_grids, env.agent_name_mapping[agt]) for agt in env.agents])
            actions, logprobs, _, values = agent.get_action_and_value(obs, args, action_masks=masks,device=device)
            actions_dict = convert_actions_to_dict(actions)
            next_obs, rewards, terms, truncs, infos = env.step(CRL.unbatchify(actions, env))
            print(actions_dict)
            print(actions_dict, file=f)
            #f.write(actions_dict)
            #f.flush()
            
            env.show_grid(env.all_grids)

            for agt in (agt for agt in terms if terms[agt] and agt not in dead_agents):
                dead_agents.append(agt)
                env.agents.remove(agt)

            if (
                    all([terms[a] for a in terms])
                    or all([truncs[a] for a in truncs])
                    or terms == []
                    or env.agents == []
            ):
                end_step = step

                f.close()

                break
    
