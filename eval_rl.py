"""Script to evaluate the performance of a trained RL agent on the environment."""

import argparse
import json
import os

from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy

from scripts.gym_wrapper import DroneForestEnv


def main(args):
    """Evaluate the trained RL agent on the environment.

    Args:
        args (argparse.Namespace): The parsed command-line arguments.
    """
    # Load the experiment configuration
    with open(os.path.join(args.exp_dir, "env_config.json"), "r") as f:
        config_dict = json.load(f)
    assert config_dict["nb_actions"] in [4, 8], "Invalid number of actions."

    # Load the environment
    env = DroneForestEnv(
        actions=config_dict["actions"],
        dt=config_dict["sim_step"],
        x_lim=(config_dict["x_lim"]["min"], config_dict["x_lim"]["max"]),
        y_lim=(config_dict["y_lim"]["min"], config_dict["y_lim"]["max"]),
        n_trees=config_dict["n_trees"],
        tree_radius_lim=(
            config_dict["tree_radius_lim"]["min"],
            config_dict["tree_radius_lim"]["max"],
        ),
        n_lidar_beams=config_dict["n_lidar_beams"],
        lidar_range=config_dict["lidar_range"],
        min_tree_spare_distance=config_dict["min_tree_spare_distance"],
        max_spawn_attempts=config_dict["max_spawn_attempts"],
        max_speed=config_dict["max_speed"],
        max_acceleration=config_dict["max_acceleration"],
    )

    # Load the trained RL agent
    model = PPO.load(os.path.join(args.exp_dir, "best_model.zip"))

    # Evaluate the agent
    mean_reward, std_reward = evaluate_policy(
        model,
        env,
        n_eval_episodes=args.num_episodes,
        render=False,
        deterministic=True,
    )

    # Print the evaluation results
    print(f"Mean reward: {mean_reward:.2f}, Std reward: {std_reward:.2f}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Evaluate the performance of a trained RL agent on the environment."
    )
    parser.add_argument("exp_dir", type=str, help="Path to the experiment directory.")
    parser.add_argument(
        "--num_episodes",
        type=int,
        default=100,
        help="Number of episodes to evaluate the agent for.",
    )
    args = parser.parse_args()
    main(args)
