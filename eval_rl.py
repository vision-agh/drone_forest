"""Script to evaluate the performance of a trained RL agent on the environment."""

import argparse

# import cv2
import os

from stable_baselines3 import PPO

# from stable_baselines3.common.evaluation import evaluate_policy

from scripts.gym_wrapper import DroneForestEnv

import scripts.json_utils as jutils

EVAL_RESULTS_DIR = "eval_rl"


def main(args):
    """Evaluate the trained RL agent on the environment.

    Args:
        args (argparse.Namespace): The parsed command-line arguments.
    """
    # Save path to experiment directory into evaluation results
    if args.store_results:
        with open(os.path.join(EVAL_RESULTS_DIR, "exp_dir.txt"), "w") as f:
            f.write(os.path.abspath(args.exp_dir))

    # Load the experiment configuration
    config_dict = jutils.read_env_config(os.path.join(args.exp_dir, "env_config.json"))

    # Load the environment
    env = DroneForestEnv(
        actions=config_dict["actions"],
        dt=config_dict["sim_step"],
        x_lim=(config_dict["x_lim"]["min"], config_dict["x_lim"]["max"]),
        y_lim=(config_dict["y_lim"]["min"], config_dict["y_lim"]["max"]),
        y_static_limit=config_dict["y_static_limit"],
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
        drone_width_m=config_dict["drone_width"],
        drone_height_m=config_dict["drone_height"],
    )

    # Load the trained RL agent
    model = PPO.load(os.path.join(args.exp_dir, "best_model.zip"))

    end_evaluation = False
    avg_reward = 0
    success_rate = 0
    print(" Seed " + " " * 5 + "Score " + " " * 3 + "Success")
    for i in range(args.num_episodes):
        if args.store_results:
            open(os.path.join(EVAL_RESULTS_DIR, "drone_pos.csv"), "w").close()
        obs, _ = env.reset(seed=i)
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            if args.render:
                env.render()
            if done:
                if info["is_goal_reached"]:
                    success_rate += 1
                avg_reward += info["drone_position_y"]
                print(
                    f"{i:5d} {info['drone_position_y']:10.2f} "
                    f"{('Yes' if info['is_goal_reached'] else 'No')!s:>10}"
                )
            if args.store_results:
                with open(os.path.join(EVAL_RESULTS_DIR, "drone_pos.csv"), "a") as f:
                    f.write(f"{info['drone_position_x']},{info['drone_position_y']}\n")
        if end_evaluation:
            break
    env.close()
    avg_reward /= args.num_episodes
    success_rate /= args.num_episodes

    # Print the evaluation results
    print(f"Average score: {avg_reward:.2f}")
    print(f"Succes rate: {success_rate:3.2%}")


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
    parser.add_argument(
        "--render",
        action="store_true",
        help="Render the environment during evaluation.",
    )
    parser.add_argument(
        "--store_results",
        action="store_true",
        help="Store the evaluation results in a directory.",
    )
    args = parser.parse_args()
    main(args)
