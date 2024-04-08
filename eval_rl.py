"""Script to evaluate the performance of a trained RL agent on the environment."""

import argparse

from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy

from drone_forest.gym_wrapper import DroneForestEnv


def main(args):
    # Load the environment
    env = DroneForestEnv(
        dt=0.1,
        xlim=(-15, 15),
        ylim=(-3, 27),
        n_trees=100,
        max_tree_radius=0.7,
        n_lidar_beams=36,
        max_lidar_range=3.0,
        min_spare_distance=0.5,
        max_spawn_attempts=50,
        seed=0,
    )
    # Load the trained RL agent
    model = PPO.load(args.model_path)

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
    parser.add_argument(
        "model_path", type=str, help="Path to the trained RL agent model."
    )
    parser.add_argument(
        "--num_episodes",
        type=int,
        default=100,
        help="Number of episodes to evaluate the agent for.",
    )
    args = parser.parse_args()
    main(args)
