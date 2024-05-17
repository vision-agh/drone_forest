"""Scripts with utils for JSON files."""

import json

from typing import Dict, Union


def read_env_config(file_path: str) -> Union[Dict, None]:
    """Read the environment configuration from a JSON file and validate it."""
    # Read JSON and check if the configuration is valid
    with open(file_path, "r") as config_file:
        config_dict = json.load(config_file)

    # Check JSON content and add missing keys
    if "nb_directions" not in config_dict:
        config_dict["nb_directions"] = 4
        print(
            f"The number of directions was not specified. "
            f"Defaulting to {config_dict['nb_directions']}."
        )
    if config_dict["nb_directions"] != 4:
        print(
            f"Only 4 directions are supported (given: {config_dict['nb_directions']})."
        )
        return None
    if "nb_actions" not in config_dict:
        config_dict["nb_actions"] = 4
        print(
            f"The number of actions was not specified. "
            f"Defaulting to {config_dict['nb_actions']}."
        )
    if config_dict["nb_actions"] % config_dict["nb_directions"] != 0:
        print(
            f"The number of actions must be a multiple of the number of directions "
            f"(given: nb_actions = {config_dict['nb_actions']}, "
            f"nb_directions = {config_dict['nb_directions']})."
        )
        return None
    if "actions" not in config_dict:
        base_actions_x = [0.0, -1.0, 0.0, 1.0]
        base_actions_y = [1.0, 0.0, -1.0, 0.0]
        actions = []
        actions_per_direction = (
            config_dict["nb_actions"] // config_dict["nb_directions"]
        )
        for idx_dir in range(config_dict["nb_directions"]):
            actions.extend(
                [
                    [
                        base_actions_x[idx_dir] * (i + 1) / actions_per_direction,
                        base_actions_y[idx_dir] * (i + 1) / actions_per_direction,
                    ]
                    for i in range(actions_per_direction)
                ]
            )
        config_dict["actions"] = actions
        print("Action vectors were not specified. Defaulting to the following:")
        print(actions)
    if len(config_dict["actions"]) != config_dict["nb_actions"]:
        print(
            f"The length of action vector does not match the number of actions "
            f"(given: nb_actions = {config_dict['nb_actions']}, "
            f"len(actions) = {len(config_dict['actions'])})."
        )
        return None
    if "sim_step" not in config_dict:
        config_dict["sim_step"] = 0.1
        print(
            f"The simulation step was not specified. "
            f"Defaulting to {config_dict['sim_step']}."
        )
    if "x_lim" not in config_dict:
        config_dict["x_lim"] = {"min": -10, "max": 10}
        print(
            f"The x-axis limits were not specified. "
            f"Defaulting to {config_dict['x_lim']}."
        )
    if "y_lim" not in config_dict:
        config_dict["y_lim"] = {"min": -10, "max": 10}
        print(
            f"The y-axis limits were not specified. "
            f"Defaulting to {config_dict['y_lim']}."
        )
    if "y_static_limit" not in config_dict:
        config_dict["y_static_limit"] = config_dict["y_lim"]["max"]
        print(
            f"The y-axis static limit was not specified. "
            f"Defaulting to {config_dict['y_static_limit']}."
        )
    if "n_trees" not in config_dict:
        config_dict["n_trees"] = 100
        print(
            f"The number of trees was not specified. "
            f"Defaulting to {config_dict['n_trees']}."
        )
    if "tree_radius_lim" not in config_dict:
        config_dict["tree_radius_lim"] = {"min": 0.2, "max": 1.0}
        print(
            f"The tree radius limits were not specified. "
            f"Defaulting to {config_dict['tree_radius_lim']}."
        )
    if "n_lidar_beams" not in config_dict:
        config_dict["n_lidar_beams"] = 36
        print(
            f"The number of LiDAR beams was not specified. "
            f"Defaulting to {config_dict['n_lidar_beams']}."
        )
    if "lidar_range" not in config_dict:
        config_dict["lidar_range"] = 3.0
        print(
            f"The LiDAR range was not specified. "
            f"Defaulting to {config_dict['lidar_range']}."
        )
    if "min_tree_spare_distance" not in config_dict:
        config_dict["min_tree_spare_distance"] = 0.75
        print(
            f"The minimum tree spare distance was not specified. "
            f"Defaulting to {config_dict['min_tree_spare_distance']}."
        )
    if "max_spawn_attempts" not in config_dict:
        config_dict["max_spawn_attempts"] = 50
        print(
            f"The maximum spawn attempts were not specified. "
            f"Defaulting to {config_dict['max_spawn_attempts']}."
        )
    if "max_speed" not in config_dict:
        config_dict["max_speed"] = 1.0
        print(
            f"The maximum speed was not specified. "
            f"Defaulting to {config_dict['max_speed']}."
        )
    if "max_acceleration" not in config_dict:
        config_dict["max_acceleration"] = 0.6
        print(
            f"The maximum acceleration was not specified. "
            f"Defaulting to {config_dict['max_acceleration']}."
        )
    if "drone_width" not in config_dict:
        config_dict["drone_width"] = 0.2
        print(
            f"The drone width was not specified. "
            f"Defaulting to {config_dict['drone_width']}."
        )
    if "drone_height" not in config_dict:
        config_dict["drone_height"] = 0.4
        print(
            f"The drone height was not specified. "
            f"Defaulting to {config_dict['drone_height']}."
        )

    return config_dict


def read_rl_config(file_path: str) -> Union[Dict, None]:
    """Read the RL configuration from a JSON file and validate it."""
    # Read JSON and check if the configuration is valid
    with open(file_path, "r") as config_file:
        config_dict = json.load(config_file)

    # Check JSON content and add missing keys
    if "algorithm" not in config_dict:
        config_dict["algorithm"] = "ppo"
        print(
            f"The RL algorithm was not specified. "
            f"Defaulting to {config_dict['algorithm']}."
        )
    supported_algorithms = ["ppo"]
    if config_dict["algorithm"] not in supported_algorithms:
        print(
            f"Suppoerted algorithms: {supported_algorithms} "
            f"(given: {config_dict['algorithm']})."
        )
        return None
    if "nb_layers" not in config_dict:
        config_dict["nb_layers"] = 2
        print(
            f"The number of layers was not specified. "
            f"Defaulting to {config_dict['nb_layers']}."
        )
    if "nb_neurons" not in config_dict:
        config_dict["nb_neurons"] = 64
        print(
            f"The number of neurons was not specified. "
            f"Defaulting to {config_dict['nb_neurons']}."
        )
    if "activation_fn" not in config_dict:
        config_dict["activation_fn"] = "sigmoid"
        print(
            f"The activation function was not specified. "
            f"Defaulting to {config_dict['activation_fn']}."
        )
    supported_activation_fns = ["relu", "tanh", "sigmoid"]
    if config_dict["activation_fn"] not in supported_activation_fns:
        print(
            f"Supported activation functions: {supported_activation_fns} "
            f"(given: {config_dict['activation_fn']})."
        )
        return None
    if "nb_training_steps" not in config_dict:
        config_dict["nb_training_steps"] = 30_000_000
        print(
            f"The number of training steps was not specified. "
            f"Defaulting to {config_dict['nb_training_steps']}."
        )
    if "nb_training_envs" not in config_dict:
        config_dict["nb_training_envs"] = 1
        print(
            f"The number of training environments was not specified. "
            f"Defaulting to {config_dict['nb_training_envs']}."
        )
    if "nb_eval_envs" not in config_dict:
        config_dict["nb_eval_envs"] = 1
        print(
            f"The number of evaluation environments was not specified. "
            f"Defaulting to {config_dict['nb_eval_envs']}."
        )
    if "nb_eval_every" not in config_dict:
        config_dict["nb_eval_every"] = 100_000
        print(
            f"The evaluation frequency was not specified. "
            f"Defaulting to {config_dict['nb_eval_every']}."
        )

    return config_dict
