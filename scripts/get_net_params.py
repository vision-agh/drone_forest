"""Get the network parameters from the trained model."""

import torch

import numpy as np
import gymnasium as gym
from stable_baselines3.ppo.policies import MlpPolicy
from stable_baselines3.common.utils import constant_fn

action_space = gym.spaces.Discrete(4)
observation_space = gym.spaces.Box(low=0.0, high=3.0, shape=(36,), dtype=np.float64)

model = MlpPolicy(
    observation_space=observation_space,
    action_space=action_space,
    lr_schedule=constant_fn(0.0),
)
model.load_state_dict(torch.load("policy.pth"))
