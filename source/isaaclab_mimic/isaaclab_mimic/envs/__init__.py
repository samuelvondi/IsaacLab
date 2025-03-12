# Copyright (c) 2024-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Sub-package with environment wrappers for Isaac Lab Mimic."""

import gymnasium as gym

# from isaaclab_mimic.envs import franka_lift_ik_rel_mimic_env_cfg
# from isaaclab_mimic.envs import franka_lift_ik_rel_mimic_env

from .franka_stack_ik_rel_mimic_env import FrankaCubeStackIKRelMimicEnv
from .franka_stack_ik_rel_mimic_env_cfg import FrankaCubeStackIKRelMimicEnvCfg

from .franka_lift_ik_rel_mimic_env import FrankaCubeLiftIKRelMimicEnv #added line
from .franka_lift_ik_rel_mimic_env_cfg import FrankaCubeLiftIKRelMimicEnvCfg #added

##
# Inverse Kinematics - Relative Pose Control
##

gym.register(
    id="Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0",
    entry_point="isaaclab_mimic.envs:FrankaCubeStackIKRelMimicEnv",
    kwargs={
        "env_cfg_entry_point": franka_stack_ik_rel_mimic_env_cfg.FrankaCubeStackIKRelMimicEnvCfg,
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Lift-Cube-Franka-IK-Rel-Mimic-v0",
    entry_point="isaaclab_mimic.envs:FrankaCubeLiftIKRelMimicEnv",
    kwargs={
        "env_cfg_entry_point": franka_lift_ik_rel_mimic_env_cfg.FrankaCubeLiftIKRelMimicEnvCfg,
    },
    disable_env_checker=True,
)