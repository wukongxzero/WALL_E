"""Verify AsroNavigationEnvCfg actually constructs, resets, and steps.

Run with: ./isaaclab.sh -p <this file> --headless
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Verify AsroNavigationEnvCfg.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sys

import torch

sys.path.insert(0, "/home/wukong/WALL_E/simulation/isaac_lab")
from asro_navigation_env_cfg import AsroNavigationEnvCfg
from isaaclab.envs import ManagerBasedRLEnv


def main():
    cfg = AsroNavigationEnvCfg()
    cfg.scene.num_envs = 4
    env = ManagerBasedRLEnv(cfg=cfg)

    print(f"[INFO] Observation space: {env.observation_space}")
    print(f"[INFO] Action space: {env.action_space}")

    obs, _ = env.reset()
    print(f"[INFO] Reset OK. Obs keys: {list(obs.keys())}, policy obs shape: {obs['policy'].shape}")

    for step in range(30):
        actions = torch.rand(env.action_space.shape, device=env.device) * 2 - 1
        obs, rew, terminated, truncated, info = env.step(actions)
        if step % 10 == 0:
            print(f"[INFO] step={step} reward={rew.tolist()} terminated={terminated.tolist()}")

    print("[INFO] Test complete — env constructs, resets, and steps correctly.")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
