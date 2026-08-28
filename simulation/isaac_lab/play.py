"""Watch a trained checkpoint drive either robot toward its commanded goal.

Loads the most recent (or a given) checkpoint from train.py and runs
inference only -- no learning, no auto-close, so there's actually time to
look at it in the GUI. Uses few envs by default so the view isn't a
64-robot grid.

Run with:
  ./isaaclab.sh -p <this file> --robot wall_e
  ./isaaclab.sh -p <this file> --robot asro --algorithm td3 --checkpoint /path/to/checkpoint.pt
"""

import argparse
import glob
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Watch a trained checkpoint.")
parser.add_argument("--robot", type=str, choices=["asro", "wall_e"], required=True)
parser.add_argument("--algorithm", type=str, choices=["ppo", "td3"], default="ppo")
parser.add_argument("--checkpoint", type=str, default=None,
                     help="Path to a specific .pt checkpoint. Defaults to the most recent run's latest checkpoint.")
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--steps", type=int, default=1000)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sys

import torch

sys.path.insert(0, "/home/wukong/WALL_E/simulation/isaac_lab")

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.utils.io import load_yaml
from isaaclab_rl.skrl import SkrlVecEnvWrapper
from skrl.utils.runner.torch import Runner

AGENT_CFG_PATHS = {
    "ppo": "/home/wukong/WALL_E/simulation/isaac_lab/agents/skrl_ppo_cfg.yaml",
    "td3": "/home/wukong/WALL_E/simulation/isaac_lab/agents/skrl_td3_cfg.yaml",
}


def find_latest_checkpoint(robot: str, algorithm: str) -> str:
    log_root = f"/home/wukong/IsaacLab/logs/skrl/{robot}_navigation"
    runs = sorted(glob.glob(os.path.join(log_root, f"*_{algorithm}_torch")))
    if not runs:
        raise FileNotFoundError(f"No training runs found under {log_root} -- run train.py first.")
    checkpoints = sorted(
        glob.glob(os.path.join(runs[-1], "checkpoints", "*.pt")),
        key=os.path.getmtime,
    )
    if not checkpoints:
        raise FileNotFoundError(f"No checkpoints found in {runs[-1]}/checkpoints/")
    return checkpoints[-1]


def main():
    if args_cli.robot == "asro":
        from asro_navigation_env_cfg import AsroNavigationEnvCfg as EnvCfg
    else:
        from wall_e_navigation_env_cfg import WallENavigationEnvCfg as EnvCfg

    checkpoint = args_cli.checkpoint or find_latest_checkpoint(args_cli.robot, args_cli.algorithm)
    print(f"[INFO] Loading checkpoint: {checkpoint}")

    env_cfg = EnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs

    agent_cfg = load_yaml(AGENT_CFG_PATHS[args_cli.algorithm])
    # inference only -- no exploration, no learning
    agent_cfg["agent"]["random_timesteps"] = 0
    agent_cfg["agent"]["exploration_noise"] = None  # watch the deterministic policy, not noisy exploration
    agent_cfg["trainer"]["timesteps"] = args_cli.steps

    env = ManagerBasedRLEnv(cfg=env_cfg)
    # See train.py's matching fix for why: TD3 unconditionally clamps to
    # action_space bounds when exploration_noise is set, and
    # ManagerBasedRLEnv's action_space defaults to unbounded (-inf, inf).
    # Harmless for PPO.
    import gymnasium as gym
    action_dim = env.single_action_space.shape[0]
    env.single_action_space = gym.spaces.Box(low=-3.0, high=3.0, shape=(action_dim,), dtype="float32")
    env.action_space = gym.vector.utils.batch_space(env.single_action_space, env.num_envs)
    env = SkrlVecEnvWrapper(env, ml_framework="torch")

    runner = Runner(env, agent_cfg)
    runner.agent.load(checkpoint)
    runner.agent.enable_training_mode(False)

    obs, _ = env.reset()
    with torch.inference_mode():
        for step in range(args_cli.steps):
            actions = runner.agent.act(obs, None, timestep=0, timesteps=0)[0]
            obs, rew, _terminated, _truncated, _info = env.step(actions)
            if step % 100 == 0:
                print(f"[INFO] step={step} reward_mean={rew.mean().item():.3f}")

    print("[INFO] Done -- window stays open, Ctrl+C to exit.")
    while simulation_app.is_running():
        simulation_app.update()


if __name__ == "__main__":
    main()
    simulation_app.close()
