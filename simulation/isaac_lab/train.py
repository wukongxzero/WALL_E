"""Train either robot on the shared navigation task with skrl (PPO or TD3).

Deliberately simpler than Isaac Lab's own scripts/reinforcement_learning/
skrl/train.py: that script is built around gym.register + Hydra task
discovery for Isaac Lab's own multi-task CLI tooling, which is overkill for
two robots living outside isaaclab_tasks entirely. This constructs the env
directly from our own EnvCfg classes (same pattern test_asro_navigation.py /
test_wall_e_navigation.py already use) and drives skrl's real Runner
machinery directly -- no gym.make, no Hydra, otherwise the same training
loop, just verified end to end against Isaac-Cartpole-Direct-v0 first.

rsl_rl was tried first (see agents/skrl_ppo_cfg.yaml's header comment for
why it doesn't work in this environment) -- skrl is the one that actually
trains here.

PPO (on-policy, rollout-based) and TD3 (off-policy, replay-buffer-based)
have different config shapes -- PPO's max_iterations maps to
timesteps = iterations * rollouts, TD3 has no "rollouts" concept and maps
1:1 to timesteps (every env step is a potential learning step off a replay
buffer, not gated behind collecting a full rollout first).

This is Layer 4 in testing_notes.txt: the one test that can actually answer
whether either robot can learn the task, rather than continuing to guess
from a hand-rolled heuristic controller.

Run with:
  ./isaaclab.sh -p <this file> --robot wall_e --headless
  ./isaaclab.sh -p <this file> --robot asro --algorithm td3 --headless --max_iterations 500
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train a robot on the shared navigation task.")
parser.add_argument("--robot", type=str, choices=["asro", "wall_e"], required=True)
parser.add_argument("--algorithm", type=str, choices=["ppo", "td3"], default="ppo")
parser.add_argument("--num_envs", type=int, default=None, help="Override scene.num_envs.")
parser.add_argument("--max_iterations", type=int, default=None,
                     help="Override total timesteps. PPO: iterations * rollouts. TD3: 1:1 with env steps.")
parser.add_argument("--seed", type=int, default=None, help="Override agent/env seed.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import time
from datetime import datetime

sys.path.insert(0, "/home/wukong/WALL_E/simulation/isaac_lab")

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.utils.io import dump_yaml, load_yaml
from isaaclab_rl.skrl import SkrlVecEnvWrapper
from skrl.utils.runner.torch import Runner

AGENT_CFG_PATHS = {
    "ppo": "/home/wukong/WALL_E/simulation/isaac_lab/agents/skrl_ppo_cfg.yaml",
    "td3": "/home/wukong/WALL_E/simulation/isaac_lab/agents/skrl_td3_cfg.yaml",
}


def main():
    if args_cli.robot == "asro":
        from asro_navigation_env_cfg import AsroNavigationEnvCfg as EnvCfg
    else:
        from wall_e_navigation_env_cfg import WallENavigationEnvCfg as EnvCfg

    env_cfg = EnvCfg()
    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs

    agent_cfg = load_yaml(AGENT_CFG_PATHS[args_cli.algorithm])
    agent_cfg["agent"]["experiment"]["directory"] = f"{args_cli.robot}_navigation"
    if args_cli.max_iterations is not None:
        if args_cli.algorithm == "ppo":
            agent_cfg["trainer"]["timesteps"] = args_cli.max_iterations * agent_cfg["agent"]["rollouts"]
        else:
            agent_cfg["trainer"]["timesteps"] = args_cli.max_iterations
    if args_cli.seed is not None:
        agent_cfg["seed"] = args_cli.seed
    agent_cfg["trainer"]["close_environment_at_exit"] = False

    # certain randomizations happen at environment init, so the seed has to
    # be set before construction -- same ordering isaaclab's own train.py uses
    env_cfg.seed = agent_cfg["seed"]

    log_root_path = os.path.abspath(os.path.join("logs", "skrl", agent_cfg["agent"]["experiment"]["directory"]))
    log_dir = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + f"_{args_cli.algorithm}_torch"  # noqa: DTZ005 -- human-readable local-time log folder name, not a real timestamp comparison
    agent_cfg["agent"]["experiment"]["directory"] = log_root_path
    agent_cfg["agent"]["experiment"]["experiment_name"] = log_dir
    log_dir = os.path.join(log_root_path, log_dir)

    print(f"[INFO] robot={args_cli.robot} num_envs={env_cfg.scene.num_envs} "
          f"timesteps={agent_cfg['trainer']['timesteps']} logging to {log_dir}")

    env_cfg.log_dir = log_dir
    env = ManagerBasedRLEnv(cfg=env_cfg)
    # ManagerBasedRLEnv leaves action_space unbounded (-inf, inf) -- fine for
    # PPO, but TD3 unconditionally clamps to the action space bounds when
    # exploration_noise / smooth_regularization_noise are enabled
    # (skrl/agents/torch/td3/td3.py), and compute_space_limits() returns
    # None for both bounds the instant any dimension is unbounded, which
    # crashes torch.clamp. action_space is a plain settable instance
    # attribute (isaaclab/envs/manager_based_rl_env.py), not a computed
    # property, so this is a real fix, not a workaround: +/-3.0 gives
    # headroom over the ~unit-scale raw actions JointVelocityActionCfg's
    # own `scale` then converts to physical velocity.
    import gymnasium as gym
    action_dim = env.single_action_space.shape[0]
    env.single_action_space = gym.spaces.Box(low=-3.0, high=3.0, shape=(action_dim,), dtype="float32")
    env.action_space = gym.vector.utils.batch_space(env.single_action_space, env.num_envs)
    env = SkrlVecEnvWrapper(env, ml_framework="torch")

    os.makedirs(os.path.join(log_dir, "params"), exist_ok=True)
    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)

    runner = Runner(env, agent_cfg)

    start_time = time.time()
    try:
        runner.run()
        print(f"Training time: {round(time.time() - start_time, 2)} seconds")

        # skrl only saves checkpoints at checkpoint_interval multiples during
        # training, so save a final checkpoint to ensure at least one exists
        total_timesteps = agent_cfg["trainer"]["timesteps"]
        os.makedirs(os.path.join(log_dir, "checkpoints"), exist_ok=True)
        runner.agent.write_checkpoint(timestep=total_timesteps, timesteps=total_timesteps)
        print(f"[INFO] Saved final agent checkpoint to: {log_dir}/checkpoints")
        env.close()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
    simulation_app.close()
