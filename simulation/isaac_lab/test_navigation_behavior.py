"""Deeper validation of the shared navigation env, for either robot: does
reward actually track distance-to-goal (not just "doesn't crash"), does
time_out reset fire and actually reset spawn position, does it stay
numerically stable over multiple episodes.

Unlike test_asro_navigation.py/test_wall_e_navigation.py (random actions,
30 steps, only checks shapes), this drives a simple proportional
pursuit controller toward the commanded goal using the command term's own
body-frame target (env.command_manager), so error_pos should trend down
within an episode and reward should trend up to match.

Run with: ./isaaclab.sh -p <this file> --headless --robot asro
          ./isaaclab.sh -p <this file> --headless --robot wall_e
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Validate navigation env reward/reset behavior.")
parser.add_argument("--robot", type=str, choices=["asro", "wall_e"], required=True)
parser.add_argument("--steps", type=int, default=300)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sys

import torch

sys.path.insert(0, "/home/wukong/WALL_E/simulation/isaac_lab")


def main():
    if args_cli.robot == "asro":
        from asro_navigation_env_cfg import AsroNavigationEnvCfg as EnvCfg
        # WHEEL_JOINT_NAMES order: fl, fr, ml, mr, rl, rr -- left = 0,2,4, right = 1,3,5

        left_idx, right_idx = [0, 2, 4], [1, 3, 5]

    else:

        from wall_e_navigation_env_cfg import WallENavigationEnvCfg as EnvCfg

        # TRACK_JOINT_NAMES order: left, right
        left_idx, right_idx = [0], [1]


    from isaaclab.envs import ManagerBasedRLEnv


    cfg = EnvCfg()
    cfg.scene.num_envs = 4
    env = ManagerBasedRLEnv(cfg=cfg)

    n_actions = env.action_space.shape[1]
    obs, _ = env.reset()

    print(f"[INFO] robot={args_cli.robot} num_envs={cfg.scene.num_envs} n_actions={n_actions}")

    cmd_term = env.command_manager.get_term("pose_command")
    episode_len = None
    prev_terminated = torch.zeros(cfg.scene.num_envs, dtype=torch.bool)
    prev_pos = env.scene["robot"].data.root_pos_w[:, :2].clone()
    reset_checked = False
    error_history = []
    reward_history = []
    nan_seen = False

    for step in range(args_cli.steps):
        command = env.command_manager.get_command("pose_command")  # (num_envs, 4): [x_b, y_b, z_b, heading_b]
        x_b, y_b = command[:, 0], command[:, 1]

        forward = torch.clamp(x_b, -1.0, 1.0)
        turn = torch.clamp(y_b, -1.0, 1.0)

        actions = torch.zeros((cfg.scene.num_envs, n_actions), device=env.device)
        for i in left_idx:
            actions[:, i] = torch.clamp(forward - turn, -1.0, 1.0)
        for i in right_idx:
            actions[:, i] = torch.clamp(forward + turn, -1.0, 1.0)

        obs, rew, terminated, truncated, _info = env.step(actions)
        # mdp.time_out populates `truncated` (Gymnasium convention: time
        # limits are truncation, not a true MDP terminal state) -- checking
        # `terminated` alone here would never see the episode boundary.
        done = terminated | truncated

        if torch.isnan(rew).any() or torch.isnan(obs["policy"]).any():
            nan_seen = True
            print(f"[ERROR] step={step} NaN detected in reward or observation")

        err = cmd_term.metrics["error_pos"][0].item()
        error_history.append(err)
        reward_history.append(rew[0].item())

        # Detect the first reset on env 0 to check episode length + spawn teleport
        if done[0] and not prev_terminated[0] and episode_len is None:
            episode_len = step
        if prev_terminated[0] and not reset_checked:
            new_pos = env.scene["robot"].data.root_pos_w[0, :2]
            jump = torch.linalg.norm(new_pos - prev_pos[0]).item()
            print(f"[INFO] step={step}: post-reset position jump = {jump:.3f} m (should be nonzero -- spawn re-randomized)")
            reset_checked = True
        prev_terminated = done.clone()
        prev_pos = env.scene["robot"].data.root_pos_w[:, :2].clone()

        if step % 30 == 0:
            print(f"[INFO] step={step} error_pos={err:.3f} reward={rew[0].item():.3f} done={done.tolist()}")

    print(f"[INFO] episode length observed (env 0 first terminate): {episode_len} steps "
          f"(expected ~{int(cfg.episode_length_s / (cfg.sim.dt * cfg.decimation))})")

    # Reward-tracks-distance sanity check: within any contiguous non-reset stretch,
    # error going down should correlate with reward going up.
    import numpy as np
    err_arr = np.array(error_history)
    rew_arr = np.array(reward_history)
    # only look at the first episode's worth of steps (before any reset) to avoid
    # mixing pre/post-reset discontinuities into the correlation
    cutoff = episode_len if episode_len else len(err_arr)
    if cutoff > 5:
        corr = np.corrcoef(err_arr[:cutoff], rew_arr[:cutoff])[0, 1]
        print(f"[INFO] correlation(error_pos, reward) over first episode: {corr:.3f} (expect negative -- reward up as error down)")

    print(f"[INFO] NaN seen: {nan_seen}")
    print(f"[INFO] error_pos: start={error_history[0]:.3f} end_of_first_ep={error_history[min(cutoff-1, len(error_history)-1)]:.3f}")
    print("[INFO] Test complete.")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
