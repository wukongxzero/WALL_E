"""Verify AsroSceneCfg actually spawns multiple parallel ASRo copies and
drives them all at once. Adapted from Isaac Lab's own
scripts/tutorials/02_scene/create_scene.py pattern.

Run with: ./isaaclab.sh -p <this file> --num_envs 4 --headless
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Verify AsroSceneCfg with multiple parallel envs.")
parser.add_argument("--num_envs", type=int, default=4, help="Number of parallel ASRo environments.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sys

import isaaclab.sim as sim_utils
import torch
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext

sys.path.insert(0, "/home/wukong/WALL_E/simulation/isaac_lab")
from asro_cfg import WHEEL_JOINT_NAMES
from asro_scene_cfg import AsroSceneCfg


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([8.0, 8.0, 8.0], [0.0, 0.0, 0.0])

    scene_cfg = AsroSceneCfg(num_envs=args_cli.num_envs, env_spacing=4.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    robot = scene["asro"]
    print(f"[INFO] Spawned {args_cli.num_envs} parallel ASRo environments.")
    print(f"[INFO] Robot data shape (num_envs, num_joints): {robot.data.joint_pos.shape}")
    print(f"[INFO] Env origins:\n{scene.env_origins}")

    sim_dt = sim.get_physics_dt()
    wheel_ids = [robot.data.joint_names.index(n) for n in WHEEL_JOINT_NAMES]

    for step in range(60):
        # Every env's ASRo gets driven the same way — this is what "parallel"
        # actually means: one action, applied identically to num_envs copies
        # at once. A real policy would output different actions per env.
        vel_target = torch.zeros_like(robot.data.joint_vel.torch)
        for wid in wheel_ids:
            vel_target[:, wid] = 200.0
        robot.set_joint_velocity_target_index(target=vel_target, joint_ids=wheel_ids)
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        if step % 20 == 0:
            positions = robot.data.root_pos_w.tolist()
            print(f"[INFO] step={step} positions (all envs)={positions}")

    print("[INFO] Final positions:", robot.data.root_pos_w.tolist())
    print("[INFO] Test complete.")


if __name__ == "__main__":
    main()
    simulation_app.close()
