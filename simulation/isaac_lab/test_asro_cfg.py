"""Verify ASRO_CFG actually spawns and drives correctly in Isaac Lab.

Adapted from Isaac Lab's own scripts/tutorials/01_assets/run_articulation.py
minimal pattern. Not a real training env yet — just confirms the
ArticulationCfg is structurally correct before building a scene/manager
env on top of it.

Run with: ./isaaclab.sh -p <this file> --headless
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Verify ASRO_CFG spawns and drives.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sys

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext

sys.path.insert(0, "/home/wukong/WALL_E/simulation/isaac_lab")
from asro_cfg import ASRO_CFG, WHEEL_JOINT_NAMES


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([3.0, 3.0, 3.0], [0.0, 3.0, 0.3])

    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/defaultGroundPlane", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)

    asro_cfg = ASRO_CFG.copy()
    asro_cfg.prim_path = "/World/Asro"
    robot = Articulation(cfg=asro_cfg)

    sim.reset()
    print(f"[INFO] Joint names found by Isaac Lab: {robot.data.joint_names}")
    print(f"[INFO] Expected wheel joints: {WHEEL_JOINT_NAMES}")
    missing = set(WHEEL_JOINT_NAMES) - set(robot.data.joint_names)
    if missing:
        print(f"[ERROR] Missing expected joints: {missing}")
    else:
        print("[INFO] All expected wheel joints present.")

    sim_dt = sim.get_physics_dt()
    wheel_ids = [robot.data.joint_names.index(n) for n in WHEEL_JOINT_NAMES]

    for step in range(60):
        vel_target = torch.zeros_like(robot.data.joint_vel.torch)
        for wid in wheel_ids:
            vel_target[:, wid] = 200.0  # deg/s-equivalent forward spin, all wheels
        robot.set_joint_velocity_target_index(target=vel_target, joint_ids=wheel_ids)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim_dt)

        if step % 20 == 0:
            pos = robot.data.root_pos_w[0].tolist()
            print(f"[INFO] step={step} pos={pos}")

    final_pos = robot.data.root_pos_w[0].tolist()
    print(f"[INFO] Final position: {final_pos}")
    print("[INFO] Test complete.")


if __name__ == "__main__":
    main()
    simulation_app.close()
