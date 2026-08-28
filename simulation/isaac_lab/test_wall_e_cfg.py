"""Verify WALL_E_CFG actually spawns and drives correctly in Isaac Lab.

Mirrors test_asro_cfg.py — confirms the ArticulationCfg is structurally
correct (right joint names, actually drives) before building a scene/
manager env on top of it.

Run with: ./isaaclab.sh -p <this file> --headless
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Verify WALL_E_CFG spawns and drives.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sys

import isaaclab.sim as sim_utils
import torch
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

sys.path.insert(0, "/home/wukong/WALL_E/simulation/isaac_lab")
from wall_e_cfg import TRACK_JOINT_NAMES, WALL_E_CFG


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([3.0, 3.0, 3.0], [0.0, 0.0, 0.2])

    # GroundPlaneCfg's default usd_path points at a Nucleus-hosted asset that
    # isn't resolvable from this machine (omni.client asset resolver fails
    # even though plain HTTPS to the same host works) -- same issue
    # test_asro_cfg.py would hit. Raw USD instead, no remote asset, no
    # Isaac-version-specific convenience wrapper (this IsaacLab's bundled
    # Isaac Sim under _isaac_sim doesn't have isaacsim.core.experimental).
    stage = sim.stage
    ground = UsdGeom.Cube.Define(stage, "/World/defaultGroundPlane")
    ground.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(ground).SetScale(Gf.Vec3f(100.0, 100.0, 0.1))
    UsdGeom.XformCommonAPI(ground).SetTranslate(Gf.Vec3d(0.0, 0.0, -0.05))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    UsdLux.DistantLight.Define(stage, "/World/Light").CreateIntensityAttr(3000.0)

    wall_e_cfg = WALL_E_CFG.copy()
    wall_e_cfg.prim_path = "/World/WallE"
    robot = Articulation(cfg=wall_e_cfg)

    sim.reset()
    print(f"[INFO] Joint names found by Isaac Lab: {robot.data.joint_names}")
    print(f"[INFO] Expected track joints: {TRACK_JOINT_NAMES}")
    missing = set(TRACK_JOINT_NAMES) - set(robot.data.joint_names)
    if missing:
        print(f"[ERROR] Missing expected joints: {missing}")
    else:
        print("[INFO] All expected track joints present.")

    sim_dt = sim.get_physics_dt()
    track_ids = [robot.data.joint_names.index(n) for n in TRACK_JOINT_NAMES]

    for step in range(60):
        vel_target = torch.zeros_like(robot.data.joint_vel.torch)
        for tid in track_ids:
            vel_target[:, tid] = 5.0  # rad/s forward spin, both tracks
        robot.set_joint_velocity_target_index(target=vel_target, joint_ids=track_ids)
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
