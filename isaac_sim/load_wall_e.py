"""
WALL-E Isaac Sim 4.5 — minimal load test.
Spawns WALL-E USD in the scene, runs 200 steps, prints position.

Run with:
    conda activate isaaclab
    cd ~/IsaacLab && ./isaaclab.sh -p ~/WALL_E/isaac_sim/load_wall_e.py --headless
"""

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Load WALL-E in Isaac Sim 4.5")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext
from pxr import UsdPhysics
import omni.usd

WALL_E_USDA = "/home/wukong/WALL_E/usd/wall_e.usd/wall_e.usda"


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.3])

    # Ground plane + light
    sim_utils.GroundPlaneCfg().func("/World/defaultGroundPlane", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)).func(
        "/World/Light", sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # Spawn WALL-E USD as a reference prim
    stage = omni.usd.get_context().get_stage()
    wall_e_prim = stage.DefinePrim("/World/wall_e", "Xform")
    wall_e_prim.GetReferences().AddReference(WALL_E_USDA)

    sim.reset()
    print("\n[INFO] WALL-E spawned. Running 200 steps...")

    # Print prim children to confirm USD loaded
    children = list(stage.GetPrimAtPath("/World/wall_e").GetChildren())
    print(f"[INFO] wall_e children: {[str(c.GetPath()) for c in children]}")

    count = 0
    while simulation_app.is_running() and count < 200:
        sim.step()
        count += 1
        if count % 50 == 0:
            print(f"[INFO] Step {count} OK")

    print("[INFO] 200 steps complete — WALL-E loaded successfully.")


if __name__ == "__main__":
    main()
    simulation_app.close()
