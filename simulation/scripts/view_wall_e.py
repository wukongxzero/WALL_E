"""Open the existing WALL-E USD in Isaac Lab's GUI.

Usage:
    cd /home/wukong/IsaacLab
    ./isaaclab.sh -p /home/wukong/WALL_E/simulation/scripts/view_wall_e.py
"""

import argparse
import contextlib

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="View WALL-E USD in Isaac Lab.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
import omni.kit.app
from isaaclab.sim import SimulationContext

USD_PATH = "/home/wukong/WALL_E/simulation/usd/wall_e.usd/wall_e.usda"


def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.01)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[2.0, 2.0, 1.5], target=[0.0, 0.0, 0.5])

    # Ground plane and lighting
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/GroundPlane", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)

    # Load the existing USD
    cfg = sim_utils.UsdFileCfg(usd_path=USD_PATH)
    cfg.func("/World/WallE", cfg)

    sim.reset()
    print("[INFO] WALL-E loaded. Running simulation — Ctrl+C to quit.")

    app = omni.kit.app.get_app_interface()
    with contextlib.suppress(KeyboardInterrupt):
        while app.is_running():
            sim.step()


if __name__ == "__main__":
    main()
    simulation_app.close()
