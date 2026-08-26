"""
Convert WALL-E URDF to USD using Isaac Lab's URDF converter.
Produces a correctly structured articulation with:
  - base_link as ArticulationRoot (free-floating, mobile robot)
  - left_track + right_track as separate rigid bodies with revolute joints
  - camera, imu, supports merged into base_link (merge_fixed_links=True)
"""
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Convert WALL-E URDF to USD")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

cfg = UrdfConverterCfg(
    asset_path="/home/wukong/WALL_E/simulation/usd/wall_e.urdf",
    usd_dir="/home/wukong/WALL_E/simulation/usd/",
    usd_file_name="wall_e_v2.usd",
    fix_base=False,
    merge_fixed_joints=True,
    force_usd_conversion=True,
    make_instanceable=False,
    joint_drive=None,
)

print("[INFO] Converting URDF to USD...")
converter = UrdfConverter(cfg)
print(f"[INFO] USD saved to: {converter.usd_path}")

simulation_app.close()
