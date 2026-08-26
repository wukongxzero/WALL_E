from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import argparse

from isaacsim.asset.importer.urdf import _urdf

parser = argparse.ArgumentParser()
parser.add_argument("input", help="Path to URDF file")
parser.add_argument("output", help="Path to output USD file")
parser.add_argument("--merge-joints", action="store_true")
args = parser.parse_args()

config = _urdf.ImportConfig()
config.merge_fixed_joints = args.merge_joints
config.fix_base = False
config.import_inertia_tensor = True
config.distance_scale = 1.0
config.density = 0.0
config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY
config.default_drive_strength = 1047.19751
config.default_position_drive_damping = 52.35988

result, prim_path = _urdf.import_urdf(args.input, args.output, config)

if result:
    print(f"[OK] USD saved to: {args.output}")
else:
    print("[ERROR] Conversion failed")

simulation_app.close()
