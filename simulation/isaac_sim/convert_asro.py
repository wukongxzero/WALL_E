"""
Convert ASRo URDF to USD via Isaac Sim's URDFImporter Python API — bypasses
the GUI File->Import entirely, which is broken on this machine (Nucleus sync
misconfigured; every import tries to copy to omniverse:/// and fails with
ERROR_CONNECTION, even for local files already on disk).

Run with: ~/WALL_E/isaac-sim/python.sh ~/WALL_E/isaac_sim/convert_asro.py
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

URDF_PATH = "/home/wukong/WALL_E/simulation/usd/ASRo_URDF_Simple_6_Wheel_Robot.urdf"
USD_OUT_DIR = "/home/wukong/WALL_E/simulation/usd"

config = URDFImporterConfig(
    urdf_path=URDF_PATH,
    usd_path=USD_OUT_DIR,
    fix_base=False,             # mobile robot, base must be free
    merge_fixed_joints=True,    # collapse fixed joints, simpler hierarchy
    allow_self_collision=False,
)

importer = URDFImporter()
output_path = importer.import_urdf(config)
print(f"[INFO] Converted successfully -> {output_path}")

simulation_app.close()
