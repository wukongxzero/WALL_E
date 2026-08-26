"""
Open wall_e_scene.usd and reference the converted ASRo robot into it for
manual inspection. No control loop — just loads both and idles.

Run with: ~/WALL_E/isaac-sim/python.sh ~/WALL_E/isaac_sim/open_scene_with_asro.py
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.usd
from isaacsim.core.experimental.utils.stage import open_stage
from pxr import UsdGeom

SCENE_USD = "/home/wukong/WALL_E/simulation/usd/wall_e_scene.usd"
ASRO_USD = "/home/wukong/WALL_E/simulation/usd/ASRo_URDF_Simple_6_Wheel_Robot/ASRo_URDF_Simple_6_Wheel_Robot.usda"

open_stage(SCENE_USD)
stage = omni.usd.get_context().get_stage()

asro_prim = stage.DefinePrim("/World/asro", "Xform")
asro_prim.GetReferences().AddReference(ASRO_USD)
UsdGeom.XformCommonAPI(asro_prim).SetTranslate((0.0, 3.0, 0.1))  # offset from WALL-E at origin

simulation_app.update()
print(f"[INFO] Opened {SCENE_USD} with {ASRO_USD} referenced at /World/asro — idling for manual inspection.")

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
