"""
Open wall_e_scene.usd and reference crawler_mower_chassis.usda into it for
manual inspection. No control loop — just loads both and idles.

GUI File->Import is broken on this machine (Nucleus sync misconfigured —
every import tries to copy to omniverse:/// and fails with ERROR_CONNECTION,
even for local files). Referencing directly via script sidesteps that.

Run with: ~/WALL_E/isaac-sim/python.sh ~/WALL_E/isaac_sim/open_scene_with_mower.py
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.usd
from isaacsim.core.experimental.utils.stage import open_stage

SCENE_USD = "/home/wukong/WALL_E/simulation/usd/wall_e_scene.usd"
MOWER_USD = "/home/wukong/WALL_E/simulation/usd/crawler_mower_chassis.usda"

open_stage(SCENE_USD)
stage = omni.usd.get_context().get_stage()

mower_prim = stage.DefinePrim("/World/crawler_mower", "Xform")
mower_prim.GetReferences().AddReference(MOWER_USD)

simulation_app.update()
print(f"[INFO] Opened {SCENE_USD} with {MOWER_USD} referenced at /World/crawler_mower — idling for manual inspection.")

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
