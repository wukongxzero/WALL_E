"""
WALL-E Isaac Sim 6.0 — load test.
Spawns WALL-E USD in the scene, runs 200 steps, prints position.

Run with:
    ~/isaac-sim/python.sh ~/WALL_E/isaac_sim/load_wall_e.py
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.objects import DistantLight, GroundPlane
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Usd

WALL_E_USD = "/home/wukong/WALL_E/usd/wall_e_v2.usd"

# New stage
stage_utils.create_new_stage()
stage = Usd.Stage.Open(stage_utils.get_current_stage_url()) if False else None

# Ground + light
GroundPlane("/World/GroundPlane")
light = DistantLight("/World/DistantLight")
light.set_intensities(3000)

# Spawn WALL-E as a USD reference
import omni.usd
stage = omni.usd.get_context().get_stage()
wall_e = stage.DefinePrim("/World/wall_e", "Xform")
wall_e.GetReferences().AddReference(WALL_E_USD)

SimulationManager.set_physics_dt(1.0 / 60.0)

app_utils.play()
simulation_app.update()

print("\n[INFO] WALL-E spawned. Running simulation (Ctrl+C to stop)...")
children = list(stage.GetPrimAtPath("/World/wall_e").GetChildren())
print(f"[INFO] wall_e children: {[str(c.GetPath()) for c in children]}")

for i in range(1000000):
    SimulationManager.step()
    RenderingManager.render()
    simulation_app.update()
    if i % 50 == 0:
        print(f"[INFO] Step {i} OK")

print("[INFO] Simulation complete.")
app_utils.stop()
simulation_app.close()
