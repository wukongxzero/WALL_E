"""
Open wall_e_scene.usd directly in the Isaac Sim GUI for manual inspection.
No control loop — just loads the stage and idles so you can look around,
use the Stage panel, etc.

Run with: ~/WALL_E/isaac-sim/python.sh ~/WALL_E/isaac_sim/open_scene.py
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.experimental.utils.stage import open_stage

SCENE_USD = "/home/wukong/WALL_E/simulation/usd/wall_e_scene.usd"

open_stage(SCENE_USD)
simulation_app.update()
print(f"[INFO] Opened {SCENE_USD} — idling for manual inspection.")

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
