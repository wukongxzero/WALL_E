"""
WALL-E Isaac Sim — Scene 1: Load robot, add ground plane, verify it spawns.

Run with:
    ~/.conda/envs/isaacenv/bin/isaacsim --exec load_wall_e.py

What this does:
- Launches Isaac Sim headless=False (GUI opens)
- Adds a ground plane
- Spawns WALL-E from the existing USD file
- Runs the simulation so you can see the robot sitting on the ground
- Press Ctrl+C to exit
"""

from isaacsim import SimulationApp

# Launch the GUI — set headless=True for no window
sim_app = SimulationApp({"headless": False})

import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.objects import GroundPlane
from omni.isaac.core.robots import Robot
import omni.usd
import numpy as np

# Path to your USD file
WALL_E_USD_PATH = "/home/wukong/WALL_E/usd/wall_e.usd"

def main():
    # Create the world — handles physics, rendering, stepping
    world = World(stage_units_in_meters=1.0)

    # Add a ground plane so the robot doesn't fall forever
    world.scene.add_default_ground_plane()

    # Add WALL-E — spawns at origin, 0.2m above ground so it settles naturally
    robot = world.scene.add(
        Robot(
            prim_path="/World/wall_e",
            name="wall_e",
            usd_path=WALL_E_USD_PATH,
            position=np.array([0.0, 0.0, 0.2]),
        )
    )

    # Initialize the world (sets up physics, loads assets)
    world.reset()

    print("\nWALL-E loaded. Running simulation...")
    print("Joint names:", robot.dof_names)
    print("Press Ctrl+C to exit.\n")

    step = 0
    while sim_app.is_running():
        world.step(render=True)

        # Print robot position every 100 steps so you can see it settling
        if step % 100 == 0:
            pos, rot = robot.get_world_pose()
            print(f"Step {step:5d} | position: {pos}")

        step += 1

    sim_app.close()

if __name__ == "__main__":
    main()
