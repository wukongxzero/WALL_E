"""
Convert Jackal UGV IGES CAD model to USD via Isaac Sim's bundled CAD
Converter extension (HOOPS-based, supports IGES/STEP/JT/DGN).

Run with: ~/WALL_E/isaac-sim/python.sh ~/WALL_E/isaac_sim/convert_jackal.py
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import asyncio
import isaacsim.core.experimental.utils.app as app_utils

app_utils.enable_extension("omni.kit.converter.cad")
simulation_app.update()

import omni.kit.asset_converter

INPUT_PATH = "/home/wukong/Downloads/jackal_input.iges"
OUTPUT_PATH = "/home/wukong/Downloads/jackal.usd"


async def convert():
    converter_manager = omni.kit.asset_converter.get_instance()
    context = omni.kit.asset_converter.AssetConverterContext()
    task = converter_manager.create_converter_task(INPUT_PATH, OUTPUT_PATH, None, context)
    success = await task.wait_until_finished()
    if success:
        print(f"[INFO] Converted successfully -> {OUTPUT_PATH}")
    else:
        print(f"[ERROR] Conversion failed: status={task.get_status()} error={task.get_error_message()}")


asyncio.get_event_loop().run_until_complete(convert())
simulation_app.close()
