"""One-off diagnostic: inspect the bundled RealSense D455 USD asset's prim
hierarchy so we know the actual RGB/Depth camera prim paths before wiring
ROS2 publishers to them. Run with: ~/isaac-sim/python.sh inspect_d455.py
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from isaacsim.storage.native import get_assets_root_path
from pxr import Usd

stage = omni.usd.get_context().get_stage()
prim = stage.DefinePrim("/World/D455Probe", "Xform")
usd_path = get_assets_root_path() + "/Isaac/Sensors/RealSense/D455/rsd455.usd"
print(f"[DEBUG] referencing: {usd_path}")
prim.GetReferences().AddReference(usd_path)
simulation_app.update()
simulation_app.update()

for p in Usd.PrimRange(prim):
    type_name = p.GetTypeName()
    if type_name in ("Camera",) or "camera" in p.GetName().lower():
        attrs = [a.GetName() for a in p.GetAttributes() if "sensor" in a.GetName().lower()]
        print(f"[DEBUG] {p.GetPath()}  type={type_name}  sensor-attrs={attrs}")

simulation_app.close()
