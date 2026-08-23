"""
Check whether a specific PointInstancer instance (e.g. Block_tree_279) actually
occludes a camera position — correctly, per-instance, unlike the earlier check
which called ComputeWorldBound() on the instancer prim itself.

Why the earlier check was wrong: ComputeWorldBound() on a PointInstancer prim
returns the UNION bounding box of every instance it contains, not the box for
one instance. A ray/point test against that aggregate box tells you almost
nothing about whether one specific instance (out of possibly hundreds) is in
the way — it just tells you whether you're anywhere near the whole cluster.

Correct approach: PointInstancer.ComputeInstanceTransformsAtTime() gives the
real per-instance local transform (already composed from positions/
orientations/scales/protoIndices — don't hand-roll quaternion math for this).
Combine that with (a) the instancer's own prim-to-world transform and (b) the
prototype's own local bound, to get instance 279's actual world-space AABB.

Run with: ~/WALL_E/isaac-sim/python.sh check_instancer_occlusion.py
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import UsdGeom, Gf, Usd
from isaacsim.core.experimental.utils.stage import open_stage
import omni.usd

SCENE_USD = "/home/wukong/WALL_E/simulation/usd/wall_e_scene.usd"

# Fill these in for the actual failing case
INSTANCER_PATH = "/World/REPLACE_WITH_ACTUAL_INSTANCER_PATH"
INSTANCE_INDEX = 279
CAMERA_TEST_POS = Gf.Vec3d(-20.0, 31.8, 0.3)


def instance_world_bbox(stage, instancer_path, index, time=Usd.TimeCode.Default()):
    instancer_prim = stage.GetPrimAtPath(instancer_path)
    instancer = UsdGeom.PointInstancer(instancer_prim)
    if not instancer:
        raise RuntimeError(f"{instancer_path} is not a valid PointInstancer")

    # Per-instance local transform, correctly composed by USD itself.
    instance_transforms = instancer.ComputeInstanceTransformsAtTime(
        time, time, UsdGeom.PointInstancer.ProtoXformInclusion.IncludeProtoXform
    )
    if index >= len(instance_transforms):
        raise IndexError(f"Instance {index} out of range (instancer has {len(instance_transforms)})")
    instance_xform = instance_transforms[index]

    # Which prototype this instance actually uses.
    proto_indices = instancer.GetProtoIndicesAttr().Get(time)
    prototypes_rel = instancer.GetPrototypesRel().GetForwardedTargets()
    proto_path = prototypes_rel[proto_indices[index]]
    proto_prim = stage.GetPrimAtPath(proto_path)

    # Prototype's own LOCAL bound (not world — we apply our own instance transform).
    proto_bbox_cache = UsdGeom.BBoxCache(time, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    local_bound = proto_bbox_cache.ComputeLocalBound(proto_prim)

    # Instancer's own prim-to-world transform (in case the instancer itself is parented/offset).
    instancer_xformable = UsdGeom.Xformable(instancer_prim)
    instancer_to_world = instancer_xformable.ComputeLocalToWorldTransform(time)

    full_transform = instance_xform * instancer_to_world
    world_bound = local_bound.ComputeAlignedRange()
    world_bound = Gf.Range3d(
        full_transform.Transform(world_bound.GetMin()),
        full_transform.Transform(world_bound.GetMax()),
    )
    return world_bound, proto_path


def main():
    open_stage(SCENE_USD)
    stage = omni.usd.get_context().get_stage()

    bbox, proto_path = instance_world_bbox(stage, INSTANCER_PATH, INSTANCE_INDEX)
    print(f"[INFO] Instance {INSTANCE_INDEX} uses prototype: {proto_path}")
    print(f"[INFO] Instance {INSTANCE_INDEX} correct world-space AABB: min={bbox.GetMin()} max={bbox.GetMax()}")

    contains_point = bbox.Contains(CAMERA_TEST_POS)
    print(f"[INFO] Camera test position {CAMERA_TEST_POS} inside this instance's real bbox: {contains_point}")

    # For comparison: what the old (wrong) check actually saw.
    instancer_prim = stage.GetPrimAtPath(INSTANCER_PATH)
    aggregate_bbox = UsdGeom.Boundable(instancer_prim).ComputeWorldBound(
        Usd.TimeCode.Default(), UsdGeom.Tokens.default_
    ).ComputeAlignedRange()
    print(f"[INFO] OLD (aggregate, all instances) bbox: min={aggregate_bbox.GetMin()} max={aggregate_bbox.GetMax()}")
    print("[INFO] The aggregate box being huge/containing the point tells you nothing about instance 279 specifically.")

    simulation_app.close()


if __name__ == "__main__":
    main()
