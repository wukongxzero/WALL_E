# URDF / Isaac Sim import issues

Issues found and fixed while getting `wall_e.urdf.xacro` importing cleanly
into Isaac Sim 6.0. Kept here so the same class of bug is faster to spot
next time (next robot, next importer version).

## 1. Wheels rendered wrong / joint DriveAPI broke traction

Isaac's URDF importer doesn't reliably preserve `rpy` set on a `<visual>` or
`<collision>` `<origin>` inside a link. The cylinder wheels had
`rpy="1.5707 0 0"` on the visual origin to lay them flat, and Isaac lost the
rotation.

**Fix:** move the RPY off the visual/collision origin and onto the *joint*
origin that attaches the wheel to the base. That rotates the whole link
(which Isaac imports correctly) instead of rotating the mesh inside the link.

Side effect: the joint `<axis>` also changes — after rotating the link 90°
around X, the roll direction becomes the link's Z-axis instead of Y-axis, so
`axis xyz="0 1 0"` becomes `axis xyz="0 0 1"`.

## 2. Massless placeholder links caused negative-mass warnings

Frame-only links used purely as coordinate references (`base_footprint`,
`imu_link`, `camera_depth_optical_frame`, `camera_color_optical_frame`) were
defined as empty self-closing tags with no `<inertial>`. Gazebo treats these
as pure transforms and is fine with it; Isaac's importer treats every link
as a rigid body regardless, and invents garbage inertia when `<inertial>` is
missing — hence "negative mass, invalid inertia {1,1,1}" warnings.

**Fix:** add a tiny explicit inertial to each: `mass=0.001`, diagonal
inertia `1e-6`. Physically negligible, numerically well-defined.

## 3. Two joints were missing from the xacro entirely

`camera_color_optical_joint` and `imu_joint` weren't declared. The links
existed but nothing connected them to a parent, producing
`PxJoint::setActors: at least one actor must be non-static` errors and
orphaned links in the articulation tree.

**Fix:** add the fixed joints explicitly. This was a bug in the xacro
itself, not an Isaac quirk — Gazebo tolerates orphaned links more
gracefully, which is why it went unnoticed there.

## 4. Tracks were boxes instead of cylinders

A prior workaround for issue #1 had changed the tracks from cylinders to
boxes. Boxes "drive" via friction sliding, not rolling — fine for a demo,
wrong for anything where wheel physics matters.

**Fix:** switched back to cylinders once #1 was properly fixed via the
joint-RPY trick.

## 5. Scene vs. robot USDs got confused

Multiple `wall_e_*.usd` files scattered across `usd/` and inside a nested
`wall_e.usd/` folder, with no clear "robot only" vs. "scene with
ground/environment" naming. Drive scripts referenced whichever file had
been saved most recently, which changed silently. Not a URDF bug, but a
project-hygiene issue that made every other problem harder to diagnose.

## 6. Ground plane not saved into the scene USD

Importing the URDF into a fresh stage and adding
Create → Physics → Ground Plane didn't persist on save.

**Fix (pending):** either create the ground plane programmatically in the
drive script after `open_stage`, or re-save the scene with the ground
explicitly included.
