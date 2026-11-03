"""Build the diff-drive warehouse robot as a USD articulation.

Must be imported after SimulationApp exists (needs pxr). Layout:
  <root>                    Xform, ArticulationRootAPI, placed at (x, y, yaw)
    chassis                 rigid body: box + 2 frictionless caster spheres
    left_wheel, right_wheel rigid bodies (cylinder), revolute joints about y
    lift                    rigid body plate, prismatic joint along z
    joints/                 joint prims (left_wheel_joint, right_wheel_joint, lift_joint)
Wheel joints have velocity drives (stiffness 0); the lift has a position drive.
Drive targets are plain USD attributes, so a test can set them directly; Day 4
replaces that with an OmniGraph ArticulationController fed by /cmd_vel.
"""
import math

from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

import robot_params as P

WHEEL_JOINTS = ("left_wheel_joint", "right_wheel_joint")


def _vis(stage, path, rgb):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _phys_mat(stage, path, static, dynamic):
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(static)
    api.CreateDynamicFrictionAttr(dynamic)
    api.CreateRestitutionAttr(0.0)
    if static == 0.0:
        # use the minimum of the two materials so the caster really slides
        PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr("min")
    else:
        PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr("max")
    return mat


def _bind(prim, vis=None, phys=None):
    api = UsdShade.MaterialBindingAPI.Apply(prim)
    if vis is not None:
        api.Bind(vis)
    if phys is not None:
        api.Bind(phys, UsdShade.Tokens.weakerThanDescendants, "physics")


def _body(stage, path, pos, mass):
    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
    UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(mass)
    return x


def build_robot(stage, path="/World/Robot", x=0.0, y=0.0, yaw=0.0):
    """Create the robot at floor pose (x, y, yaw rad). Returns the root path."""
    root = UsdGeom.Xform.Define(stage, path)
    root.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    root.AddOrientOp().Set(Gf.Quatf(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim()).CreateEnabledSelfCollisionsAttr(False)

    looks = f"{path}/Looks"
    v_body = _vis(stage, f"{looks}/body", (0.9, 0.5, 0.1))
    v_wheel = _vis(stage, f"{looks}/wheel", (0.1, 0.1, 0.1))
    v_lift = _vis(stage, f"{looks}/lift", (0.6, 0.6, 0.65))
    m_grip = _phys_mat(stage, f"{looks}/rubber", 1.0, 0.9)
    m_slip = _phys_mat(stage, f"{looks}/caster", 0.0, 0.0)
    m_lift = _phys_mat(stage, f"{looks}/lift_pad", 0.9, 0.8)

    # --- chassis (+ casters as extra colliders on the same body) ---
    cz = P.chassis_centre_z()
    chassis = _body(stage, f"{path}/chassis", (0, 0, cz), P.CHASSIS_MASS)
    box = UsdGeom.Cube.Define(stage, f"{path}/chassis/body")
    box.CreateSizeAttr(1.0)
    box.AddScaleOp().Set(Gf.Vec3f(*P.CHASSIS_SIZE))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    _bind(box.GetPrim(), v_body, m_grip)
    for name, sx in (("caster_front", P.CASTER_X), ("caster_back", -P.CASTER_X)):
        s = UsdGeom.Sphere.Define(stage, f"{path}/chassis/{name}")
        s.CreateRadiusAttr(P.CASTER_RADIUS)
        s.AddTranslateOp().Set(Gf.Vec3d(sx, 0, P.CASTER_RADIUS - cz))
        UsdPhysics.CollisionAPI.Apply(s.GetPrim())
        _bind(s.GetPrim(), v_lift, m_slip)

    UsdGeom.Scope.Define(stage, f"{path}/joints")

    # --- wheels ---
    for side, sy in (("left", 1.0), ("right", -1.0)):
        wy = sy * P.WHEEL_SEPARATION / 2
        wheel = _body(stage, f"{path}/{side}_wheel", (0, wy, P.WHEEL_RADIUS), P.WHEEL_MASS)
        cyl = UsdGeom.Cylinder.Define(stage, f"{path}/{side}_wheel/tire")
        cyl.CreateRadiusAttr(P.WHEEL_RADIUS)
        cyl.CreateHeightAttr(P.WHEEL_WIDTH)
        cyl.CreateAxisAttr("Y")
        _bind(cyl.GetPrim(), v_wheel)
        # Collider: an invisible sphere of the wheel radius. PhysX cylinder
        # colliders (convex approximations) made the robot veer/roll on Day 3;
        # a sphere has exactly one smooth contact point.
        col = UsdGeom.Sphere.Define(stage, f"{path}/{side}_wheel/collider")
        col.CreateRadiusAttr(P.WHEEL_RADIUS)
        col.MakeInvisible()
        UsdPhysics.CollisionAPI.Apply(col.GetPrim())
        _bind(col.GetPrim(), None, m_grip)

        j = UsdPhysics.RevoluteJoint.Define(stage, f"{path}/joints/{side}_wheel_joint")
        j.CreateBody0Rel().SetTargets([chassis.GetPath()])
        j.CreateBody1Rel().SetTargets([wheel.GetPath()])
        j.CreateLocalPos0Attr(Gf.Vec3f(0, wy, P.WHEEL_RADIUS - cz))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateAxisAttr("Y")
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(P.WHEEL_DAMPING)
        d.CreateMaxForceAttr(P.WHEEL_MAX_TORQUE)
        d.CreateTargetVelocityAttr(0.0)   # deg/s

    # --- lift plate on a prismatic joint ---
    top = P.chassis_top_z()
    lz = top + P.LIFT_SIZE[2] / 2
    lift = _body(stage, f"{path}/lift", (0, 0, lz), P.LIFT_MASS)
    plate = UsdGeom.Cube.Define(stage, f"{path}/lift/plate")
    plate.CreateSizeAttr(1.0)
    plate.AddScaleOp().Set(Gf.Vec3f(*P.LIFT_SIZE))
    UsdPhysics.CollisionAPI.Apply(plate.GetPrim())
    _bind(plate.GetPrim(), v_lift, m_lift)
    pj = UsdPhysics.PrismaticJoint.Define(stage, f"{path}/joints/lift_joint")
    pj.CreateBody0Rel().SetTargets([chassis.GetPath()])
    pj.CreateBody1Rel().SetTargets([lift.GetPath()])
    pj.CreateLocalPos0Attr(Gf.Vec3f(0, 0, lz - cz))
    pj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    pj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    pj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    pj.CreateAxisAttr("Z")
    pj.CreateLowerLimitAttr(P.LIFT_TRAVEL[0])
    pj.CreateUpperLimitAttr(P.LIFT_TRAVEL[1])
    ld = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), "linear")
    ld.CreateTypeAttr("force")
    ld.CreateStiffnessAttr(5.0e4)
    ld.CreateDampingAttr(2.0e3)
    ld.CreateMaxForceAttr(500.0)
    ld.CreateTargetPositionAttr(0.0)
    return path


def set_wheel_speeds(stage, left_rad_s, right_rad_s, path="/World/Robot"):
    """Set wheel velocity-drive targets (rad/s) via USD (live while playing)."""
    for name, w in zip(WHEEL_JOINTS, (left_rad_s, right_rad_s)):
        prim = stage.GetPrimAtPath(f"{path}/joints/{name}")
        UsdPhysics.DriveAPI(prim, "angular").GetTargetVelocityAttr().Set(P.rad_to_deg(w))


def set_lift_height(stage, z, path="/World/Robot"):
    prim = stage.GetPrimAtPath(f"{path}/joints/lift_joint")
    UsdPhysics.DriveAPI(prim, "linear").GetTargetPositionAttr().Set(float(z))
