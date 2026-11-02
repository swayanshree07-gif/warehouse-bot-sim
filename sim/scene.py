"""Build the warehouse stage (floor, walls, shelves, boxes) with plain USD/PhysX.

Must be imported after SimulationApp is created (needs pxr from Isaac Sim).
All collision shapes are primitive cubes; boxes are light rigid bodies.
"""
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

import warehouse_layout as L


def _material(stage, path, rgb):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _cube(stage, path, centre, size, material=None):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if material is not None:
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
    return cube


def build_warehouse(stage, root="/World"):
    """Populate `stage` and return a dict with counts of created prims."""
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, root)

    scene = UsdPhysics.Scene.Define(stage, f"{root}/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr(60)

    # Friction material shared by floor/shelves/boxes
    phys_mat = UsdShade.Material.Define(stage, f"{root}/Looks/physics_default")
    pm = UsdPhysics.MaterialAPI.Apply(phys_mat.GetPrim())
    pm.CreateStaticFrictionAttr(0.8)
    pm.CreateDynamicFrictionAttr(0.6)
    pm.CreateRestitutionAttr(0.0)

    m_floor = _material(stage, f"{root}/Looks/floor", (0.55, 0.55, 0.55))
    m_wall = _material(stage, f"{root}/Looks/wall", (0.85, 0.85, 0.8))
    m_shelf = _material(stage, f"{root}/Looks/shelf", (0.15, 0.3, 0.6))
    m_box = _material(stage, f"{root}/Looks/box", (0.75, 0.55, 0.3))

    def bind_phys(prim):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            phys_mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    counts = {"walls": 0, "shelves": 0, "boxes": 0}
    floor = _cube(stage, f"{root}/Floor", (0, 0, -0.05),
                  (L.ROOM_X + 2, L.ROOM_Y + 2, 0.1), m_floor)
    bind_phys(floor.GetPrim())

    UsdGeom.Xform.Define(stage, f"{root}/Walls")
    for name, c, s in L.walls():
        _cube(stage, f"{root}/Walls/{name}", c, s, m_wall)
        counts["walls"] += 1

    UsdGeom.Xform.Define(stage, f"{root}/Shelves")
    sx, sy, sz = L.SHELF_SIZE
    for shelf in L.SHELVES:
        p = _cube(stage, f"{root}/Shelves/{shelf.name}", (shelf.x, shelf.y, sz / 2), (sx, sy, sz), m_shelf)
        bind_phys(p.GetPrim())
        counts["shelves"] += 1

    UsdGeom.Xform.Define(stage, f"{root}/Boxes")
    for b in L.BOXES:
        e = L.BOX_SIZE
        p = _cube(stage, f"{root}/Boxes/{b.name}", (b.x, b.y, b.z), (e, e, e), m_box)
        UsdPhysics.RigidBodyAPI.Apply(p.GetPrim())
        UsdPhysics.MassAPI.Apply(p.GetPrim()).CreateMassAttr(L.BOX_MASS)
        bind_phys(p.GetPrim())
        counts["boxes"] += 1

    light = stage.DefinePrim(f"{root}/Light", "DistantLight")
    light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(3000.0)
    return counts
