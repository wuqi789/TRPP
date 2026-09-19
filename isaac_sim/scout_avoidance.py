#!/usr/bin/env python3
"""Scout Mini + InteriorAgent + ROS 2 bridge simulation for Isaac Sim 4.5."""

import argparse
import math
import os
import time
from pathlib import Path

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--debug-lidar", action="store_true")
    room = parser.add_mutually_exclusive_group()
    room.add_argument("--full-room", dest="full_room", action="store_true")
    room.add_argument("--navigation-room", dest="full_room", action="store_false")
    parser.set_defaults(full_room=True)
    parser.add_argument("--test-seconds", type=float, default=0.0)
    parser.add_argument("--test-curtain", action="store_true")
    parser.add_argument("--start-x", type=float, default=8.2)
    parser.add_argument("--start-y", type=float, default=-3.2)
    parser.add_argument("--start-yaw", type=float, default=0.0)
    parser.add_argument("--arm-mount-x", type=float, default=-0.05)
    parser.add_argument("--arm-mount-y", type=float, default=0.0)
    parser.add_argument("--arm-mount-z", type=float, default=0.13)
    parser.add_argument("--traversal-acceptance", action="store_true")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument(
        "--acceptance-obstacle",
        choices=("curtain", "movable_box", "fixed_box"),
        default="curtain",
    )
    return parser.parse_known_args()[0]


ARGS = parse_args()
THREAD_COUNT = len(os.sched_getaffinity(0))
APP = SimulationApp(
    {
        "headless": ARGS.headless,
        "renderer": "RaytracedLighting",
        "anti_aliasing": 2,
        "width": 1280,
        "height": 720,
        "window_width": 1280,
        "window_height": 800,
        "multi_gpu": False,
        "extra_args": [
            f"--/plugins/carb.tasking.plugin/threadCount={THREAD_COUNT}",
            "--/app/runLoops/main/rateLimitEnabled=true",
            "--/app/runLoops/main/rateLimitFrequency=30",
        ],
    }
)

import carb
import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.replicator.core as rep
import omni.timeline
import omni.usd
import rclpy
import usdrt.Sdf
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from isaacsim.core.api import World
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim, SingleXFormPrim
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from nav_msgs.msg import OccupancyGrid, Path as NavigationPath
from omni.physx.scripts import particleUtils, physicsUtils
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from usd.schema.isaac import ISAAC_NAME_OVERRIDE

from room_occupancy import (
    OPEN_KITCHEN_DOOR_CENTER_Y,
    OPEN_KITCHEN_DOOR_HALF_WIDTH,
    OPEN_KITCHEN_DOOR_TOP_Z,
    intersects_navigation_height,
    is_blocking_other_prim,
    solid_spans_around_openings,
)


WORKSPACE = Path(__file__).resolve().parents[1]
FULL_ROOM_USD = WORKSPACE / "isaacsim-assets/InteriorAgent/kujiale_0021/kujiale_0021.usda"
NAVIGATION_ROOM_USD = WORKSPACE / "isaac_sim/kujiale_0021_navigation.usda"
ROBOT_URDF = WORKSPACE / "isaac_sim/scout_mini_isaac.urdf"
ARM_MJCF = (
    WORKSPACE
    / "src/agx_arm_sim/mujoco/agilex_arm/agilex_piper/piper.xml"
)
ARM_VISUAL_PATCHES = ("link3", "link4", "link5")
ARM_PATH = "/World/Piper"
LIDAR_CONFIG_DIR = WORKSPACE / "isaac_sim"
CURTAIN_TEXTURE = WORKSPACE / "isaac_sim/assets/dog_door_curtain.png"
WHEEL_RADIUS = 0.08
WHEEL_DISTANCE = 0.416503
WHEEL_STATIC_FRICTION = 0.0
WHEEL_DYNAMIC_FRICTION = 0.0
WHEEL_JOINT_NAMES = (
    "front_left_wheel",
    "front_right_wheel",
    "rear_left_wheel",
    "rear_right_wheel",
)
ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 9))
ARM_START = np.array([0.0, 0.8, -1.0, 0.0, 0.287, 0.00, 0.02, -0.02])
ARM_KP = np.array([400.0, 400.0, 400.0, 200.0, 100.0, 100.0, 400.0, 400.0])
ARM_KD = np.array([15.0, 15.0, 15.0, 10.0, 5.0, 5.0, 10.0, 10.0])
# The migrated probe controller advances a feedback-bounded target instead of
# publishing the old full-pose step. Retain enough PhysX damping to suppress
# oscillation while allowing the articulation to follow those small targets.
ARM_DAMPING_SCALE = 0.12
ARM_MAX_EFFORT = np.array([100.0] * 6 + [10.0, 10.0])
COMMAND_TIMEOUT = 0.5
PHYSICS_DT = 1.0 / 60.0
RENDERING_DT = 1.0 / 30.0
CURTAIN_X = 11.49
CURTAIN_CENTER_Y = -3.204
CURTAIN_WIDTH = 0.74
CURTAIN_TOP_Z = 2.02
CURTAIN_BOTTOM_Z = 0.22
CURTAIN_COLUMNS = 21
CURTAIN_ROWS = 46
ACCEPTANCE_BOX_ALONG_M = 1.80
ACCEPTANCE_BOX_LATERAL_M = -0.15
ACCEPTANCE_PRIMS = {
    "curtain": "/World/DoorCurtain/Cloth",
    "movable_box": "/World/TraversalFixtures/AcceptanceBox",
    "fixed_box": "/World/TraversalFixtures/AcceptanceBox",
}


def require_file(path: Path) -> None:
    if not path.is_file():
        raise FileNotFoundError(f"Required file not found: {path}")


def wait_for_stage() -> None:
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        APP.update()


def sanitize_display_primvars(stage: Usd.Stage) -> None:
    """Fix single-value primvars mislabeled as face-varying in the room asset."""
    room = stage.GetPrimAtPath("/World/Room")
    for prim in Usd.PrimRange(room):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        primvars = UsdGeom.PrimvarsAPI(prim)
        for name in ("displayColor", "displayOpacity"):
            primvar = primvars.GetPrimvar(name)
            values = primvar.Get() if primvar else None
            if primvar and primvar.GetInterpolation() == "faceVarying" and values is not None and len(values) == 1:
                primvar.SetInterpolation("constant")


def create_indoor_lights(stage: Usd.Stage) -> int:
    """Add real ceiling lights; room fixtures are decorative meshes only."""
    light_positions = (
        (1.8, -1.8, 2.9),
        (5.3, -1.8, 2.9),
        (8.4, -2.5, 3.0),
        (4.7, 2.7, 2.8),
        (12.9, -0.7, 2.2),
        (13.6, -3.6, 2.2),
    )
    for index, position in enumerate(light_positions):
        light = UsdLux.RectLight.Define(stage, f"/World/IndoorLights/ceiling_{index:02d}")
        light.CreateWidthAttr(2.5)
        light.CreateHeightAttr(2.0)
        light.CreateIntensityAttr(1500.0)
        light.CreateExposureAttr(2.0)
        light.CreateColorAttr(Gf.Vec3f(1.0, 0.86, 0.72))
        light.CreateNormalizeAttr(False)
        UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(*position))
    return len(light_positions)


def create_door_curtain(stage: Usd.Stage, physics_scene_path: str) -> int:
    """Hang a collidable particle-cloth curtain in the doorway ahead of the robot."""
    root = UsdGeom.Xform.Define(stage, "/World/DoorCurtain")
    cloth_path = root.GetPath().AppendChild("Cloth")
    cloth = UsdGeom.Mesh.Define(stage, cloth_path)
    points = []
    triangles = []
    for row in range(CURTAIN_ROWS):
        z = CURTAIN_TOP_Z - (CURTAIN_TOP_Z - CURTAIN_BOTTOM_Z) * row / (CURTAIN_ROWS - 1)
        for column in range(CURTAIN_COLUMNS):
            fraction = column / (CURTAIN_COLUMNS - 1)
            y = CURTAIN_CENTER_Y - CURTAIN_WIDTH / 2.0 + CURTAIN_WIDTH * fraction
            pleat = 0.015 * math.sin(6.0 * math.pi * fraction)
            points.append(Gf.Vec3f(CURTAIN_X + pleat, y, z))
            if row == 0 or column == 0:
                continue
            upper_left = (row - 1) * CURTAIN_COLUMNS + column - 1
            upper_right = upper_left + 1
            lower_left = row * CURTAIN_COLUMNS + column - 1
            lower_right = lower_left + 1
            triangles.extend(
                [upper_left, lower_left, lower_right, upper_left, lower_right, upper_right]
            )

    cloth.CreatePointsAttr().Set(Vt.Vec3fArray(points))
    cloth.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(triangles))
    cloth.CreateFaceVertexCountsAttr().Set(Vt.IntArray([3] * (len(triangles) // 3)))
    cloth.CreateDoubleSidedAttr().Set(True)
    cloth.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    texture_coordinates = [
        Gf.Vec2f(
            column / (CURTAIN_COLUMNS - 1),
            1.0 - row / (CURTAIN_ROWS - 1),
        )
        for row in range(CURTAIN_ROWS)
        for column in range(CURTAIN_COLUMNS)
    ]
    texture_primvar = UsdGeom.PrimvarsAPI(cloth).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
    )
    texture_primvar.Set(Vt.Vec2fArray(texture_coordinates))

    visual_material = UsdShade.Material.Define(
        stage, root.GetPath().AppendChild("VisualMaterial")
    )
    surface = UsdShade.Shader.Define(
        stage, visual_material.GetPath().AppendChild("PreviewSurface")
    )
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.72)
    surface.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    texture = UsdShade.Shader.Define(
        stage, visual_material.GetPath().AppendChild("DiffuseTexture")
    )
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(str(CURTAIN_TEXTURE))
    )
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(
        "sRGB"
    )
    texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    primvar_reader = UsdShade.Shader.Define(
        stage, visual_material.GetPath().AppendChild("TextureCoordinates")
    )
    primvar_reader.CreateIdAttr("UsdPrimvarReader_float2")
    primvar_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    primvar_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        primvar_reader.ConnectableAPI(), "result"
    )
    surface.CreateInput(
        "diffuseColor", Sdf.ValueTypeNames.Color3f
    ).ConnectToSource(texture.ConnectableAPI(), "rgb")
    surface.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    visual_material.CreateSurfaceOutput().ConnectToSource(
        surface.ConnectableAPI(), "surface"
    )
    UsdShade.MaterialBindingAPI.Apply(cloth.GetPrim()).Bind(visual_material)

    particle_system_path = root.GetPath().AppendChild("ParticleSystem")
    particle_spacing = min(
        CURTAIN_WIDTH / (CURTAIN_COLUMNS - 1),
        (CURTAIN_TOP_Z - CURTAIN_BOTTOM_Z) / (CURTAIN_ROWS - 1),
    )
    rest_offset = particle_spacing * 0.45
    particleUtils.add_physx_particle_system(
        stage=stage,
        particle_system_path=particle_system_path,
        simulation_owner=Sdf.Path(physics_scene_path),
        contact_offset=rest_offset * 1.5,
        rest_offset=rest_offset,
        particle_contact_offset=rest_offset * 1.5,
        solid_rest_offset=rest_offset,
        fluid_rest_offset=0.0,
        solver_position_iterations=16,
        enable_ccd=True,
        non_particle_collision_enabled=True,
    )
    particle_material_path = root.GetPath().AppendChild("ParticleMaterial")
    particleUtils.add_pbd_particle_material(
        stage,
        particle_material_path,
        friction=0.4,
        damping=0.1,
        drag=0.05,
    )
    physicsUtils.add_physics_material_to_prim(
        stage, stage.GetPrimAtPath(particle_system_path), particle_material_path
    )
    particleUtils.add_physx_particle_cloth(
        stage=stage,
        path=cloth_path,
        dynamic_mesh_path=None,
        particle_system_path=particle_system_path,
        spring_stretch_stiffness=500.0,
        spring_bend_stiffness=1.0,
        spring_shear_stiffness=100.0,
        spring_damping=0.2,
        self_collision=True,
        self_collision_filter=True,
    )
    UsdPhysics.MassAPI.Apply(cloth.GetPrim()).CreateMassAttr().Set(0.18)

    top_points = Vt.Vec3fArray(points[:CURTAIN_COLUMNS])
    attachment = PhysxSchema.PhysxPhysicsAttachment.Define(
        stage, root.GetPath().AppendChild("TopAttachment")
    )
    attachment.GetActor0Rel().SetTargets([cloth_path])
    attachment.CreatePoints0Attr().Set(top_points)
    attachment.CreatePoints1Attr().Set(top_points)

    rod = UsdGeom.Cylinder.Define(stage, root.GetPath().AppendChild("Rod"))
    rod.CreateAxisAttr().Set(UsdGeom.Tokens.y)
    rod.CreateRadiusAttr().Set(0.018)
    rod.CreateHeightAttr().Set(CURTAIN_WIDTH + 0.10)
    rod.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.15, 0.11, 0.08)]))
    UsdGeom.Xformable(rod).AddTranslateOp().Set(
        Gf.Vec3d(CURTAIN_X, CURTAIN_CENTER_Y, CURTAIN_TOP_Z + 0.04)
    )
    UsdPhysics.CollisionAPI.Apply(rod.GetPrim())
    return len(points)


def create_room_collision_proxies(
    stage: Usd.Stage,
) -> tuple[list[tuple[Gf.Vec3d, Gf.Vec3d]], int]:
    """Build map bounds for all obstacles and physical proxies for structure."""
    room = stage.GetPrimAtPath("/World/Room")
    for prim in Usd.PrimRange(room):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False).Set(False)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False).Set(False)

    sources = []
    wall_group = stage.GetPrimAtPath("/World/Room/Meshes/wall")
    sources.extend(wall_group.GetChildren())
    other_group = stage.GetPrimAtPath("/World/Room/Meshes/other")
    sources.extend(
        prim
        for prim in other_group.GetChildren()
        if is_blocking_other_prim(prim.GetName())
    )
    meshes = stage.GetPrimAtPath("/World/Room/Meshes")
    ignored_groups = {"ceiling", "floor", "other", "wall"}
    for group in meshes.GetChildren():
        if group.GetName() not in ignored_groups:
            sources.extend(group.GetChildren())

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)
    obstacle_bounds = []
    collider_count = 0
    for source in sources:
        bounds = bbox_cache.ComputeWorldBound(source).ComputeAlignedBox()
        minimum, maximum = bounds.GetMin(), bounds.GetMax()
        dimensions = maximum - minimum
        if any(value <= 1e-4 or value > 100.0 for value in dimensions):
            continue
        structural = source.GetParent() in (wall_group, other_group)
        acceptance_source = (
            ARGS.traversal_acceptance
            and str(source.GetPath()) == ACCEPTANCE_PRIMS[ARGS.acceptance_obstacle]
        )
        if not structural and not intersects_navigation_height(minimum[2], maximum[2]):
            continue
        proxy_bounds = [(minimum, maximum)]
        if source.GetName() == "wall_0011":
            # The source mesh combines multiple wall sections, so its aligned
            # AABB fills both real portals. Keep wall sides and physical
            # lintels, while the 2D map later ignores lintels above the robot.
            openings = (
                (CURTAIN_CENTER_Y - 0.42, CURTAIN_CENTER_Y + 0.42),
                (
                    OPEN_KITCHEN_DOOR_CENTER_Y - OPEN_KITCHEN_DOOR_HALF_WIDTH,
                    OPEN_KITCHEN_DOOR_CENTER_Y + OPEN_KITCHEN_DOOR_HALF_WIDTH,
                ),
            )
            proxy_bounds = [
                (
                    Gf.Vec3d(minimum[0], solid_min_y, minimum[2]),
                    Gf.Vec3d(maximum[0], solid_max_y, maximum[2]),
                )
                for solid_min_y, solid_max_y in solid_spans_around_openings(
                    minimum[1], maximum[1], openings
                )
            ]
            for opening_min_y, opening_max_y, opening_top_z in (
                (*openings[0], CURTAIN_TOP_Z + 0.08),
                (*openings[1], OPEN_KITCHEN_DOOR_TOP_Z),
            ):
                if opening_top_z < maximum[2]:
                    proxy_bounds.append(
                        (
                            Gf.Vec3d(minimum[0], opening_min_y, opening_top_z),
                            Gf.Vec3d(maximum[0], opening_max_y, maximum[2]),
                        )
                    )
        for proxy_minimum, proxy_maximum in proxy_bounds:
            if (
                not acceptance_source
                and intersects_navigation_height(proxy_minimum[2], proxy_maximum[2])
            ):
                obstacle_bounds.append((proxy_minimum, proxy_maximum))
            if not structural:
                continue
            proxy_dimensions = proxy_maximum - proxy_minimum
            cube = UsdGeom.Cube.Define(
                stage, f"/World/CollisionProxies/proxy_{collider_count:03d}"
            )
            cube.CreateSizeAttr(1.0)
            xform = UsdGeom.Xformable(cube)
            xform.AddTranslateOp().Set((proxy_minimum + proxy_maximum) / 2.0)
            xform.AddScaleOp().Set(proxy_dimensions)
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
            physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
            physx_collision.CreateRestOffsetAttr().Set(0.0)
            physx_collision.CreateContactOffsetAttr().Set(0.001)
            UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
            collider_count += 1
    return obstacle_bounds, collider_count


def configure_acceptance_furniture(stage: Usd.Stage):
    """Apply physics only to the explicitly selected acceptance-test instance."""
    if not ARGS.traversal_acceptance:
        return None
    if ARGS.acceptance_obstacle == "curtain":
        print(
            "Acceptance curtain uses the real particle-cloth doorway fixture; "
            "no rigid staging proxy is created",
            flush=True,
        )
        return None
    if ARGS.acceptance_obstacle in {"movable_box", "fixed_box"}:
        path = ACCEPTANCE_PRIMS[ARGS.acceptance_obstacle]
        half_x, half_y, half_z = 0.22, 0.18, 0.40
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr().Set(Vt.Vec3fArray([
            Gf.Vec3f(-half_x, -half_y, -half_z),
            Gf.Vec3f(half_x, -half_y, -half_z),
            Gf.Vec3f(half_x, half_y, -half_z),
            Gf.Vec3f(-half_x, half_y, -half_z),
            Gf.Vec3f(-half_x, -half_y, half_z),
            Gf.Vec3f(half_x, -half_y, half_z),
            Gf.Vec3f(half_x, half_y, half_z),
            Gf.Vec3f(-half_x, half_y, half_z),
        ]))
        mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray([4] * 6))
        mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray([
            0, 3, 2, 1,
            4, 5, 6, 7,
            0, 4, 7, 3,
            1, 2, 6, 5,
            0, 1, 5, 4,
            3, 7, 6, 2,
        ]))
        mesh.CreateExtentAttr().Set(Vt.Vec3fArray([
            Gf.Vec3f(-half_x, -half_y, -half_z),
            Gf.Vec3f(half_x, half_y, half_z),
        ]))
        mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr().Set(True)
        mesh.CreateDisplayColorAttr().Set([Gf.Vec3f(0.64, 0.42, 0.20)])
        mesh.CreatePurposeAttr().Set(UsdGeom.Tokens.render)
        mesh.CreateVisibilityAttr().Set(UsdGeom.Tokens.inherited)
        xform = UsdGeom.Xformable(mesh)
        box_x = (
            ARGS.start_x
            + math.cos(ARGS.start_yaw) * ACCEPTANCE_BOX_ALONG_M
            - math.sin(ARGS.start_yaw) * ACCEPTANCE_BOX_LATERAL_M
        )
        box_y = (
            ARGS.start_y
            + math.sin(ARGS.start_yaw) * ACCEPTANCE_BOX_ALONG_M
            + math.cos(ARGS.start_yaw) * ACCEPTANCE_BOX_LATERAL_M
        )
        xform.AddTranslateOp().Set(Gf.Vec3d(box_x, box_y, half_z))
        xform.AddOrientOp().Set(Gf.Quatf(
            math.cos(ARGS.start_yaw / 2.0), 0.0, 0.0,
            math.sin(ARGS.start_yaw / 2.0),
        ))
        collision = UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        collision.CreateCollisionEnabledAttr().Set(True)
        UsdPhysics.MeshCollisionAPI.Apply(
            mesh.GetPrim()
        ).CreateApproximationAttr().Set("convexHull")
        rigid = UsdPhysics.RigidBodyAPI.Apply(mesh.GetPrim())
        rigid.CreateRigidBodyEnabledAttr().Set(True)
        fixed = ARGS.acceptance_obstacle == "fixed_box"
        rigid.CreateKinematicEnabledAttr().Set(fixed)
        UsdPhysics.MassAPI.Apply(mesh.GetPrim()).CreateMassAttr().Set(2.0)
        material = UsdShade.Material.Define(
            stage, "/World/PhysicsMaterials/AcceptanceBox"
        )
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr().Set(0.45)
        physics_material.CreateDynamicFrictionAttr().Set(0.35)
        physics_material.CreateRestitutionAttr().Set(0.0)
        UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(
            material, materialPurpose="physics"
        )
        print(
            "Acceptance box configured: "
            f"mode={ARGS.acceptance_obstacle}, dimensions=0.44x0.36x0.80m, "
            f"kinematic={fixed}, map_excluded=true, "
            f"initial_path_offset={ACCEPTANCE_BOX_ALONG_M:.2f}m, "
            f"path_lateral_offset={ACCEPTANCE_BOX_LATERAL_M:.2f}m",
            flush=True,
        )
        return mesh.GetPrim()
    raise RuntimeError(f"Unsupported acceptance obstacle: {ARGS.acceptance_obstacle}")


def create_map_publisher(obstacle_bounds: list[tuple[Gf.Vec3d, Gf.Vec3d]]):
    """Create a static occupancy grid aligned with the simulation start pose."""
    resolution = 0.05
    margin = 0.5
    min_x = math.floor((min(bounds[0][0] for bounds in obstacle_bounds) - margin) / resolution) * resolution
    min_y = math.floor((min(bounds[0][1] for bounds in obstacle_bounds) - margin) / resolution) * resolution
    max_x = math.ceil((max(bounds[1][0] for bounds in obstacle_bounds) + margin) / resolution) * resolution
    max_y = math.ceil((max(bounds[1][1] for bounds in obstacle_bounds) + margin) / resolution) * resolution
    width = round((max_x - min_x) / resolution)
    height = round((max_y - min_y) / resolution)
    occupancy = np.zeros((height, width), dtype=np.int8)
    for minimum, maximum in obstacle_bounds:
        x0 = max(0, math.floor((minimum[0] - min_x) / resolution))
        y0 = max(0, math.floor((minimum[1] - min_y) / resolution))
        x1 = min(width, math.ceil((maximum[0] - min_x) / resolution))
        y1 = min(height, math.ceil((maximum[1] - min_y) / resolution))
        occupancy[y0:y1, x0:x1] = 100

    rclpy.init(args=[])
    node = rclpy.create_node("isaac_room_map")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = node.create_publisher(OccupancyGrid, "/map", qos)
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.resolution = resolution
    message.info.width = width
    message.info.height = height
    dx = min_x - ARGS.start_x
    dy = min_y - ARGS.start_y
    cosine = math.cos(ARGS.start_yaw)
    sine = math.sin(ARGS.start_yaw)
    message.info.origin.position.x = cosine * dx + sine * dy
    message.info.origin.position.y = -sine * dx + cosine * dy
    message.info.origin.orientation.z = math.sin(-ARGS.start_yaw / 2.0)
    message.info.origin.orientation.w = math.cos(-ARGS.start_yaw / 2.0)
    message.data = occupancy.ravel().tolist()
    occupied = int(np.count_nonzero(occupancy))
    print(f"Occupancy map: {width}x{height}, occupied={occupied}, free={width * height - occupied}", flush=True)
    return node, publisher, message


def apply_traction_material(stage: Usd.Stage, robot_path: str, ground) -> int:
    """Disable wheel traction because the chassis follows ROS velocity directly."""
    ground_material = PhysicsMaterial(
        prim_path="/World/PhysicsMaterials/Ground",
        static_friction=WHEEL_STATIC_FRICTION,
        dynamic_friction=WHEEL_DYNAMIC_FRICTION,
        restitution=0.0,
    )
    wheel_material = PhysicsMaterial(
        prim_path="/World/PhysicsMaterials/Wheels",
        static_friction=WHEEL_STATIC_FRICTION,
        dynamic_friction=WHEEL_DYNAMIC_FRICTION,
        restitution=0.0,
    )
    PhysxSchema.PhysxMaterialAPI.Apply(
        wheel_material.prim
    ).CreateFrictionCombineModeAttr().Set("min")
    ground.apply_physics_material(ground_material)
    wheel_colliders = 0
    robot_root = stage.GetPrimAtPath(robot_path).GetParent()
    for prim in Usd.PrimRange(robot_root):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if "wheel" not in prim.GetPath().pathString.lower():
            continue
        binding = UsdShade.MaterialBindingAPI.Apply(prim)
        binding.Bind(wheel_material.material, UsdShade.Tokens.strongerThanDescendants, "physics")
        wheel_colliders += 1
    return wheel_colliders


def import_robot() -> str:
    _, config = omni.kit.commands.execute("URDFCreateImportConfig")
    config.set_merge_fixed_joints(True)
    config.set_convex_decomp(False)
    config.set_import_inertia_tensor(True)
    config.set_fix_base(False)
    config.set_self_collision(False)
    config.set_distance_scale(1.0)
    config.set_default_drive_type(2)
    config.set_default_drive_strength(1000.0)
    config.set_default_position_drive_damping(100.0)
    config.set_make_default_prim(False)
    config.set_create_physics_scene(False)
    config.set_collision_from_visuals(False)
    status, root_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(ROBOT_URDF),
        import_config=config,
        get_articulation_root=True,
    )
    if not status or not root_path:
        raise RuntimeError(f"URDF import failed: {ROBOT_URDF}")
    return root_path


def import_arm() -> None:
    _, config = omni.kit.commands.execute("MJCFCreateImportConfig")
    config.set_fix_base(True)
    config.set_import_inertia_tensor(True)
    config.set_self_collision(False)
    config.set_make_default_prim(False)
    config.set_create_physics_scene(False)
    config.set_visualize_collision_geoms(False)
    status, _ = omni.kit.commands.execute(
        "MJCFCreateAsset",
        mjcf_path=str(ARM_MJCF),
        import_config=config,
        prim_path=ARM_PATH,
    )
    if not status:
        raise RuntimeError(f"MJCF import failed: {ARM_MJCF}")


def restore_arm_visuals() -> None:
    """Restore arm bodies hidden by the Isaac MJCF collision import."""
    wait_for_stage()
    for link_name in ARM_VISUAL_PATCHES:
        visual = ARM_MJCF.parent / f"assets/{link_name}.tmp.usd"
        require_file(visual)
        add_reference_to_stage(
            str(visual),
            f"{ARM_PATH}/base_link/{link_name}/visual_patch",
        )
    wait_for_stage()


def create_d435(stage: Usd.Stage) -> str:
    """Create a centered D435-sized RGB sensor on the Piper gripper."""
    mount_path = f"{ARM_PATH}/base_link/link6/d435"
    mount = UsdGeom.Xform.Define(stage, mount_path)
    mount_xform = UsdGeom.Xformable(mount)
    mount_xform.AddTranslateOp().Set(Gf.Vec3d(-0.065, 0.0, 0.024))
    mount_xform.AddRotateYOp().Set(-80.0)

    body = UsdGeom.Cube.Define(stage, f"{mount_path}/body")
    body.CreateSizeAttr(1.0)
    body.CreateDisplayColorAttr([(0.18, 0.18, 0.18)])
    body_xform = UsdGeom.Xformable(body)
    body_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    body_xform.AddScaleOp().Set(Gf.Vec3d(0.02505, 0.09, 0.025))

    for name, y in (("left_lens", -0.03), ("right_lens", 0.03)):
        lens = UsdGeom.Cylinder.Define(stage, f"{mount_path}/{name}")
        lens.CreateAxisAttr("X")
        lens.CreateRadiusAttr(0.005)
        lens.CreateHeightAttr(0.002)
        lens.CreateDisplayColorAttr([(0.01, 0.01, 0.01)])
        UsdGeom.Xformable(lens).AddTranslateOp().Set(Gf.Vec3d(0.0135, y, 0.0))

    camera_path = f"{mount_path}/Camera"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera.CreateFocalLengthAttr(18.0)
    camera.CreateHorizontalApertureAttr(24.9)
    camera.CreateVerticalApertureAttr(14.0)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.03, 10.0))
    UsdGeom.Xformable(camera).AddOrientOp().Set(
        Gf.Quatf(0.5, 0.5, -0.5, -0.5)
    )
    camera.GetPrim().CreateAttribute(
        ISAAC_NAME_OVERRIDE, Sdf.ValueTypeNames.String, True
    ).Set("d435_color_optical_frame")
    return camera_path


def mount_arm(robot_path: str) -> None:
    """Mount Piper and Scout as one articulation safe for base teleporting."""
    from isaacsim.robot_setup.assembler import RobotAssembler

    RobotAssembler().assemble_articulations(
        base_robot_path=robot_path,
        attach_robot_path=ARM_PATH,
        base_robot_mount_frame="",
        attach_robot_mount_frame="/base_link/base_link",
        fixed_joint_offset=np.array(
            [ARGS.arm_mount_x, ARGS.arm_mount_y, ARGS.arm_mount_z]
        ),
        fixed_joint_orient=np.array([1.0, 0.0, 0.0, 0.0]),
        mask_all_collisions=True,
        single_robot=True,
    )
    stage = omni.usd.get_context().get_stage()
    arm = stage.GetPrimAtPath(ARM_PATH)
    removed = []
    for prim in Usd.PrimRange(arm):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            removed.append(str(prim.GetPath()))
        physx_articulation = getattr(PhysxSchema, "PhysxArticulationAPI", None)
        if physx_articulation is not None and prim.HasAPI(physx_articulation):
            prim.RemoveAPI(physx_articulation)
    if removed:
        print(
            "Removed nested Piper articulation roots after assembly: "
            + ", ".join(removed),
            flush=True,
        )


def create_arm_state_graph(robot_path: str) -> None:
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/PiperROS2Graph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("Publish", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            keys.SET_VALUES: [
                ("Publish.inputs:topicName", "/isaac_joint_states"),
                ("Publish.inputs:targetPrim", [usdrt.Sdf.Path(robot_path)]),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "Publish.inputs:execIn"),
                ("Context.outputs:context", "Publish.inputs:context"),
                ("SimTime.outputs:simulationTime", "Publish.inputs:timeStamp"),
            ],
        },
    )


def create_camera_graph(camera_path: str, base_link_path: str) -> None:
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/D435ROS2Graph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Render", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.SET_VALUES: [
                ("Render.inputs:cameraPrim", [usdrt.Sdf.Path(camera_path)]),
                ("Render.inputs:width", 640),
                ("Render.inputs:height", 480),
                ("PublishRGB.inputs:topicName", "/isaac/color_image_raw"),
                ("PublishRGB.inputs:type", "rgb"),
                ("PublishRGB.inputs:frameId", "d435_color_optical_frame"),
                ("PublishDepth.inputs:topicName", "/isaac/aligned_depth_to_color/image_raw"),
                ("PublishDepth.inputs:type", "depth"),
                ("PublishDepth.inputs:frameId", "d435_color_optical_frame"),
                ("PublishInfo.inputs:topicName", "/isaac/color/camera_info"),
                ("PublishInfo.inputs:frameId", "d435_color_optical_frame"),
                ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path(camera_path)]),
                ("PublishTF.inputs:parentPrim", [usdrt.Sdf.Path(base_link_path)]),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "Render.inputs:execIn"),
                ("Render.outputs:execOut", "PublishRGB.inputs:execIn"),
                ("Render.outputs:execOut", "PublishDepth.inputs:execIn"),
                ("Render.outputs:execOut", "PublishInfo.inputs:execIn"),
                ("Render.outputs:execOut", "PublishTF.inputs:execIn"),
                ("Render.outputs:renderProductPath", "PublishRGB.inputs:renderProductPath"),
                ("Render.outputs:renderProductPath", "PublishDepth.inputs:renderProductPath"),
                ("Render.outputs:renderProductPath", "PublishInfo.inputs:renderProductPath"),
                ("Context.outputs:context", "PublishRGB.inputs:context"),
                ("Context.outputs:context", "PublishDepth.inputs:context"),
                ("Context.outputs:context", "PublishInfo.inputs:context"),
                ("SimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
            ],
        },
    )


def create_odometry_graph(robot_path: str) -> None:
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/ROS2Graph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("Odometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.SET_VALUES: [
                ("Odometry.inputs:chassisPrim", [usdrt.Sdf.Path(robot_path)]),
                ("PublishOdom.inputs:topicName", "/odom"),
                ("PublishOdom.inputs:odomFrameId", "isaac_world"),
                ("PublishOdom.inputs:chassisFrameId", "base_link"),
                ("PublishTF.inputs:parentFrameId", "isaac_world"),
                ("PublishTF.inputs:childFrameId", "base_link"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "Clock.inputs:execIn"),
                ("Tick.outputs:tick", "Odometry.inputs:execIn"),
                ("Tick.outputs:tick", "PublishOdom.inputs:execIn"),
                ("Tick.outputs:tick", "PublishTF.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ("Odometry.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                ("Odometry.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("Odometry.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("Odometry.outputs:position", "PublishOdom.inputs:position"),
                ("Odometry.outputs:orientation", "PublishTF.inputs:rotation"),
                ("Odometry.outputs:position", "PublishTF.inputs:translation"),
            ],
        },
    )


def create_velocity_subscription(node):
    command = {"linear": 0.0, "angular": 0.0, "received_at": float("-inf")}

    def on_command(message: Twist) -> None:
        command["linear"] = float(message.linear.x)
        command["angular"] = float(message.angular.z)
        command["received_at"] = time.monotonic()

    node.create_subscription(Twist, "/cmd_vel", on_command, 1)
    return command


def create_arm_subscription(node, robot: SingleArticulation) -> None:
    def on_command(message: JointState) -> None:
        if not message.name or len(message.position) != len(message.name):
            node.get_logger().warning(
                "Piper command requires equally sized name and position arrays"
            )
            return
        invalid_names = [name for name in message.name if name not in ARM_JOINT_NAMES]
        if invalid_names:
            node.get_logger().warning(
                f"Ignoring non-Piper joints: {invalid_names}"
            )
            return
        robot.apply_action(
            ArticulationAction(
                joint_positions=np.array(message.position),
                joint_indices=np.array(
                    [robot.get_dof_index(name) for name in message.name]
                ),
            )
        )
    # The probe server publishes incremental targets. Keeping only the newest
    # target prevents an old command backlog from driving the arm after a
    # phase change or during recovery.
    node.create_subscription(JointState, "/isaac_joint_command", on_command, 1)


def configure_arm_drives(stage: Usd.Stage) -> None:
    configured = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ARM_PATH)):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(
                prim
            ).CreateDisableGravityAttr().Set(True)
        name = prim.GetName()
        if name not in ARM_JOINT_NAMES:
            continue
        index = ARM_JOINT_NAMES.index(name)
        linear = prim.IsA(UsdPhysics.PrismaticJoint)
        drive = UsdPhysics.DriveAPI.Apply(
            prim, "linear" if linear else "angular"
        )
        drive.CreateTypeAttr().Set("force")
        drive.CreateMaxForceAttr().Set(float(ARM_MAX_EFFORT[index]))
        unit_scale = 1.0 if linear else math.pi / 180.0
        drive.CreateStiffnessAttr().Set(float(ARM_KP[index] * unit_scale))
        drive.CreateDampingAttr().Set(
            float(ARM_KD[index] * ARM_DAMPING_SCALE)
        )
        target = ARM_START[index] if linear else np.degrees(ARM_START[index])
        drive.CreateTargetPositionAttr().Set(float(target))
        drive.CreateTargetVelocityAttr().Set(0.0)
        configured += 1
    if configured != len(ARM_JOINT_NAMES):
        raise RuntimeError(f"Expected 8 Piper joints, configured {configured}")


def apply_velocity_command(
    robot: SingleArticulation,
    wheel_indices: np.ndarray,
    linear: float,
    angular: float,
) -> None:
    half_track = WHEEL_DISTANCE / 2.0
    turn_speed = angular * half_track
    left = (linear - turn_speed) / WHEEL_RADIUS
    right = (linear + turn_speed) / WHEEL_RADIUS
    robot.apply_action(
        ArticulationAction(
            joint_velocities=np.array([left, right, left, right]),
            joint_indices=wheel_indices,
        )
    )
    _, orientation = robot.get_world_pose()
    w, x, y, z = orientation
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    robot.set_world_velocity(
        np.array([
            linear * math.cos(yaw),
            linear * math.sin(yaw),
            0.0,
            0.0,
            0.0,
            angular,
        ])
    )


def create_lidar(base_link_path: str):
    settings = carb.settings.get_settings()
    profile_setting = "/app/sensors/nv/lidar/profileBaseFolder"
    profile_dirs = list(settings.get(profile_setting) or [])
    profile_dirs.insert(0, str(LIDAR_CONFIG_DIR) + os.sep)
    settings.set_string_array(profile_setting, profile_dirs)

    _, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=f"{base_link_path}/lidar_link",
        parent=None,
        config="ScoutMiniLidar",
        translation=Gf.Vec3d(0.21, 0.0, 0.17),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )
    render_product = rep.create.render_product(sensor.GetPath(), [1, 1], name="ScoutMiniLidar")
    scan_writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
    scan_writer.initialize(topicName=ARGS.scan_topic, frameId="lidar_link")
    scan_writer.attach([render_product])

    debug_writer = None
    if ARGS.debug_lidar:
        debug_writer = rep.writers.get("RtxLidarDebugDrawPointCloud")
        debug_writer.attach([render_product])
    return render_product, scan_writer, debug_writer


def main() -> None:
    if not ARGS.scan_topic.startswith("/"):
        raise ValueError("--scan-topic must be an absolute ROS topic")
    room_usd = FULL_ROOM_USD if ARGS.full_room else NAVIGATION_ROOM_USD
    require_file(room_usd)
    require_file(ROBOT_URDF)
    require_file(ARM_MJCF)
    require_file(LIDAR_CONFIG_DIR / "ScoutMiniLidar.json")
    require_file(CURTAIN_TEXTURE)

    enable_extension("isaacsim.asset.importer.urdf")
    enable_extension("isaacsim.asset.importer.mjcf")
    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.sensors.rtx")
    enable_extension("isaacsim.robot_setup.assembler")
    APP.update()

    world = World(physics_dt=PHYSICS_DT, rendering_dt=RENDERING_DT, stage_units_in_meters=1.0)
    physics_context = world.get_physics_context()
    physics_context.enable_gpu_dynamics(True)
    physics_context.set_broadphase_type("GPU")
    print(f"Loading room: {room_usd.name}", flush=True)
    add_reference_to_stage(str(room_usd), "/World/Room")
    wait_for_stage()
    stage = omni.usd.get_context().get_stage()
    sanitize_display_primvars(stage)
    # Replace the original door leaf with the dynamic cloth curtain.
    stage.GetPrimAtPath("/World/Room/Meshes/other/door_0000").SetActive(False)
    indoor_light_count = create_indoor_lights(stage)
    obstacle_bounds, room_collider_count = create_room_collision_proxies(stage)
    acceptance_furniture = configure_acceptance_furniture(stage)
    for index, (minimum, maximum) in enumerate(obstacle_bounds):
        if (
            minimum[0] < ARGS.start_x + 0.31
            and maximum[0] > ARGS.start_x - 0.31
            and minimum[1] < ARGS.start_y + 0.293
            and maximum[1] > ARGS.start_y - 0.293
            and minimum[2] < 0.289
            and maximum[2] > 0.053
        ):
            print(f"Warning: start pose overlaps room collider {index}: {minimum} .. {maximum}", flush=True)

    ground = world.scene.add_default_ground_plane(z_position=-0.01)
    UsdGeom.Imageable(ground.prim).MakeInvisible()
    box_acceptance = (
        ARGS.traversal_acceptance
        and ARGS.acceptance_obstacle in {"movable_box", "fixed_box"}
    )
    curtain_particle_count = (
        0 if box_acceptance else create_door_curtain(stage, physics_context.prim_path)
    )

    robot_path = import_robot()
    import_arm()
    restore_arm_visuals()
    camera_path = create_d435(stage)
    mount_arm(robot_path)
    configure_arm_drives(stage)
    create_arm_state_graph(robot_path)
    create_camera_graph(camera_path, robot_path)
    wheel_collider_count = apply_traction_material(stage, robot_path, ground)
    world.reset()
    robot = SingleArticulation(robot_path)
    robot.initialize()
    acceptance_rigid = None
    acceptance_visual = None
    acceptance_goal = {"point": None}
    acceptance_path = {"points": None}
    acceptance_goal_subscription = None
    acceptance_path_subscription = None
    if acceptance_furniture is not None:
        acceptance_rigid = SingleRigidPrim(
            str(acceptance_furniture.GetPath()), name="acceptance_furniture"
        )
        acceptance_rigid.initialize()
        # RigidPrim writes a live PhysX transform once simulation has started,
        # while RTX sensors render the authored USD/Fabric transform. Keep a
        # separate XForm view so acceptance staging updates both representations.
        acceptance_visual = SingleXFormPrim(
            str(acceptance_furniture.GetPath()), name="acceptance_furniture_visual"
        )
    print(f"Articulation DOFs: {robot.dof_names}", flush=True)
    wheel_indices = np.array(
        [robot.get_dof_index(name) for name in WHEEL_JOINT_NAMES]
    )
    arm_indices = np.array(
        [robot.get_dof_index(name) for name in ARM_JOINT_NAMES]
    )
    print(f"Piper DOFs: {list(ARM_JOINT_NAMES)}", flush=True)
    robot.set_world_pose(
        position=np.array([ARGS.start_x, ARGS.start_y, 0.171]),
        orientation=np.array([np.cos(ARGS.start_yaw / 2), 0.0, 0.0, np.sin(ARGS.start_yaw / 2)]),
    )
    robot.set_world_velocity(np.zeros(6))
    robot.set_joint_positions(ARM_START, joint_indices=arm_indices)
    robot.set_joint_velocities(np.zeros(len(robot.dof_names)))
    robot.apply_action(
        ArticulationAction(joint_positions=ARM_START, joint_indices=arm_indices)
    )
    create_odometry_graph(robot_path)
    render_product, scan_writer, debug_writer = create_lidar(robot_path)
    map_node, map_publisher, map_message = create_map_publisher(obstacle_bounds)
    if acceptance_rigid is not None:
        def on_acceptance_goal(message: PointStamped) -> None:
            if message.header.frame_id.lstrip("/") not in {"map", "odom", ""}:
                return
            point = float(message.point.x), float(message.point.y)
            if all(math.isfinite(value) for value in point):
                acceptance_goal["point"] = point

        acceptance_goal_subscription = map_node.create_subscription(
            PointStamped, "/clicked_point", on_acceptance_goal, 10
        )

        def on_acceptance_path(message: NavigationPath) -> None:
            if message.header.frame_id.lstrip("/") not in {"map", "odom", ""}:
                return
            points = [
                (float(pose.pose.position.x), float(pose.pose.position.y))
                for pose in message.poses
            ]
            if len(points) >= 2 and all(
                math.isfinite(value) for point in points for value in point
            ):
                acceptance_path["points"] = points

        acceptance_path_subscription = map_node.create_subscription(
            NavigationPath, "/neupan_initial_path", on_acceptance_path, 10
        )
    furniture_pose_publisher = map_node.create_publisher(
        PoseStamped, "/isaac/acceptance_furniture_pose", 10
    )
    velocity_command = create_velocity_subscription(map_node)
    create_arm_subscription(map_node, robot)

    if not ARGS.headless:
        set_camera_view(
            eye=np.array([ARGS.start_x, ARGS.start_y - 4.0, 1.8]),
            target=np.array([ARGS.start_x + 0.1, ARGS.start_y, 0.55]),
            camera_prim_path="/OmniverseKit_Persp",
        )

    print(
        f"Isaac Sim ready: robot={robot_path}, room_colliders={room_collider_count}, "
        f"map_obstacles={len(obstacle_bounds)}, "
        f"wheel_colliders={wheel_collider_count}, indoor_lights={indoor_light_count}, "
        f"curtain_particles={curtain_particle_count}, "
        f"wheel_friction={WHEEL_STATIC_FRICTION}/{WHEEL_DYNAMIC_FRICTION}, "
        f"arm_mount=({ARGS.arm_mount_x}, {ARGS.arm_mount_y}, {ARGS.arm_mount_z}), "
        "base_control=direct_body_velocity, camera=640x480, render_target=30Hz, realtime=60Hz",
        flush=True,
    )
    print(
        f"ROS 2 topics: /clock /map {ARGS.scan_topic} /odom /tf /cmd_vel "
        "/isaac/color_image_raw /isaac/aligned_depth_to_color/image_raw "
        "/isaac/color/camera_info /isaac_joint_command /isaac_joint_states",
        flush=True,
    )
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    now = map_node.get_clock().now().to_msg()
    map_message.header.stamp = now
    map_message.info.map_load_time = now
    map_publisher.publish(map_message)
    started_at = timeline.get_current_time()
    next_step_at = time.perf_counter()
    frame = 0
    curtain_test_commanded = False
    # Rigid acceptance fixtures are authored at their route position before
    # RTX products are created. Moving a kinematic prim after the timeline
    # starts updates PhysX immediately but can leave RTX LiDAR on stale Fabric
    # geometry, producing a physical collision with no scan return.
    acceptance_staged = acceptance_rigid is not None
    while APP.is_running():
        rclpy.spin_once(map_node, timeout_sec=0.0)
        if (
            acceptance_rigid is not None
            and not acceptance_staged
            and acceptance_goal["point"] is not None
            and acceptance_path["points"] is not None
        ):
            robot_position, _ = robot.get_world_pose()
            delta_x = float(robot_position[0]) - ARGS.start_x
            delta_y = float(robot_position[1]) - ARGS.start_y
            cosine = math.cos(ARGS.start_yaw)
            sine = math.sin(ARGS.start_yaw)
            robot_local = np.array([
                cosine * delta_x + sine * delta_y,
                -sine * delta_x + cosine * delta_y,
            ])
            target_local = np.asarray(acceptance_goal["point"], dtype=float)
            path_points = [np.asarray(point, dtype=float) for point in acceptance_path["points"]]
            path_length = sum(
                float(np.linalg.norm(second - first))
                for first, second in zip(path_points, path_points[1:])
            )
            direct_distance = float(np.linalg.norm(target_local - robot_local))
            # The first /clicked_point is Module4's hold-at-current-pose command.
            # Stage only on the first actual route segment.
            minimum_staging_segment = 1.0
            minimum_endpoint_distance = 0.0
            if (
                path_length >= minimum_staging_segment
                and direct_distance >= minimum_endpoint_distance
            ):
                target_distance = min(1.80, max(1.20, path_length - 1.0))
                traversed = 0.0
                center_local = path_points[-1]
                staging_direction = np.array([1.0, 0.0])
                for first, second in zip(path_points, path_points[1:]):
                    segment = second - first
                    segment_length = float(np.linalg.norm(segment))
                    if segment_length <= 1e-6:
                        continue
                    if traversed + segment_length >= target_distance:
                        fraction = (target_distance - traversed) / segment_length
                        center_local = first + fraction * segment
                        staging_direction = segment / segment_length
                        break
                    traversed += segment_length
                world_x = ARGS.start_x + cosine * center_local[0] - sine * center_local[1]
                world_y = ARGS.start_y + sine * center_local[0] + cosine * center_local[1]
                world_x -= float(staging_direction[1]) * ACCEPTANCE_BOX_LATERAL_M
                world_y += float(staging_direction[0]) * ACCEPTANCE_BOX_LATERAL_M
                furniture_position, furniture_orientation = acceptance_rigid.get_world_pose()
                staged_orientation = furniture_orientation
                path_yaw_world = ARGS.start_yaw + math.atan2(
                    float(staging_direction[1]),
                    float(staging_direction[0]),
                )
                staged_orientation = np.array([
                    math.cos(path_yaw_world / 2.0),
                    0.0,
                    0.0,
                    math.sin(path_yaw_world / 2.0),
                ])
                staged_position = np.array([
                    world_x, world_y, float(furniture_position[2])
                ])
                acceptance_visual.set_world_pose(
                    staged_position, staged_orientation
                )
                acceptance_rigid.set_world_pose(staged_position, staged_orientation)
                if ARGS.acceptance_obstacle == "movable_box":
                    acceptance_rigid.set_linear_velocity(np.zeros(3))
                    acceptance_rigid.set_angular_velocity(np.zeros(3))
                physics_position, _ = acceptance_rigid.get_world_pose()
                visual_position = np.asarray(
                    UsdGeom.XformCache().GetLocalToWorldTransform(
                        acceptance_furniture
                    ).ExtractTranslation(),
                    dtype=float,
                )
                staging_error = float(np.linalg.norm(
                    np.asarray(physics_position, dtype=float) - visual_position
                ))
                if staging_error > 0.02:
                    raise RuntimeError(
                        "Acceptance furniture staging diverged between PhysX "
                        f"and USD/RTX by {staging_error:.3f}m"
                    )
                acceptance_staged = True
                print(
                    "Acceptance furniture staged on real Module4 segment: "
                    f"mode={ARGS.acceptance_obstacle}, "
                    f"map=({center_local[0]:.3f}, {center_local[1]:.3f}), "
                    f"segment_goal=({target_local[0]:.3f}, {target_local[1]:.3f}), "
                    f"endpoint_distance={direct_distance:.3f}m, "
                    f"physics_usd_error={staging_error:.4f}m",
                    flush=True,
                )
        if (
            ARGS.test_curtain
            and not curtain_test_commanded
            and timeline.get_current_time() - started_at >= 1.0
        ):
            curtain_pose = np.array([0.0, 2.59, -2.65, 0.0, 0.30, 0.0, 0.02, -0.02])
            robot.apply_action(
                ArticulationAction(
                    joint_positions=curtain_pose,
                    joint_indices=arm_indices,
                )
            )
            curtain_test_commanded = True
            print("Curtain contact test pose commanded", flush=True)
        command_age = time.monotonic() - velocity_command["received_at"]
        if command_age <= COMMAND_TIMEOUT:
            linear = velocity_command["linear"]
            angular = velocity_command["angular"]
        else:
            linear = 0.0
            angular = 0.0
        apply_velocity_command(robot, wheel_indices, linear, angular)
        world.step(render=True)
        next_step_at += RENDERING_DT
        sleep_time = next_step_at - time.perf_counter()
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_step_at = time.perf_counter()
        if acceptance_rigid is not None and frame % 3 == 0:
            translation, rotation = acceptance_rigid.get_world_pose()
            pose = PoseStamped()
            pose.header.stamp = map_node.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            delta_x = float(translation[0]) - ARGS.start_x
            delta_y = float(translation[1]) - ARGS.start_y
            cosine = math.cos(ARGS.start_yaw)
            sine = math.sin(ARGS.start_yaw)
            pose.pose.position.x = cosine * delta_x + sine * delta_y
            pose.pose.position.y = -sine * delta_x + cosine * delta_y
            pose.pose.position.z = float(translation[2])
            half = -ARGS.start_yaw / 2.0
            local_sine, local_cosine = math.sin(half), math.cos(half)
            world_w, world_x, world_y, world_z = map(float, rotation)
            pose.pose.orientation.x = local_cosine * world_x - local_sine * world_y
            pose.pose.orientation.y = local_cosine * world_y + local_sine * world_x
            pose.pose.orientation.z = local_cosine * world_z + local_sine * world_w
            pose.pose.orientation.w = local_cosine * world_w - local_sine * world_z
            furniture_pose_publisher.publish(pose)
        frame += 1
        if ARGS.test_seconds > 0 and timeline.get_current_time() - started_at >= ARGS.test_seconds:
            break

    if ARGS.test_seconds > 0 and curtain_particle_count:
        position, orientation = robot.get_world_pose()
        curtain = UsdGeom.Mesh.Get(stage, "/World/DoorCurtain/Cloth")
        curtain_transform = UsdGeom.XformCache().GetLocalToWorldTransform(
            curtain.GetPrim()
        )
        curtain_points = np.asarray([
            curtain_transform.Transform(point)
            for point in curtain.GetPointsAttr().Get()
        ])
        curtain_rest_x = np.tile(
            CURTAIN_X
            + 0.015
            * np.sin(
                6.0
                * math.pi
                * np.arange(CURTAIN_COLUMNS)
                / (CURTAIN_COLUMNS - 1)
            ),
            CURTAIN_ROWS,
        )
        curtain_x_deflection = curtain_points[:, 0] - curtain_rest_x
        d435_position = UsdGeom.XformCache().GetLocalToWorldTransform(
            stage.GetPrimAtPath(camera_path)
        ).ExtractTranslation()
        print(f"Final test pose: position={position}, orientation={orientation}", flush=True)
        print(
            f"Final test velocities: linear={robot.get_linear_velocity()}, "
            f"angular={robot.get_angular_velocity()}, "
            f"wheels={robot.get_joint_velocities(joint_indices=wheel_indices)}",
            flush=True,
        )
        print(
            f"Final Piper joints: {robot.get_joint_positions(joint_indices=arm_indices)}",
            flush=True,
        )
        print(f"Final D435 position: {d435_position}", flush=True)
        print(
            f"Final curtain: top_z=({curtain_points[:CURTAIN_COLUMNS, 2].min():.3f},"
            f"{curtain_points[:CURTAIN_COLUMNS, 2].max():.3f}), "
            f"bottom_z=({curtain_points[-CURTAIN_COLUMNS:, 2].min():.3f},"
            f"{curtain_points[-CURTAIN_COLUMNS:, 2].max():.3f}), "
            f"x_span=({curtain_points[:, 0].min():.3f},"
            f"{curtain_points[:, 0].max():.3f}), "
            f"x_deflection=({curtain_x_deflection.min():.3f},"
            f"{curtain_x_deflection.max():.3f}), "
            f"x_rms={np.sqrt(np.mean(curtain_x_deflection ** 2)):.3f}",
            flush=True,
        )

    timeline.stop()
    scan_writer.detach()
    if debug_writer is not None:
        debug_writer.detach()
    render_product.destroy()
    map_node.destroy_node()
    rclpy.shutdown()
    APP.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"Isaac Sim setup failed: {error}", flush=True)
        APP.close()
        raise
