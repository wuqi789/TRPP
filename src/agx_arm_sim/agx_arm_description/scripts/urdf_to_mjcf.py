#!/usr/bin/env python3
"""Generate MuJoCo MJCF models from the D435 URDF variants."""

from __future__ import annotations

import os
import re
from collections import defaultdict
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PACKAGE_ROOT.parent
URDF_DIR = PACKAGE_ROOT / "urdf"
MUJOCO_DIR = PACKAGE_ROOT / "mujoco"
MUJOCO_MESH_DIR = MUJOCO_DIR / "meshes"

MODELS = (
    ("nero_gripper_d435.urdf", "nero_gripper_d435.xml"),
    ("piper_girpper_D435.urdf", "piper_girpper_D435.xml"),
    ("piper_h_gripper_d435.urdf", "piper_h_gripper_d435.xml"),
    ("piper_l_gripper_d435.urdf", "piper_l_gripper_d435.xml"),
    ("piper_x_gripper_d435.urdf", "piper_x_gripper_d435.xml"),
)

PACKAGE_DIRS = {
    "agx_arm_description": PACKAGE_ROOT,
    "realsense2_description": SRC_ROOT / "realsense2_description",
}

DAE_REPLACEMENTS = {
    (PACKAGE_ROOT / "meshes" / "realsense_mid_stand.dae").resolve(): MUJOCO_MESH_DIR
    / "realsense_mid_stand.stl",
    (SRC_ROOT / "realsense2_description" / "meshes" / "d435.dae").resolve(): MUJOCO_MESH_DIR
    / "d435.stl",
}


def fmt(value: float | str) -> str:
    number = float(value)
    if abs(number) < 1e-12:
        number = 0.0
    return f"{number:.10g}"


def vec(values: list[float] | tuple[float, ...]) -> str:
    return " ".join(fmt(value) for value in values)


def parse_vec(text: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if text is None:
        return default
    values = tuple(float(part) for part in text.split())
    if len(values) != len(default):
        raise ValueError(f"Expected {len(default)} values, got {text!r}")
    return values


def parse_origin(node: ET.Element | None) -> tuple[tuple[float, ...], tuple[float, ...]]:
    if node is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = parse_vec(node.get("xyz"), (0.0, 0.0, 0.0))
    rpy = parse_vec(node.get("rpy"), (0.0, 0.0, 0.0))
    return xyz, rpy


def set_pose_attrs(attrs: dict[str, str], xyz: tuple[float, ...], rpy: tuple[float, ...]) -> None:
    if any(abs(value) > 1e-12 for value in xyz):
        attrs["pos"] = vec(xyz)
    if any(abs(value) > 1e-12 for value in rpy):
        attrs["euler"] = vec(rpy)


def sanitize(name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_]", "_", name)
    if not cleaned or cleaned[0].isdigit():
        return f"m_{cleaned}"
    return cleaned


def resolve_uri(uri: str) -> Path:
    if uri.startswith("package://"):
        package, relpath = uri[len("package://") :].split("/", 1)
        if package not in PACKAGE_DIRS:
            raise ValueError(f"Unsupported package URI: {uri}")
        return (PACKAGE_DIRS[package] / relpath).resolve()
    path = Path(uri)
    if path.is_absolute():
        return path.resolve()
    return (URDF_DIR / path).resolve()


def relative_to_mujoco(path: Path) -> str:
    return os.path.relpath(path, MUJOCO_DIR).replace(os.sep, "/")


def material_for_link(link_name: str) -> str:
    if "camera" in link_name or "d435" in link_name:
        return "camera"
    if "gripper" in link_name:
        return "gripper"
    return "robot"


class MjcfBuilder:
    def __init__(self, urdf_path: Path, output_path: Path):
        self.urdf_path = urdf_path
        self.output_path = output_path
        self.asset_meshes: dict[tuple[str, tuple[float, ...]], str] = {}
        self.used_asset_names: set[str] = set()
        self.movable_joints: list[dict[str, str | None]] = []
        self.warnings: list[str] = []

        self.urdf = ET.parse(urdf_path).getroot()
        self.links = {link.get("name"): link for link in self.urdf.findall("link")}
        self.children: dict[str, list[ET.Element]] = defaultdict(list)
        self.child_links: set[str] = set()
        for joint in self.urdf.findall("joint"):
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            self.children[parent].append(joint)
            self.child_links.add(child)

    def build(self) -> ET.ElementTree:
        model_name = self.output_path.stem
        root = ET.Element("mujoco", {"model": model_name})
        root.append(
            ET.Comment(
                f"Generated from {self.urdf_path.name} by scripts/urdf_to_mjcf.py. "
                "STL meshes are used for MuJoCo compatibility."
            )
        )

        ET.SubElement(
            root,
            "compiler",
            {
                "angle": "radian",
                "eulerseq": "xyz",
                "autolimits": "true",
            },
        )
        ET.SubElement(
            root,
            "option",
            {
                "timestep": "0.002",
                "gravity": "0 0 -9.81",
            },
        )

        visual = ET.SubElement(root, "visual")
        ET.SubElement(visual, "headlight", {"diffuse": "0.5 0.5 0.5", "ambient": "0.2 0.2 0.2"})
        ET.SubElement(visual, "map", {"znear": "0.01", "zfar": "20"})

        defaults = ET.SubElement(root, "default")
        ET.SubElement(defaults, "joint", {"damping": "1", "armature": "0.01"})
        ET.SubElement(
            defaults,
            "geom",
            {
                "condim": "3",
                "friction": "1 0.005 0.0001",
                "margin": "0.001",
            },
        )
        visual_default = ET.SubElement(defaults, "default", {"class": "visual"})
        ET.SubElement(visual_default, "geom", {"contype": "0", "conaffinity": "0", "group": "1"})

        self.asset = ET.SubElement(root, "asset")
        ET.SubElement(self.asset, "material", {"name": "robot", "rgba": "0.62 0.64 0.67 1"})
        ET.SubElement(self.asset, "material", {"name": "gripper", "rgba": "0.18 0.18 0.2 1"})
        ET.SubElement(self.asset, "material", {"name": "camera", "rgba": "0.05 0.06 0.07 1"})
        ET.SubElement(self.asset, "material", {"name": "ground", "rgba": "0.55 0.57 0.58 1"})

        worldbody = ET.SubElement(root, "worldbody")
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": "ground",
                "type": "plane",
                "size": "3 3 0.02",
                "material": "ground",
            },
        )
        ET.SubElement(
            worldbody,
            "light",
            {
                "name": "key_light",
                "pos": "1.5 -2.5 3",
                "dir": "-0.4 0.6 -1",
                "directional": "true",
                "diffuse": "0.8 0.8 0.8",
                "specular": "0.2 0.2 0.2",
            },
        )
        ET.SubElement(
            worldbody,
            "light",
            {
                "name": "fill_light",
                "pos": "-2 2 2",
                "dir": "0.6 -0.4 -1",
                "directional": "true",
                "diffuse": "0.35 0.35 0.35",
            },
        )
        ET.SubElement(
            worldbody,
            "camera",
            {
                "name": "overview",
                "pos": "1.2 -1.8 1.1",
                "xyaxes": "0.83205 0.5547 0 -0.30769 0.46154 0.83205",
            },
        )

        for root_link, root_joint in self.root_entries():
            self.add_body(worldbody, root_link, root_joint)

        actuator = ET.SubElement(root, "actuator")
        for joint in self.movable_joints:
            self.add_position_actuator(actuator, joint)

        return ET.ElementTree(root)

    def root_entries(self) -> list[tuple[str, ET.Element | None]]:
        if "world" in self.links and self.children.get("world"):
            return [(joint.find("child").get("link"), joint) for joint in self.children["world"]]
        return [(name, None) for name in self.links if name not in self.child_links]

    def add_body(self, parent_node: ET.Element, link_name: str, joint: ET.Element | None) -> None:
        attrs = {"name": link_name}
        if joint is not None:
            xyz, rpy = parse_origin(joint.find("origin"))
            set_pose_attrs(attrs, xyz, rpy)
        body = ET.SubElement(parent_node, "body", attrs)

        link = self.links[link_name]
        self.add_inertial(body, link)
        if joint is not None:
            self.add_joint(body, joint)
        self.add_link_geoms(body, link)

        for child_joint in self.children.get(link_name, []):
            child_name = child_joint.find("child").get("link")
            self.add_body(body, child_name, child_joint)

    def add_inertial(self, body: ET.Element, link: ET.Element) -> None:
        inertial = link.find("inertial")
        if inertial is None:
            return
        mass_node = inertial.find("mass")
        inertia_node = inertial.find("inertia")
        if mass_node is None or inertia_node is None:
            return

        xyz, rpy = parse_origin(inertial.find("origin"))
        attrs = {
            "mass": fmt(mass_node.get("value")),
            "pos": vec(xyz),
            "fullinertia": vec(
                tuple(
                    float(inertia_node.get(key, "0"))
                    for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
                )
            ),
        }
        if any(abs(value) > 1e-12 for value in rpy):
            attrs["euler"] = vec(rpy)
        ET.SubElement(body, "inertial", attrs)

    def add_joint(self, body: ET.Element, joint: ET.Element) -> None:
        joint_type = joint.get("type", "fixed")
        if joint_type == "fixed":
            return
        if joint_type in ("revolute", "continuous"):
            mjcf_type = "hinge"
        elif joint_type == "prismatic":
            mjcf_type = "slide"
        else:
            self.warnings.append(f"Skipped unsupported joint type {joint_type!r} on {joint.get('name')}")
            return

        axis_node = joint.find("axis")
        axis = parse_vec(axis_node.get("xyz") if axis_node is not None else None, (1.0, 0.0, 0.0))
        attrs = {
            "name": joint.get("name"),
            "type": mjcf_type,
            "axis": vec(axis),
        }

        limit = joint.find("limit")
        lower = upper = effort = None
        if limit is not None:
            lower = limit.get("lower")
            upper = limit.get("upper")
            effort = limit.get("effort")
            if lower is not None and upper is not None:
                attrs["range"] = f"{fmt(lower)} {fmt(upper)}"
                attrs["limited"] = "true"

        dynamics = joint.find("dynamics")
        if dynamics is not None and dynamics.get("damping") is not None:
            attrs["damping"] = fmt(dynamics.get("damping"))

        ET.SubElement(body, "joint", attrs)
        self.movable_joints.append(
            {
                "name": joint.get("name"),
                "lower": lower,
                "upper": upper,
                "effort": effort,
                "is_gripper": "gripper" in joint.get("name", ""),
            }
        )

    def add_link_geoms(self, body: ET.Element, link: ET.Element) -> None:
        link_name = link.get("name")
        mesh_paths_used: set[Path] = set()

        for index, collision in enumerate(link.findall("collision")):
            geom_node = collision.find("geometry")
            if geom_node is None:
                continue
            xyz, rpy = parse_origin(collision.find("origin"))
            used_path = self.add_geom_from_urdf_geometry(
                body,
                geom_node,
                f"{link_name}_{index}_collision",
                link_name,
                xyz,
                rpy,
                visual_only=False,
            )
            if used_path is not None:
                mesh_paths_used.add(used_path)

        for index, visual in enumerate(link.findall("visual")):
            geom_node = visual.find("geometry")
            mesh_node = geom_node.find("mesh") if geom_node is not None else None
            if mesh_node is None:
                continue
            if link_name == "camera_link":
                # The camera body is represented by its URDF collision box in MJCF.
                continue
            mesh_path = self.resolve_mesh_path(mesh_node)
            if mesh_path is None or mesh_path in mesh_paths_used:
                continue
            if mesh_path.suffix.lower() != ".stl":
                continue
            xyz, rpy = parse_origin(visual.find("origin"))
            self.add_mesh_geom(
                body,
                mesh_path,
                parse_vec(mesh_node.get("scale"), (1.0, 1.0, 1.0)),
                f"{link_name}_{index}_visual",
                material_for_link(link_name),
                xyz,
                rpy,
                visual_only=True,
            )

    def add_geom_from_urdf_geometry(
        self,
        body: ET.Element,
        geom_node: ET.Element,
        geom_name: str,
        link_name: str,
        xyz: tuple[float, ...],
        rpy: tuple[float, ...],
        visual_only: bool,
    ) -> Path | None:
        mesh_node = geom_node.find("mesh")
        if mesh_node is not None:
            mesh_path = self.resolve_mesh_path(mesh_node)
            if mesh_path is None:
                return None
            if mesh_path.suffix.lower() != ".stl":
                self.warnings.append(f"Skipped non-STL mesh {mesh_path} on {link_name}")
                return None
            self.add_mesh_geom(
                body,
                mesh_path,
                parse_vec(mesh_node.get("scale"), (1.0, 1.0, 1.0)),
                geom_name,
                material_for_link(link_name),
                xyz,
                rpy,
                visual_only,
            )
            return mesh_path

        box_node = geom_node.find("box")
        if box_node is not None:
            size = tuple(value / 2.0 for value in parse_vec(box_node.get("size"), (1.0, 1.0, 1.0)))
            self.add_simple_geom(body, "box", size, geom_name, material_for_link(link_name), xyz, rpy, visual_only)
            return None

        cylinder_node = geom_node.find("cylinder")
        if cylinder_node is not None:
            radius = float(cylinder_node.get("radius"))
            half_length = float(cylinder_node.get("length")) / 2.0
            self.add_simple_geom(
                body,
                "cylinder",
                (radius, half_length),
                geom_name,
                material_for_link(link_name),
                xyz,
                rpy,
                visual_only,
            )
            return None

        sphere_node = geom_node.find("sphere")
        if sphere_node is not None:
            radius = float(sphere_node.get("radius"))
            self.add_simple_geom(body, "sphere", (radius,), geom_name, material_for_link(link_name), xyz, rpy, visual_only)
            return None

        self.warnings.append(f"Skipped unsupported geometry on {link_name}")
        return None

    def resolve_mesh_path(self, mesh_node: ET.Element) -> Path | None:
        mesh_path = resolve_uri(mesh_node.get("filename"))
        replacement = DAE_REPLACEMENTS.get(mesh_path)
        if replacement is not None:
            if replacement.exists():
                return replacement.resolve()
            self.warnings.append(f"Replacement mesh is missing for {mesh_path}: {replacement}")
            return None
        return mesh_path

    def add_mesh_geom(
        self,
        body: ET.Element,
        mesh_path: Path,
        scale: tuple[float, ...],
        geom_name: str,
        material: str,
        xyz: tuple[float, ...],
        rpy: tuple[float, ...],
        visual_only: bool,
    ) -> None:
        asset_name = self.add_mesh_asset(mesh_path, scale, geom_name)
        attrs = {
            "name": sanitize(geom_name),
            "type": "mesh",
            "mesh": asset_name,
            "material": material,
        }
        if visual_only:
            attrs["class"] = "visual"
        set_pose_attrs(attrs, xyz, rpy)
        ET.SubElement(body, "geom", attrs)

    def add_simple_geom(
        self,
        body: ET.Element,
        geom_type: str,
        size: tuple[float, ...],
        geom_name: str,
        material: str,
        xyz: tuple[float, ...],
        rpy: tuple[float, ...],
        visual_only: bool,
    ) -> None:
        attrs = {
            "name": sanitize(geom_name),
            "type": geom_type,
            "size": vec(size),
            "material": material,
        }
        if visual_only:
            attrs["class"] = "visual"
        set_pose_attrs(attrs, xyz, rpy)
        ET.SubElement(body, "geom", attrs)

    def add_mesh_asset(self, mesh_path: Path, scale: tuple[float, ...], name_hint: str) -> str:
        relpath = relative_to_mujoco(mesh_path)
        key = (relpath, scale)
        if key in self.asset_meshes:
            return self.asset_meshes[key]

        base_name = sanitize(f"{name_hint}_mesh")
        asset_name = base_name
        suffix = 2
        while asset_name in self.used_asset_names:
            asset_name = f"{base_name}_{suffix}"
            suffix += 1
        self.used_asset_names.add(asset_name)

        attrs = {"name": asset_name, "file": relpath}
        if any(abs(value - 1.0) > 1e-12 for value in scale):
            attrs["scale"] = vec(scale)
        ET.SubElement(self.asset, "mesh", attrs)
        self.asset_meshes[key] = asset_name
        return asset_name

    def add_position_actuator(self, actuator: ET.Element, joint: dict[str, str | None]) -> None:
        name = str(joint["name"])
        kp = "80" if joint["is_gripper"] else "120"
        attrs = {
            "name": sanitize(f"{name}_position"),
            "joint": name,
            "kp": kp,
        }

        lower = joint.get("lower")
        upper = joint.get("upper")
        if lower is not None and upper is not None:
            attrs["ctrlrange"] = f"{fmt(lower)} {fmt(upper)}"
            attrs["ctrllimited"] = "true"

        effort = joint.get("effort")
        if effort is not None:
            effort_value = abs(float(effort))
            attrs["forcerange"] = f"{fmt(-effort_value)} {fmt(effort_value)}"
            attrs["forcelimited"] = "true"

        ET.SubElement(actuator, "position", attrs)

    def write(self) -> None:
        tree = self.build()
        ET.indent(tree, space="  ")
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        tree.write(self.output_path, encoding="utf-8", xml_declaration=True)
        if self.warnings:
            print(f"{self.output_path.name}:")
            for warning in self.warnings:
                print(f"  warning: {warning}")


def main() -> None:
    for urdf_name, output_name in MODELS:
        builder = MjcfBuilder(URDF_DIR / urdf_name, MUJOCO_DIR / output_name)
        builder.write()


if __name__ == "__main__":
    main()
