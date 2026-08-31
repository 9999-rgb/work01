#!/usr/bin/env python3
"""Build static (or dynamic) scene models as xacro from SW2URDF exports.

The user's scene packages (``仿真场景20260831/{发电机层3.0,电气夹层3.0}/``) are
SW2URDF exports: a URDF whose links reference ``package://<中文名>/meshes/*.STL``.
Gazebo's URDF->SDF conversion drops large meshes, so the scene floor is instead
provided as an SDF model spawned by the scene switch (entity ``xczs_scene_floor``).
This tool regenerates that model as a **xacro** (root ``<sdf>`` so Gazebo keeps
the full meshes) from the export:

  * rewrites every mesh URI to
    ``model://xczs_inspection_robot_description/meshes/scenes/<scene>/<file>.STL``
  * keeps all fixture links/joints at their zero pose (full visual fidelity;
    Phase B re-runs with ``--dynamic`` to make them movable)
  * substitutes the ground link's 83.7 MB / 1.67 M-triangle mesh collision with
    a few AABB box proxies (a floor slab + equipment silhouettes) so the physics
    engine does not choke on it; the *visual* keeps the full mesh untouched

Run (Phase A, static):
    python3 scripts/tools/build_scene_sdf.py \
        --urdf 仿真场景20260831/发电机层3.0/urdf/发电机层3.0.urdf \
        --name generator_plant \
        --out xczs_inspection_robot_description/urdf/scenes/generator_plant.xacro

Run (Phase B, dynamic fixture):
    python3 scripts/tools/build_scene_sdf.py \
        --urdf 仿真场景20260831/发电机层3.0/urdf/发电机层3.0.urdf \
        --name generator_plant \
        --out xczs_inspection_robot_description/urdf/scenes/generator_plant.xacro \
        --dynamic

``--dynamic`` re-runs the same export as a movable fixture model:
``static=false``, the ground link anchored to the implicit SDF world frame
(no empty ``world`` link -- a 0-mass body in a movable model poisons the ODE
solver into NaN joint velocities, so the anchor joint binds ``parent=world``
directly), and a ``libxczs_cabinet_state`` plugin block driven by the per-scene
controls catalog (``xczs_inspection_robot_control/config/scene_controls/
<name>_controls.yaml``), so buttons/knobs carry the same plugin state machine,
springs and grasp coupling as the shared control cabinet -- with zero C++
changes.

Requires only the Python stdlib for the URDF->SDF conversion; ``yaml`` is needed
for ``--dynamic`` and ``vtk`` only for the optional
``--ground-collision-boxes`` proxy computation.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

# ---------------------------------------------------------------------------
# URDF -> intermediate representation
# ---------------------------------------------------------------------------

# Minimal mass/inertia floor for links SW2URDF left at zero (e.g. the
# generator's r25452 shaft link).  Gazebo rejects zero-mass dynamic links.
_MIN_MASS = 1e-3
_MIN_INERTIA = 1e-9


@dataclass
class LinkSpec:
    name: str
    mass: float
    inertial_pose: list[float]  # xyz rpy, link frame -> COM frame
    inertia: list[float]  # ixx ixy ixz iyy iyz izz about the COM frame
    visual_uri: str | None
    visual_rgba: list[float]
    is_ground: bool


@dataclass
class JointSpec:
    name: str
    type: str  # revolute | prismatic | fixed | continuous
    parent: str
    child: str
    pose: list[float]  # xyz rpy, parent frame -> joint frame
    axis: list[float]  # joint axis (joint frame)
    limits: dict[str, float]  # lower/upper/effort/velocity
    dynamics: dict[str, float]  # damping/friction


@dataclass
class SceneSpec:
    name: str
    links: list[LinkSpec] = field(default_factory=list)
    joints: list[JointSpec] = field(default_factory=list)

    @property
    def ground(self) -> LinkSpec:
        for link in self.links:
            if link.is_ground:
                return link
        raise ValueError(f"scene {self.name!r} has no ground link")


def _f(elem: ET.Element | None, tag: str, default: float = 0.0) -> float:
    """Read a float attribute in either of SW2URDF's two XML shapes.

    The exports mix two forms -- ``<mass value=..>`` (value on a child element)
    and ``<limit lower=..>`` / ``<inertia ixx=..>`` (attribute directly on the
    element itself).  Try the direct attribute first, then the child element's
    ``value`` / tag-named attribute.
    """
    if elem is None:
        return default
    val = elem.get(tag)
    if val is None:
        child = elem.find(tag)
        if child is not None:
            val = child.get("value") or child.get(tag)
    return float(val) if val is not None else default


def _vec(elem: ET.Element | None, tag: str) -> list[float]:
    """Read an ``xyz`` tuple from either ``elem`` or a child of the same tag.

    ``<axis xyz=..>`` is an attribute on the element itself, while a link's
    inertial frame is ``<inertial><origin xyz=..>`` -- a child element.  Both
    shapes are present in the SW2URDF exports.
    """
    if elem is None:
        return [0.0, 0.0, 0.0]
    val = elem.get(tag)
    if val is None:
        child = elem.find(tag)
        if child is not None:
            val = child.get("xyz") or child.get(tag)
    if val is None:
        return [0.0, 0.0, 0.0]
    return [float(v) for v in val.split()]


def _origin(elem: ET.Element | None) -> list[float]:
    """Read a URDF ``<origin xyz=.. rpy=..>`` as ``[x, y, z, r, p, y]``.

    The export puts the transform on a child ``<origin>`` element (joints,
    inertial frames); the element itself may also carry the attributes.
    """
    o = elem
    if o is not None and o.get("xyz") is None:
        child = o.find("origin")
        if child is not None:
            o = child
    if o is None:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    xyz = [float(v) for v in o.get("xyz").split()] if o.get("xyz") else [0.0] * 3
    rpy = [float(v) for v in o.get("rpy").split()] if o.get("rpy") else [0.0] * 3
    return xyz + rpy


def _pose(xyz: list[float], rpy: list[float]) -> list[float]:
    return xyz + rpy


def parse_urdf(urdf_path: Path, *, scene_name: str) -> SceneSpec:
    """Parse an SW2URDF export into a SceneSpec (mesh URIs rewritten)."""
    tree = ET.parse(urdf_path)
    robot = tree.getroot()
    if robot.tag != "robot":
        raise ValueError(f"{urdf_path} is not a URDF <robot> document")

    spec = SceneSpec(name=scene_name)
    for link in robot.findall("link"):
        name = link.get("name", "")
        inertial = link.find("inertial")
        mass = max(_f(inertial, "mass", 0.0), _MIN_MASS)
        inertia_elem = inertial.find("inertia") if inertial is not None else None
        i = [max(_f(inertia_elem, tag), _MIN_INERTIA) for tag in
             ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")]
        inertial_pose = _origin(inertial)

        # Visual / collision geometry share the same mesh in these exports.
        geom = link.find("visual/geometry/mesh")
        material = link.find("visual/material/color")
        rgba = (
            [float(v) for v in (material.get("rgba", "1 1 1 1").split())]
            if material is not None else [1.0, 1.0, 1.0, 1.0]
        )
        uri = None
        if geom is not None:
            fname = geom.get("filename", "")
            mesh_name = Path(fname).name
            uri = (
                f"model://xczs_inspection_robot_description/meshes/scenes/"
                f"{scene_name}/{mesh_name}"
            )
        spec.links.append(
            LinkSpec(
                name=name,
                mass=mass,
                inertial_pose=inertial_pose,
                inertia=i,
                visual_uri=uri,
                visual_rgba=rgba,
                is_ground=(name in ("fadianground", "dianqiground")),
            )
        )

    for joint in robot.findall("joint"):
        jtype = joint.get("type", "fixed")
        parent_elem = joint.find("parent")
        child_elem = joint.find("child")
        spec.joints.append(
            JointSpec(
                name=joint.get("name", ""),
                type=jtype,
                parent=parent_elem.get("link", "") if parent_elem is not None else "",
                child=child_elem.get("link", "") if child_elem is not None else "",
                pose=_origin(joint),
                axis=_vec(joint, "axis"),
                limits={
                    "lower": _f(joint.find("limit"), "lower", 0.0),
                    "upper": _f(joint.find("limit"), "upper", 0.0),
                    "effort": _f(joint.find("limit"), "effort", 100.0),
                    "velocity": _f(joint.find("limit"), "velocity", 1.0),
                },
                dynamics={
                    "damping": _f(joint.find("dynamics"), "damping", 0.1),
                    "friction": _f(joint.find("dynamics"), "friction", 1.0),
                },
            )
        )
    # URDF puts links and joints in separate namespaces, so a door's driving
    # joint may share a name with its handle/button link (the electrical
    # export does: joint ``s1r`` drives link ``s1r``).  SDF requires every
    # model child -- links *and* joints -- to have a unique name, and Gazebo
    # silently drops the colliding joint.  Rename the joint (the operation
    # target) with a ``_joint`` suffix; its parent/child fields already
    # reference link names, so nothing else needs updating.
    used_names = {link.name for link in spec.links}
    for joint in spec.joints:
        if joint.name in used_names:
            joint.name = f"{joint.name}_joint"
        used_names.add(joint.name)
    return spec


# ---------------------------------------------------------------------------
# Ground collision box proxies (vtk, optional)
# ---------------------------------------------------------------------------

def _compute_ground_collision_boxes(
    ground_uri: str,
    *,
    mesh_dir: Path,
    resolution: float = 1.0,
    slab_thickness: float = 0.2,
    slab_margin: float = 0.5,
    floor_threshold: float = 0.3,
    clearance_height: float = 0.9,
    min_region_cells: int = 1,
) -> list[dict[str, object]]:
    """Return AABB box proxies (pose + size) for the ground link.

    The ground STL is a closed slab with walls / equipment casings rising
    above it.  Each coarse grid cell is classified with the same vertical
    ray-cast parity test the nav2 map generator uses, but split three ways:

      * FLOOR -- open air at the robot's mid-body height (drivable),
      * WALL  -- material at mid-body height above a floor (blocked),
      * HOLE  -- no floor surface below (off the building footprint).

    WALL cells are clustered (8-connectivity) into regions and each region
    becomes one box, tall enough to block the robot (1.5--2.5 m).  The visual
    mesh is left untouched.  Returns boxes with ``pose`` (xyz rpy) + ``size``.

    Uses the same vertical ray-cast *interval* classification as the nav2 map
    generator: a cell is WALL when some solid interval (an enter/exit pair of
    ray crossings, assuming a closed mesh) overlaps the robot body band
    ``[floor_threshold, clearance_height]``.
    """
    try:
        import vtk
    except ImportError as error:  # pragma: no cover - dev tool
        raise SystemExit(
            "Ground collision boxes need the 'vtk' Python module "
            "(present in the ROS 2 Humble environment)."
        ) from error

    mesh_name = Path(ground_uri).name
    mesh_path = mesh_dir / mesh_name
    if not mesh_path.is_file():
        raise FileNotFoundError(f"ground mesh not found: {mesh_path}")

    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(mesh_path))
    reader.Update()
    polydata = reader.GetOutput()
    # No decimation here: ``classify_cell`` buckets the mesh with a
    # ``vtkStaticCellLocator`` and ray-casts each candidate triangle exactly in
    # numpy (~85 us/cell), so even the 1.67 M-triangle generator ground scans a
    # coarse box grid in well under a second.  Decimating would only blur the
    # thin button pedestals that must stay blockable.  (The nav2 *map*
    # generator decimates a separate in-memory copy -- see its docstring.)
    xmin, xmax, ymin, ymax, zmin, zmax = polydata.GetBounds()
    # The driving surface is the mesh's *lowest* horizontal face (verified for
    # the generator ground: the dominant flat sits at ``zmin``, not
    # ``zmin + slab_thickness``).  The slab box therefore spans
    # ``[zmin - slab_thickness, zmin]`` with its top exactly at the floor
    # surface, so a robot wheel rests cleanly ON the floor instead of wedging
    # halfway into an over-thick block (which the generator scene reproduced as
    # a delayed +0.14 m settle that kicked the gripper fingers open).
    slab_top = zmin
    boxes: list[dict[str, object]] = [{
        "pose": [
            round((xmin + xmax) / 2.0, 3),
            round((ymin + ymax) / 2.0, 3),
            round(zmin - slab_thickness / 2.0, 3),
            0.0, 0.0, 0.0,
        ],
        "size": [
            round((xmax - xmin) + 2.0 * slab_margin, 3),
            round((ymax - ymin) + 2.0 * slab_margin, 3),
            round(slab_thickness, 3),
        ],
    }]

    from _scene_ray_classify import build_locator  # dev-tool sibling
    from _scene_ray_classify import classify_cell
    from _scene_ray_classify import extract_triangles
    from _scene_ray_classify import vertical_ray_zs

    locator = build_locator(polydata)
    tri_pts = extract_triangles(polydata)

    z_high = zmax + 1.0
    z_low = zmin - 1.0
    cols = int((xmax - xmin) / resolution) + 1
    rows = int((ymax - ymin) / resolution) + 1
    walls: dict[tuple[int, int], float] = {}  # (col,row) -> roof height
    cell_ids = vtk.vtkIdList()
    for row in range(rows):
        y = ymin + (row + 0.5) * resolution
        for col in range(cols):
            x = xmin + (col + 0.5) * resolution
            kind = classify_cell(
                locator,
                tri_pts,
                x,
                y,
                z_high=z_high,
                z_low=z_low,
                floor_threshold=floor_threshold,
                clearance_height=clearance_height,
                cell_ids=cell_ids,
            )
            if kind == "wall":
                # A wall / casing reaches into the robot's body band; remember
                # its roof height for the box extent.
                ids_arr = np.array(
                    [cell_ids.GetId(i) for i in range(cell_ids.GetNumberOfIds())],
                    dtype=np.int64,
                )
                zs = vertical_ray_zs(ids_arr, tri_pts, x, y, z_high, z_low)
                walls[(col, row)] = float(zs[0]) if zs.size else z_high

    # 8-connectivity clustering of WALL cells into AABB regions.
    remaining = set(walls)
    regions: list[list[tuple[int, int]]] = []
    while remaining:
        seed = remaining.pop()
        region = [seed]
        stack = [seed]
        while stack:
            c, r = stack.pop()
            for dc, dr in ((-1, -1), (-1, 0), (-1, 1), (0, -1),
                           (0, 1), (1, -1), (1, 0), (1, 1)):
                neighbor = (c + dc, r + dr)
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    region.append(neighbor)
                    stack.append(neighbor)
        regions.append(region)

    def _emit_box(cells: set[tuple[int, int]]) -> None:
        cols_r = [cell[0] for cell in cells]
        rows_r = [cell[1] for cell in cells]
        c0, c1 = min(cols_r), max(cols_r)
        r0, r1 = min(rows_r), max(rows_r)
        margin = 0.5 * resolution
        x0 = xmin + c0 * resolution - margin
        x1 = xmin + (c1 + 1) * resolution + margin
        y0 = ymin + r0 * resolution - margin
        y1 = ymin + (r1 + 1) * resolution + margin
        # Blocking height: median roof height over the cells, clamped so the
        # box is always tall enough to stop the robot but never floats absurdly.
        roofs = sorted(walls[cell] for cell in cells)
        height = max(1.5, min(2.5, roofs[len(roofs) // 2] - slab_top))
        boxes.append({
            "pose": [
                round(0.5 * (x0 + x1), 3),
                round(0.5 * (y0 + y1), 3),
                round(slab_top + 0.5 * height, 3),
                0.0, 0.0, 0.0,
            ],
            "size": [
                round(x1 - x0, 3),
                round(y1 - y0, 3),
                round(height, 3),
            ],
        })

    for region in regions:
        if len(region) < min_region_cells:
            continue
        cols_r = [cell[0] for cell in region]
        rows_r = [cell[1] for cell in region]
        c0, c1 = min(cols_r), max(cols_r)
        r0, r1 = min(rows_r), max(rows_r)
        bbox_area = (c1 - c0 + 1) * (r1 - r0 + 1)
        fill = len(region) / bbox_area
        if fill >= 0.4:
            # A solid equipment block / wall run: one AABB is faithful.
            _emit_box(region)
        else:
            # Scattered obstacles (isolated columns, thin walls, cable trays).
            # A single AABB over all of them would swallow the surrounding
            # free space and block navigation the map says is drivable, so
            # emit one small box per WALL cell instead.
            for cell in region:
                _emit_box({cell})
    return boxes


# ---------------------------------------------------------------------------
# Plugin control block (--dynamic)
# ---------------------------------------------------------------------------

# Constant plugin parameters mirroring the shared control-cabinet model
# (urdf/control_cabinet/components/gazebo.xacro): buttons are pushed by a
# compliant spring the plugin applies to the prismatic joint; knobs carry
# three detents and the bounded one-DOF grasp coupling used for rotary tools.
_BUTTON_SPRING_DAMPING = 8.0
_KNOB_DETENT_STIFFNESS = 0.60
_KNOB_SPRING_DAMPING = 0.08
_KNOB_GRASP_STIFFNESS = 0.0
_KNOB_GRASP_DAMPING = 0.0
_KNOB_COUPLING_STIFFNESS = 5.0
_KNOB_COUPLING_DAMPING = 0.005
_KNOB_COUPLING_MAX_EFFORT = 6.0
_KNOB_MOTION_TOLERANCE = 0.025
_KNOB_GRASP_POINT = "0 0 0"


def _load_controls(controls_path: Path, *, namespace: str) -> list[dict[str, object]]:
    """Read a per-scene controls catalog into plugin ``<control>`` specs.

    The controls catalog (``scene_controls/<scene>_controls.yaml``) is the
    cross-layer contract: the same ``control_ids``/``controls`` the operator and
    planning scene consume.  Only ``button``/``knob`` kinds are emitted here;
    ``door``/``switch`` raise (the electrical scene adds them in Phase B3).
    """
    try:
        import yaml
    except ImportError as error:  # pragma: no cover - dev tool
        raise ValueError(
            "Plugin control emission needs the 'yaml' Python module."
        ) from error
    try:
        document = yaml.safe_load(controls_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(
            f"cannot read controls catalog {controls_path}: {error}"
        ) from error
    except yaml.YAMLError as error:
        raise ValueError(
            f"invalid controls catalog {controls_path}: {error}"
        ) from error
    try:
        parameters = document["/**"]["ros__parameters"]
        control_ids = parameters["control_ids"]
        controls_map = parameters["controls"]
    except (KeyError, TypeError) as error:
        raise ValueError(
            "controls catalog must define /**/ros__parameters.control_ids "
            "and controls."
        ) from error

    controls: list[dict[str, object]] = []
    for control_id in control_ids:
        spec = controls_map.get(control_id)
        if not isinstance(spec, dict):
            raise ValueError(f"controls catalog is missing '{control_id}'.")
        ctype = spec.get("type")
        if ctype == "button":
            press = spec.get("press_threshold", 0.006)
            controls.append(
                {
                    "control_id": control_id,
                    "type": "button",
                    "joint_name": spec["joint_name"],
                    "state_ids": " ".join(spec.get("state_ids", [])),
                    "spring_stiffness": spec.get("spring_stiffness", 800.0),
                    "press_threshold": press,
                    # ``properties.xacro`` halves the press threshold for the
                    # release edge so a lightly-touched button latches and a
                    # release is unambiguous.
                    "release_threshold": 0.5 * press,
                }
            )
        elif ctype == "knob":
            controls.append(
                {
                    "control_id": control_id,
                    "type": "knob",
                    "joint_name": spec["joint_name"],
                    "state_ids": " ".join(spec.get("state_ids", [])),
                    # Physical detents latch at the state positions, matching
                    # the operator's state_positions so both agree on state.
                    "detents": " ".join(
                        str(value) for value in spec.get("state_positions", [])
                    ),
                }
            )
        else:
            raise ValueError(
                f"plugin builder does not support control type {ctype!r} "
                f"for '{control_id}'."
            )
        if not controls[-1]["joint_name"]:
            raise ValueError(f"controls catalog '{control_id}' lacks joint_name.")
        controls[-1]["namespace"] = namespace
    if not controls:
        raise ValueError("controls catalog defines no control_ids.")
    return controls


def _control_element(control: dict[str, object]) -> ET.Element:
    namespace = str(control["namespace"])
    cid = str(control["control_id"])
    elem = ET.Element("control")
    ET.SubElement(elem, "control_id").text = cid
    ET.SubElement(elem, "control_type").text = str(control["type"])
    ET.SubElement(elem, "joint_name").text = str(control["joint_name"])
    ET.SubElement(elem, "joint_state_topic").text = f"{namespace}/{cid}/joint_states"
    ET.SubElement(elem, "pressed_topic").text = f"{namespace}/{cid}/pressed"
    ET.SubElement(elem, "state_topic").text = f"{namespace}/{cid}/state"
    ET.SubElement(elem, "state_ids").text = str(control["state_ids"])
    if control["type"] == "button":
        ET.SubElement(elem, "spring_stiffness").text = f"{control['spring_stiffness']:.9g}"
        ET.SubElement(elem, "spring_damping").text = f"{_BUTTON_SPRING_DAMPING:.9g}"
        ET.SubElement(elem, "press_threshold").text = f"{control['press_threshold']:.9g}"
        ET.SubElement(elem, "release_threshold").text = f"{control['release_threshold']:.9g}"
        ET.SubElement(elem, "reset_position").text = "0.0"
        ET.SubElement(elem, "graspable").text = "false"
    else:
        ET.SubElement(elem, "detents").text = str(control["detents"])
        ET.SubElement(elem, "detent_stiffness").text = f"{_KNOB_DETENT_STIFFNESS:.9g}"
        ET.SubElement(elem, "spring_damping").text = f"{_KNOB_SPRING_DAMPING:.9g}"
        ET.SubElement(elem, "grasp_stiffness").text = f"{_KNOB_GRASP_STIFFNESS:.9g}"
        ET.SubElement(elem, "grasp_damping").text = f"{_KNOB_GRASP_DAMPING:.9g}"
        ET.SubElement(elem, "grasp_coupling_stiffness").text = (
            f"{_KNOB_COUPLING_STIFFNESS:.9g}"
        )
        ET.SubElement(elem, "grasp_coupling_damping").text = (
            f"{_KNOB_COUPLING_DAMPING:.9g}"
        )
        ET.SubElement(elem, "grasp_coupling_max_effort").text = (
            f"{_KNOB_COUPLING_MAX_EFFORT:.9g}"
        )
        ET.SubElement(elem, "motion_tolerance").text = f"{_KNOB_MOTION_TOLERANCE:.9g}"
        ET.SubElement(elem, "reset_position").text = "0.0"
        ET.SubElement(elem, "graspable").text = "true"
        ET.SubElement(elem, "grasp_point").text = _KNOB_GRASP_POINT
    return elem


def _plugin_element(
    scene_name: str, namespace: str, controls: list[dict[str, object]]
) -> ET.Element:
    """Emit the ``libxczs_cabinet_state`` plugin block for a fixture scene.

    The plugin name stays unique across every model in the shared Gazebo
    process (``xczs_<scene>_state``), matching the per-cabinet convention.
    """
    plugin = ET.Element(
        "plugin",
        {"name": f"xczs_{scene_name}_state", "filename": "libxczs_cabinet_state.so"},
    )
    ros = ET.SubElement(plugin, "ros")
    ET.SubElement(ros, "namespace").text = namespace
    ET.SubElement(plugin, "state_publish_rate").text = "20.0"
    ET.SubElement(plugin, "reset_service").text = f"{namespace}/reset_physics"
    ET.SubElement(plugin, "grasp_service").text = f"{namespace}/grasp"
    ET.SubElement(plugin, "grasp_active_topic").text = f"{namespace}/grasp_active"
    ET.SubElement(plugin, "active_control_topic").text = f"{namespace}/active_control"
    ET.SubElement(plugin, "operation_heartbeat_topic").text = (
        f"{namespace}/operation_heartbeat"
    )
    ET.SubElement(plugin, "operation_fault_topic").text = f"{namespace}/operation_fault"
    ET.SubElement(plugin, "grasp_distance_threshold").text = "0.12"
    # Above the 1.25 s worst normal heartbeat, below the 3 s lease TTL.
    ET.SubElement(plugin, "operation_watchdog_timeout").text = "2.0"
    for control in controls:
        plugin.append(_control_element(control))
    return plugin


# ---------------------------------------------------------------------------
# SDF emission
# ---------------------------------------------------------------------------

def _pose_text(pose: list[float]) -> str:
    return " ".join(f"{v:.9g}" for v in pose)


def _box_collision(pose: list[float], size: list[float], name: str) -> ET.Element:
    collision = ET.Element("collision", {"name": name})
    pose_elem = ET.SubElement(collision, "pose")
    pose_elem.text = _pose_text(pose)
    geometry = ET.SubElement(collision, "geometry")
    box = ET.SubElement(geometry, "box")
    s = ET.SubElement(box, "size")
    s.text = _pose_text(size)
    return collision


def _link_to_sdf(link: LinkSpec, *, ground_collisions: list[dict[str, object]]) -> ET.Element:
    elem = ET.Element("link", {"name": link.name})
    ET.SubElement(elem, "pose").text = "0 0 0 0 0 0"

    inertial = ET.SubElement(elem, "inertial")
    ET.SubElement(inertial, "pose").text = _pose_text(link.inertial_pose)
    ET.SubElement(inertial, "mass").text = f"{link.mass:.9g}"
    ixx, ixy, ixz, iyy, iyz, izz = link.inertia
    inertia_elem = ET.SubElement(inertial, "inertia")
    for tag, value in (("ixx", ixx), ("ixy", ixy), ("ixz", ixz),
                       ("iyy", iyy), ("iyz", iyz), ("izz", izz)):
        ET.SubElement(inertia_elem, tag).text = f"{value:.9g}"

    if link.visual_uri is not None:
        visual = ET.SubElement(elem, "visual", {"name": "visual"})
        ET.SubElement(visual, "pose").text = "0 0 0 0 0 0"
        geometry = ET.SubElement(visual, "geometry")
        mesh = ET.SubElement(geometry, "mesh")
        ET.SubElement(mesh, "uri").text = link.visual_uri
        material = ET.SubElement(visual, "material")
        ambient = ET.SubElement(material, "ambient")
        r, g, b, a = link.visual_rgba
        ambient.text = f"{r:.6g} {g:.6g} {b:.6g} {a:.6g}"
        ET.SubElement(material, "diffuse").text = f"{r:.6g} {g:.6g} {b:.6g} {a:.6g}"

    if link.is_ground:
        for index, box in enumerate(ground_collisions):
            elem.append(_box_collision(box["pose"], box["size"], f"collision_{index}"))
    elif link.visual_uri is not None:
        collision = ET.SubElement(elem, "collision", {"name": "collision"})
        ET.SubElement(collision, "pose").text = "0 0 0 0 0 0"
        geometry = ET.SubElement(collision, "geometry")
        mesh = ET.SubElement(geometry, "mesh")
        ET.SubElement(mesh, "uri").text = link.visual_uri
    return elem


def _joint_to_sdf(joint: JointSpec) -> ET.Element:
    elem = ET.Element("joint", {"name": joint.name, "type": joint.type})
    ET.SubElement(elem, "pose").text = _pose_text(joint.pose)
    ET.SubElement(elem, "parent").text = joint.parent
    ET.SubElement(elem, "child").text = joint.child
    axis = ET.SubElement(elem, "axis")
    ET.SubElement(axis, "xyz").text = _pose_text(joint.axis)
    limit = ET.SubElement(axis, "limit")
    for tag, value in joint.limits.items():
        ET.SubElement(limit, tag).text = f"{value:.9g}"
    dynamics = ET.SubElement(axis, "dynamics")
    for tag, value in joint.dynamics.items():
        ET.SubElement(dynamics, tag).text = f"{value:.9g}"
    return elem


def build_sdf(
    spec: SceneSpec,
    *,
    ground_collisions: list[dict[str, object]],
    dynamic: bool = False,
    namespace: str | None = None,
    controls: list[dict[str, object]] | None = None,
) -> ET.Element:
    # The scene source is authored as xacro (root <sdf> so Gazebo keeps the full
    # meshes).  Phase A files carry no macros yet -- the namespace is declared so
    # the file is a real xacro and Phase B ``--dynamic`` can parametrize the
    # fixture joints without touching the runtime spawn path.
    sdf = ET.Element(
        "sdf",
        {"version": "1.6", "xmlns:xacro": "http://www.ros.org/wiki/xacro"},
    )
    model = ET.SubElement(sdf, "model", {"name": spec.name})
    ET.SubElement(model, "static").text = "false" if dynamic else "true"
    if dynamic:
        # No empty ``world`` link.  A 0-mass body inside a movable model poisons
        # the ODE solver: the fixed joint binds the ground to the empty link, so
        # the whole assembly either falls or NaNs the model twist (verified live:
        # with-world-link → NaN/falling, without → finite joint velocities).
        # ``parent=world`` on the anchor joint below then binds straight to the
        # implicit SDF world frame -- the same binding a URDF->SDF conversion
        # produces for the shared cabinet's root fixed joint.
        pass
    for link in spec.links:
        model.append(_link_to_sdf(link, ground_collisions=ground_collisions))
    if dynamic:
        anchor = ET.Element(
            "joint",
            {"name": f"anchor_{spec.ground.name}", "type": "fixed"},
        )
        ET.SubElement(anchor, "parent").text = "world"
        ET.SubElement(anchor, "child").text = spec.ground.name
        model.append(anchor)
    for joint in spec.joints:
        model.append(_joint_to_sdf(joint))
    if dynamic:
        # 与共享柜体可操作关节同款物理求解：implicitSpringDamper 让 ODE 用
        # 隐式弹簧-阻尼求解夹具关节（大刚度 spring 更稳），provideFeedback
        # 开启关节力反馈。缺这两项时 Gazebo 对夹具关节报 GetVelocity=NaN
        # （速度状态从不更新），导致插件把 state.velocity/effort 发成 NaN，
        # operator 的 structured-state 门禁判 INVALIDATE → 操作永远等不到
        # 新鲜状态。柜体关节带这两项而实测 velocity/effort 为有限小值。
        for joint in spec.joints:
            if joint.type == "fixed":
                continue
            gazebo_ext = ET.Element("gazebo", {"reference": joint.name})
            ET.SubElement(gazebo_ext, "implicitSpringDamper").text = "true"
            ET.SubElement(gazebo_ext, "provideFeedback").text = "true"
            sdf.append(gazebo_ext)
    if dynamic and controls is not None:
        if not namespace:
            raise ValueError("dynamic build requires a plugin namespace.")
        model.append(_plugin_element(spec.name, namespace, controls))
    return sdf


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _serialize(sdf: ET.Element) -> bytes:
    # Human-readable formatting: one element per line, no namespace prefixes.
    import xml.dom.minidom
    return xml.dom.minidom.parseString(
        ET.tostring(sdf, encoding="unicode")
    ).toprettyxml(indent="  ").encode("utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True, help="SW2URDF export URDF")
    parser.add_argument("--name", required=True, help="scene name (model name)")
    parser.add_argument("--out", required=True, help="output .xacro path")
    parser.add_argument(
        "--mesh-dir",
        help="scene mesh dir in the description package "
             "(default: <repo>/xczs_inspection_robot_description/meshes/scenes/<name>)",
    )
    parser.add_argument(
        "--ground-collision-boxes", action="store_true",
        help="compute AABB box proxies for the ground link via vtk",
    )
    parser.add_argument("--ground-resolution", type=float, default=1.0)
    parser.add_argument("--slab-thickness", type=float, default=0.2)
    parser.add_argument(
        "--dynamic", action="store_true",
        help="emit a movable fixture model: static=false, a world anchor on the "
             "ground link, and the cabinet_state plugin driven by the per-scene "
             "controls catalog",
    )
    parser.add_argument(
        "--namespace",
        help="plugin ROS namespace (default: /xczs/cabinet/<name>)",
    )
    parser.add_argument(
        "--controls-config",
        help="scene_controls YAML consumed by --dynamic "
             "(default: <repo>/xczs_inspection_robot_control/config/"
             "scene_controls/<name>_controls.yaml)",
    )
    args = parser.parse_args()

    spec = parse_urdf(Path(args.urdf), scene_name=args.name)

    ground_collisions: list[dict[str, object]] = []
    if args.ground_collision_boxes:
        mesh_dir = (
            Path(args.mesh_dir)
            if args.mesh_dir
            else Path(__file__).resolve().parent.parent.parent
            / "xczs_inspection_robot_description" / "meshes" / "scenes" / args.name
        )
        ground_collisions = _compute_ground_collision_boxes(
            spec.ground.visual_uri,
            mesh_dir=mesh_dir,
            resolution=args.ground_resolution,
            slab_thickness=args.slab_thickness,
        )
        print(f"ground collision boxes ({len(ground_collisions)}):")
        for box in ground_collisions:
            print(f"  pose={box['pose']} size={box['size']}")

    controls: list[dict[str, object]] | None = None
    namespace = args.namespace or f"/xczs/cabinet/{args.name}"
    if args.dynamic:
        controls_path = (
            Path(args.controls_config)
            if args.controls_config
            else Path(__file__).resolve().parent.parent.parent
            / "xczs_inspection_robot_control"
            / "config" / "scene_controls" / f"{args.name}_controls.yaml"
        )
        controls = _load_controls(controls_path, namespace=namespace)

    sdf = build_sdf(
        spec,
        ground_collisions=ground_collisions,
        dynamic=args.dynamic,
        namespace=namespace,
        controls=controls,
    )
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_bytes(_serialize(sdf))
    print(f"wrote {out_path}")
    print(f"  links={len(spec.links)} joints={len(spec.joints)} "
          f"static={'false' if args.dynamic else 'true'} "
          f"(Phase {'B' if args.dynamic else 'A'})"
          f"{' controls=' + str(len(controls)) if controls is not None else ''}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
