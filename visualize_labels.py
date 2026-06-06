#!/usr/bin/env python3
"""
Visualize a PCD file + BEV label JSON (from bev_labeler) using Open3D.

Usage:
    python visualize_labels.py cloud.pcd labels.json
    python visualize_labels.py cloud.pcd              # just the cloud
"""

import sys
import json
import numpy as np
import open3d as o3d

# ---------- colour palette (matches the HTML labeler) ----------
CLASS_COLORS = {
    "car":         [0.94, 0.27, 0.27],
    "pedestrian":  [0.13, 0.83, 0.91],
    "cyclist":     [0.65, 0.54, 0.98],
    "road":        [0.30, 0.86, 0.50],
    "building":    [0.98, 0.57, 0.19],
    "custom":      [0.98, 0.75, 0.14],
    "marker":         [0.0, 1.0, 0.0], # show the lane marker
#    "marker":         [1.0, 0.753, 0.796], # marker in pink
}
DEFAULT_COLOR = [0.98, 0.75, 0.14]

POLYLINE_Z   = 1.2#0.2   # height at which flat polylines are drawn
TUBE_RADIUS  = 0.2#0.05  # sphere/cylinder radius for polyline nodes & edges
POINT_SIZE   = 2.0


def hex_to_rgb(h):
    h = h.lstrip("#")
    return [int(h[i:i+2], 16) / 255 for i in (0, 2, 4)]


def class_color(cls: str):
    return CLASS_COLORS.get(cls.lower(), DEFAULT_COLOR)


# ---------- geometry helpers ----------

def make_sphere(center, radius, color):
    s = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=8)
    s.translate(center)
    s.paint_uniform_color(color)
    s.compute_vertex_normals()
    return s


def make_cylinder(p0, p1, radius, color):
    p0, p1 = np.array(p0, dtype=float), np.array(p1, dtype=float)
    vec = p1 - p0
    length = np.linalg.norm(vec)
    if length < 1e-6:
        return None
    cyl = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=length, resolution=8, split=1)
    # default cylinder is along Z; rotate to align with vec
    z = np.array([0, 0, 1], dtype=float)
    axis = np.cross(z, vec / length)
    axis_len = np.linalg.norm(axis)
    if axis_len > 1e-6:
        axis /= axis_len
        angle = np.arccos(np.clip(np.dot(z, vec / length), -1, 1))
        R = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)
        cyl.rotate(R, center=[0, 0, 0])
    elif np.dot(z, vec / length) < 0:
        cyl.rotate(o3d.geometry.get_rotation_matrix_from_axis_angle(
            np.array([1, 0, 0]) * np.pi), center=[0, 0, 0])
    cyl.translate((p0 + p1) / 2)
    cyl.paint_uniform_color(color)
    cyl.compute_vertex_normals()
    return cyl


def polyline_to_geometries(points_2d, closed: bool, cls: str, z: float):
    color = class_color(cls)
    geoms = []
    pts3d = [[p["x"], p["y"], z] for p in points_2d]
    if not pts3d:
        return geoms
    # nodes
#    for pt in pts3d:
#        geoms.append(make_sphere(pt, TUBE_RADIUS * 1.5, color))
    # edges
    edges = list(zip(pts3d, pts3d[1:]))
    if closed and len(pts3d) > 2:
        edges.append((pts3d[-1], pts3d[0]))
    for a, b in edges:
        c = make_cylinder(a, b, TUBE_RADIUS, color)
        if c:
            geoms.append(c)
    return geoms


def build_legend_text(annotations):
    counts = {}
    for a in annotations:
        counts[a["class"]] = counts.get(a["class"], 0) + 1
    lines = ["Classes:"] + [f"  {cls}: {n}" for cls, n in sorted(counts.items())]
    return "\n".join(lines)


# ---------- intensity colormap ----------
import matplotlib.cm as _cm
_turbo = _cm.get_cmap("turbo")

def turbo_color(t: float):
    """Return [r,g,b] in [0,1] using matplotlib's standard turbo colormap."""
    r, g, b, _ = _turbo(float(np.clip(t, 0, 1)))
    return [r, g, b]


def colorize_by_intensity(pcd):
    pts = np.asarray(pcd.points)
    if not pcd.has_colors():
        # try intensity from normals channel (some PCDs store it there) – fall back to Z
        pass

    # detect intensity: Open3D exposes it via point cloud if loaded from PCD with 'intensity' field
    intensity = None

    # open3d >= 0.16 stores extra fields in pcd.point (tensor API) or legacy via normals hack
    # try tensor pointcloud first
    try:
        import open3d.t.io as tio
        tpcd = tio.read_point_cloud(pcd._path)  # won't work, no _path attr
    except Exception:
        pass

    # reliable fallback: check normals — velodyne PCD loaders sometimes stuff intensity there
    if pcd.has_normals():
        normals = np.asarray(pcd.normals)
        candidate = normals[:, 0]  # x of normal is often intensity
        if candidate.min() >= 0 and candidate.max() > 1:
            intensity = candidate

    if intensity is None:
        # colour by Z height instead
        z = pts[:, 2]
        mn, mx = np.percentile(z, 1), np.percentile(z, 99)
        intensity = np.clip((z - mn) / max(mx - mn, 1e-6), 0, 1)
        print("  [info] no intensity field found — colouring by Z height")
    else:
        mn, mx = np.percentile(intensity, 1), np.percentile(intensity, 99)
        intensity = np.clip((intensity - mn) / max(mx - mn, 1e-6), 0, 1)
        print(f"  [info] intensity range after clipping: [{mn:.3f}, {mx:.3f}]")

    colors = np.array(_turbo(intensity))[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def load_pcd_with_intensity(path: str):
    """Load PCD and try to read intensity via the tensor API (o3d >= 0.16)."""
    pcd = o3d.io.read_point_cloud(path)
    pts = np.asarray(pcd.points)
    print(f"  Loaded {len(pts):,} points from {path}")

    intensity = None
    try:
        # Tensor API preserves all PCD fields
        tpcd = o3d.t.io.read_point_cloud(path)
        if "intensity" in tpcd.point:
            raw = tpcd.point["intensity"].numpy().flatten().astype(float)
            mn, mx = np.percentile(raw, 1), np.percentile(raw, 99)
            intensity = np.clip((raw - mn) / max(mx - mn, 1e-6), 0, 1)
            print(f"  [info] intensity via tensor API — raw range [{raw.min():.2f}, {raw.max():.2f}]")
    except Exception as e:
        print(f"  [info] tensor API unavailable ({e}), trying fallback")

    if intensity is None:
        # Fallback: Z-height colouring
        z = pts[:, 2]
        mn, mx = np.percentile(z, 1), np.percentile(z, 99)
        intensity = np.clip((z - mn) / max(mx - mn, 1e-6), 0, 1)
        print("  [info] colouring by Z height (no intensity field)")

    colors = np.array(_turbo(intensity))[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


# ---------- main ----------

def main():
#    if len(sys.argv) < 2:
#        print(__doc__)
#        sys.exit(1)

#    pcd_path  = sys.argv[1]
#    json_path = sys.argv[2] if len(sys.argv) >= 3 else None
    pcd_path  = r"d:\out_ABH_scene_fr.301.pcd"
    json_path = r"C:\GIT\BEV_labeler\labels.json"
    
    print(f"Loading PCD: {pcd_path}")
    pcd = load_pcd_with_intensity(pcd_path)

    geometries = [pcd]

    if json_path:
        print(f"Loading labels: {json_path}")
        with open(json_path) as f:
            data = json.load(f)

        annotations = data.get("annotations", [])
        print(f"  {len(annotations)} annotation(s)")

        # find a sensible Z for drawing polylines (just above ground)
        pts = np.asarray(pcd.points)
        z_draw = float(np.percentile(pts[:, 2], 10)) + POLYLINE_Z

        for ann in annotations:
            geoms = polyline_to_geometries(
                ann["points"], ann.get("closed", False), ann["class"], z_draw
            )
            geometries.extend(geoms)

        print(build_legend_text(annotations))
    else:
        print("No label file provided — showing point cloud only.")

    # coordinate frame at origin
    geometries.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0))

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="BEV Label Viewer", width=1280, height=800)
    for g in geometries:
        vis.add_geometry(g)

    # top-down view
    vc = vis.get_view_control()
    vc.set_zoom(0.5)
    vc.set_front([0, 0, 1])
    vc.set_up([0, 1, 0])
    vc.set_lookat(np.asarray(pcd.points).mean(axis=0).tolist())

    opt = vis.get_render_option()
    opt.background_color = np.array([0.07, 0.07, 0.07])
    opt.point_size = POINT_SIZE

    print("\nControls: left-drag rotate | right-drag pan | scroll zoom | Q/Esc quit")
    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
