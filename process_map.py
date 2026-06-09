import open3d as o3d
import numpy as np
import yaml
from datetime import datetime

input_file = "test.pcd"

output_pcd = "map_field_voxel_005.pcd"
output_ply = "map_field_voxel_005.ply"
info_file = "map_info.yaml"

# 按你的实际比赛场地范围修改
crop_box = {
    "xmin": -2.0,
    "xmax": 8.0,
    "ymin": -4.0,
    "ymax": 4.0,
    "zmin": -0.5,
    "zmax": 2.5,
}

voxel_size = 0.05

enable_outlier_removal = True
nb_neighbors = 20
std_ratio = 2.0

print("Loading:", input_file)
pcd = o3d.io.read_point_cloud(input_file)

if len(pcd.points) == 0:
    raise RuntimeError("Loaded 0 points. Please check input PCD file.")

pts = np.asarray(pcd.points)

print("Raw points:", len(pts))
print("Raw bounds:")
print("  x:", float(np.min(pts[:, 0])), float(np.max(pts[:, 0])))
print("  y:", float(np.min(pts[:, 1])), float(np.max(pts[:, 1])))
print("  z:", float(np.min(pts[:, 2])), float(np.max(pts[:, 2])))

mask = (
    (pts[:, 0] >= crop_box["xmin"]) & (pts[:, 0] <= crop_box["xmax"]) &
    (pts[:, 1] >= crop_box["ymin"]) & (pts[:, 1] <= crop_box["ymax"]) &
    (pts[:, 2] >= crop_box["zmin"]) & (pts[:, 2] <= crop_box["zmax"])
)

cropped = pcd.select_by_index(np.where(mask)[0])
print("Cropped points:", len(cropped.points))

if len(cropped.points) == 0:
    raise RuntimeError("Cropped 0 points. Please enlarge crop_box.")

filtered = cropped

if enable_outlier_removal:
    print("Removing statistical outliers...")
    filtered, ind = cropped.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio
    )
    print("After outlier removal:", len(filtered.points))

print("Voxel downsample:", voxel_size)
down = filtered.voxel_down_sample(voxel_size=voxel_size)
print("Downsampled points:", len(down.points))

o3d.io.write_point_cloud(output_pcd, down, write_ascii=False)
o3d.io.write_point_cloud(output_ply, down, write_ascii=False)

print("Saved:", output_pcd)
print("Saved:", output_ply)

info = {
    "map": {
        "name": "robocon_field",
        "date": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "raw_file": input_file,
        "processed_pcd": output_pcd,
        "processed_ply": output_ply,
    },
    "coordinate": {
        "frame_id": "map",
        "origin_definition": "robot official start pose, front direction as +X",
        "start_pose": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
    },
    "processing": {
        "crop_box": crop_box,
        "voxel_size": voxel_size,
        "enable_outlier_removal": enable_outlier_removal,
        "nb_neighbors": nb_neighbors,
        "std_ratio": std_ratio,
        "raw_points": int(len(pcd.points)),
        "cropped_points": int(len(cropped.points)),
        "filtered_points": int(len(filtered.points)),
        "downsampled_points": int(len(down.points)),
    },
    "quality_check": {
        "fence_clear_single_line": None,
        "no_obvious_double_wall": None,
        "grid_position_stable": None,
        "ground_noise_acceptable": None,
        "loop_closure_error_acceptable": None,
        "long_edges_straight": None,
    },
    "notes": [
        "LiDAR should be mounted on robot during mapping.",
        "Do not change LiDAR/IMU extrinsics after mapping.",
        "Dynamic objects should be removed from field.",
        "Waypoints must be calibrated on this processed map.",
    ],
}

with open(info_file, "w", encoding="utf-8") as f:
    yaml.dump(info, f, allow_unicode=True, sort_keys=False)

print("Saved:", info_file)
