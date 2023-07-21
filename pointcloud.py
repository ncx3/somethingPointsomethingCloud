import numpy as np
import open3d as o3d
from PIL import Image

# Load color image
color = Image.open("image_path.jpg")
color = np.asarray(color)

# Load depth map
depth = Image.open("DepthImage_path.jpg").convert("L")
depth = np.asarray(depth)

# Normalize the depth for visualization
depth = depth / np.max(depth)

# Generate x and y coordinates
y_coords, x_coords = np.indices(depth.shape)

# Stack x, y coordinates and depth map to create 3D point cloud
cloud = np.stack((x_coords, y_coords, depth * 255), axis=-1).reshape(-1, 3)

# Create Open3D PointCloud object and assign the 3D points and colors
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(cloud)
pcd.colors = o3d.utility.Vector3dVector(color.reshape(-1, 3) / 255)

# Save point cloud as .ply
o3d.io.write_point_cloud("pointcloud_path.ply", pcd)
