import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# load the point cloud from exercise 1
pcd = o3d.io.read_point_cloud("exercise1.ply")

# plane fitting with RANSAC
plane_model, inliers_indices = pcd.segment_plane(
                                            distance_threshold=0.01,    # play around with these parameters
                                            ransac_n=5000,
                                            num_iterations=500)

# visualize the RANSAC outliers as red
inlier_cloud = pcd.select_by_index(inliers_indices)
outlier_cloud = pcd.select_by_index(inliers_indices, invert=True)
outlier_cloud.paint_uniform_color([1,0,0])
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

# function that checks if a point lies on one side of the plane
def is_above_line(point, plane_model, threshold = 0.003):
    x,y,z = point
    [a,b,c,d] = plane_model
    return (a*x+b*y+c*z+d) < threshold  # returns True or False

# make a new point cloud that only contains points above the plane
new_cloud = o3d.geometry.PointCloud()

for point, color in zip(pcd.points, pcd.colors):
    if is_above_line(point, plane_model):
        new_cloud.points.append(point)
        new_cloud.colors.append(color)

o3d.visualization.draw_geometries([new_cloud])
