import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    return inlier_cloud, outlier_cloud

params = o3d.camera.PinholeCameraIntrinsic()
params.intrinsic_matrix =  np.array([[572.4114,       0.0, 325.2611],
[     0.0, 573.57043, 242.04899],
[     0.0,       0.0,      1.0]])



color_raw = o3d.io.read_image("rgb.png")
depth_raw = o3d.io.read_image("depth.png")
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, depth_raw, convert_rgb_to_intensity=False);
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, params)
#o3d.visualization.draw_geometries([pcd])

plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=500,
                                         num_iterations=500)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

#inlier_cloud, outlier_cloud = display_inlier_outlier(pcd, inliers)
#
# cl, ind = inlier_cloud.remove_statistical_outlier(nb_neighbors=50,
#                                                     std_ratio=2.0)
# inlier_cloud, outlier_cloud = display_inlier_outlier(inlier_cloud, ind)

#o3d.visualization.draw_geometries([inlier_cloud])

new_cloud = o3d.geometry.PointCloud()

def is_above_line(point, plane_model):
    x,y,z = point
    [a, b, c, d] = plane_model
    return (a*x + b*y + c*z + d) < 0.003

for pnt,color in zip(pcd.points,pcd.colors):
    if is_above_line(pnt,plane_model):
        new_cloud.points.append(pnt)
        new_cloud.colors.append(color)

#o3d.visualization.draw_geometries([new_cloud])

with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(
        new_cloud.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
new_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([new_cloud])
