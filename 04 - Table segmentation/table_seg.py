import open3d as o3d

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    return inlier_cloud, outlier_cloud

def is_above_line(point, plane_model):
    x,y,z = point
    [a, b, c, d] = plane_model
    return (a*x + b*y + c*z + d) > 0.001

pcd = o3d.io.read_point_cloud("table_scene_lms400.pcd")
o3d.visualization.draw_geometries([pcd])

plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=100,
                                         num_iterations=100)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

inlier_cloud, outlier_cloud = display_inlier_outlier(pcd, inliers)

plane_model, inliers = outlier_cloud.segment_plane(distance_threshold=0.01,
                                         ransac_n=100,
                                         num_iterations=100)

display_inlier_outlier(outlier_cloud, inliers)

print("Statistical oulier removal")
cl, ind = outlier_cloud.remove_statistical_outlier(nb_neighbors=50,
                                                    std_ratio=2.0)
inlier_cloud, outlier_cloud = display_inlier_outlier(outlier_cloud, ind)

o3d.visualization.draw_geometries([inlier_cloud])

new_cloud = o3d.geometry.PointCloud()



for pnt in inlier_cloud.points:
    if is_above_line(pnt,plane_model):
        new_cloud.points.append(pnt)

o3d.visualization.draw_geometries([new_cloud])
