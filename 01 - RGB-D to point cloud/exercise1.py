import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# load and visualize the RGB-D data
color_raw = o3d.io.read_image("rgb.png")
depth_raw = o3d.io.read_image("depth.png")
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, depth_raw, convert_rgb_to_intensity=False)
print(rgbd_image)

plt.subplot(1, 2, 1)
plt.title('RGB image')
plt.imshow(rgbd_image.color)
plt.subplot(1, 2, 2)
plt.title('Depth image')
plt.imshow(rgbd_image.depth)
plt.show()

# define a camera model
intrinsic_params = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
intrinsic_params.intrinsic_matrix = [[572.4114, 0.0, 325.2611], [ 0.0, 573.57043, 242.04899], [ 0.0, 0.0, 1.0]]

# generate a point cloud based on the camera model
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic_params)

# visualize and save the point coud
o3d.visualization.draw_geometries([pcd])
o3d.io.write_point_cloud("exercise1.pcd", pcd, write_ascii=True, print_progress=True)

# function that outputs a 3D coordinate given a depth value at a certain pixel location
def pixel_to_point(depth_map, pixel_loc, K):
    u,v = pixel_loc
    depth = depth_map[v,u]

    if depth > 0:
        fx_ = (1/K[0][0]) # inverse focal length fx
        fy_ = (1/K[1][1]) # inverse focal length fy
        u_0 = K[0][2] # optical center
        v_0 = K[1][2]


        x = depth*(u-u_0)*fx_
        y = depth*(v-v_0)*fy_
        z = depth

        return np.array([x,y,z])/1000
    else:
        print("Invalid depth data for this point")


# define two corners in pixel coordinates, and deproject them to 3D points
corner1 = (285,27)
corner2 = (322,78)
corner1_3d = pixel_to_point(depth_map=np.asarray(depth_raw), pixel_loc=corner1, K=intrinsic_params.intrinsic_matrix)
corner2_3d = pixel_to_point(depth_map=np.asarray(depth_raw), pixel_loc=corner2, K=intrinsic_params.intrinsic_matrix)
print(corner1_3d, corner2_3d)
# define the depth range of the box
corner1_3d[2] = 0
corner2_3d[2] = 2

corner_ball = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)

# define a box based on the two corners
box = o3d.geometry.AxisAlignedBoundingBox(min_bound=corner1_3d, max_bound=corner2_3d)
box.color = (1,0,0) # make it red

# crop the point cloud based on the bounding box
pcd_cropped = pcd.crop(bounding_box = box)

# create a tight bounding box around the cropped point cloud
object_box = pcd_cropped.get_axis_aligned_bounding_box()
object_box.color = (0,1,0) # make it green

# visualize everything
coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
o3d.visualization.draw_geometries([pcd, box, object_box, coordinate_frame])

o3d.visualization.draw_geometries([pcd_cropped, object_box])
