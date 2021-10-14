#!/usr/bin/env python
import numpy as np
import cv2
import os
import open3d as o3d

# can be improved. How?
def create_point_cloud(depth,rgb):
    height, width = depth.shape
    v_0, u_0 = height/2, width/2
    fx_ = (1/572.4114) # inverse focal length fx
    fy_ = (1/573.57043) # inverse focal length fy
    points = []
    for v in range(height):
        for u in range(width):
            # 3D
            d = depth[v,u]
            z = d
            x = d*(u-u_0)*fx_
            y = d*(v-v_0)*fy_
            # color
            r = rgb[v,u,0]
            g = rgb[v,u,1]
            b = rgb[v,u,2]
            points.append([x,y,z,r,g,b])

    return np.array(points)


if __name__ == '__main__':
    rgb = cv2.cvtColor(cv2.imread("rgb.png"), cv2.COLOR_BGR2RGB)/255.0
    depth = cv2.imread("depth.png",-1)/1000.0 # convert to meters

    color_points = create_point_cloud(depth,rgb)

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(color_points[:,:3])
    cloud.colors = o3d.utility.Vector3dVector(color_points[:,3:])
    o3d.io.write_point_cloud("simple_cloud.ply", cloud, write_ascii=True)

    print("Saved point cloud as simple_cloud.ply")
