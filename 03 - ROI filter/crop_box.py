import numpy as np
import pandas as pd
import argparse
from pyntcloud import PyntCloud
import os

crop_size = 0.5 # in meters
p0 = [-0.1, -0.15, 1.0]

if __name__ == "__main__":
    """
    Main function for executing the .py script.
    Command:
        -p <filename>.ply
    """
    # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--path", type=str, default="cloud.ply",
                    help="path to point cloud")
    args = vars(ap.parse_args())

    cloud = PyntCloud.from_file(args["path"])
    print(cloud.points.describe())

    xmin, xmax = p0[0] - crop_size/2, p0[0] + crop_size/2
    ymin, ymax = p0[1] - crop_size/2, p0[1] + crop_size/2
    zmin, zmax = p0[2] - crop_size/2, p0[2] + crop_size/2

    inliers_x = np.array(cloud.points["x"] > xmin) & np.array(cloud.points["x"] < xmax)
    inliers_y = np.array(cloud.points["y"] > ymin) & np.array(cloud.points["y"] < ymax)
    inliers_z = np.array(cloud.points["z"] > zmin) & np.array(cloud.points["z"] < zmax)

    inliers = inliers_x & inliers_y
    inliers = inliers_z & inliers
    cloud.points = cloud.points[inliers]
    cloud.to_file("crop.ply",as_text=True)
