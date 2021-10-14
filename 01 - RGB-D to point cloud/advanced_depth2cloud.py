import argparse
import numpy as np
import cv2
import open3d as o3d


class CameraIntrinsics(object):
    """A set of intrinsic parameters for a camera. This class is used to project
    and deproject points.
    """

    def __init__(self):
        """Initialize a CameraIntrinsics model.
        Parameters
        ----------
        """
        self._K = np.array([[572.4114,       0.0, 325.2611],
                            [     0.0, 573.57043, 242.04899],
                            [     0.0,       0.0,      1.0]])

    def deproject_pixel(self, depth, pixel):
        """Deprojects a single pixel with a given depth into a 3D point.
        Parameters
        ----------
        depth : float
            The depth value at the given pixel location.
        pixel : :obj:`autolab_core.Point`
            A 2D point representing the pixel's location in the camera image.
        Returns
        -------

        """
        point_3d = depth * np.linalg.inv(self._K).dot(np.r_[pixel, 1.0])
        return point_3d

    def deproject(self, depth_image):
        """Deprojects a DepthImage into a PointCloud.
        Parameters
        ----------
        """
        # create homogeneous pixels
        row_indices = np.arange(depth_image.shape[0])
        col_indices = np.arange(depth_image.shape[1])
        pixel_grid = np.meshgrid(col_indices, row_indices)
        pixels = np.c_[pixel_grid[0].flatten(), pixel_grid[1].flatten()].T
        pixels_homog = np.r_[pixels, np.ones([1, pixels.shape[1]])]
        depth_arr = np.tile(depth_image.flatten(), [3,1])

        # deproject
        points_3d = depth_arr * np.linalg.inv(self._K).dot(pixels_homog)
        points_3d = np.swapaxes(points_3d,0,1)
        return points_3d


if __name__ == '__main__':
    """
    Main function for executing the .py script.
    """
    camera_intr = CameraIntrinsics()

    rgb = (cv2.cvtColor(cv2.imread("rgb.png"), cv2.COLOR_BGR2RGB)/255.0).astype('float32')
    depth = cv2.imread("depth.png",-1)/1000.0 # convert to meters

    points = camera_intr.deproject(depth)
    color_points = np.concatenate((points,rgb.reshape(307200,3)),axis=1)

    # compute normals
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(color_points[:,:3])
    cloud.colors = o3d.utility.Vector3dVector(color_points[:,3:])
    o3d.io.write_point_cloud("advanced_cloud.ply", cloud)

    print("Saved point cloud as advanced_cloud.ply")
