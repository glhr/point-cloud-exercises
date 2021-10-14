# 01 - RGB-D to point cloud

This functionality is typically built into a 3D camera's SDK, but is still necessary once in an while.

1. Take a look at the two scripts for deprojecting depth maps to point clouds. Do they produce the same result? Use MeshLab or CloudCompare to visualize the point clouds.
2. Make a function that returns the 3D coordinate of a given pixel (u,v)
3. Make a function that returns the best matching pixel for a given 3D coordinate
