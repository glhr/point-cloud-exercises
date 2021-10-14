# Working-with-point-clouds

Either PCL or Open3D can be used for most exercises. Try to get experience using both.

Exercises
* 01 - RGB-D to point cloud
* 02 - Visualizing Point Clouds
* 03 - ROI filter
* 04 - Table segmentation
* 05 - PointNet implementation

## Set-up

### Python packages

Install the required packages in a virtual environment:
```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### Visualization
For visualization, install [MeshLab](https://www.meshlab.net/#download) or [CloudCompare](https://www.danielgm.net/cc/).

### PCL

For Linux:
```
sudo apt install libpcl-dev
```
