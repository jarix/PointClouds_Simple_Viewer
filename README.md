# PointClouds_Simple_Viewer

Load a point cloud file in PCD format and display it with
pcl::visualization::PCLVisualizer viewer class

PCL Documentation:
https://pointclouds.org/

PCD File format:
http://pointclouds.org/documentation/tutorials/pcd_file_format.html

## Usage

`simple_pcd_viewer <input_PCD_filename>`

E.g. `simple_pcd_viewer ../sample_point_clouds/wolf.pcd`

## PCL Visualizer Key and Mouse Mappings

- q: Exit
- +: Increase point size
- -: Decrease point size
- u: Display color lookup table (toggle on/off)
- c: Display current camera parameters (in console)
- g: Display scale grid (toggle on/off)
- Alt f: Maximize window (toggle on/off)
- Alt +: Zoom In
- Alt -: Zoom Out
- Left mouse button down: Rotate
- Middle mouse button down: Pan
- Right mouse button down: Zoom
