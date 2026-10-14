# Prototype

These are the Python scripts I started from: `fast_slam_1.py` (FastSLAM 1.0), then `fast_slam_2.py` (FastSLAM 2.0), each with its grid module.
They replay a recorded bag offline and plot the result with matplotlib.
I keep them for reference and do not maintain them.

They need numpy, numba, matplotlib, scikit-image and the ROS 2 Humble Python packages for reading a bag (`rosbag2_py`, `rclpy`, `nav_msgs`, `sensor_msgs`).
No bag is shipped. I run them from this directory (`python3 fast_slam_2.py`), and they read the bag at `./my_slam_data`, relative to the working directory. The path is set in `BAG_PATH` at the top of each script and there is no command line argument for it.

`fast_slam_2.py` has two defects that I know of, both fixed in the C++ package.
The coarse search never replaces its start pose, because the best score starts at +inf and a candidate is accepted only when its score is greater (`>`).
The matcher returns its start pose instead of the optimised one.
