Steps:

1. sudo -H pip install -r requirements.txt
2. do caktin build in your workspace
3. roslaunch circle_tracker tracker_2d.launch
4. play your rosbag

Note:

The Kalman Filter implementation is still buggy, and text marker display isn't
working on RViz, so as a proxy, the velocity and object positions are being
printed on terminal, and the cylinder indicating objects turns from dark red to
green as velocity scales.

Author: Parth Chopra
Email : parthc@umich.edu
Date : 2021 - 02 - 04
