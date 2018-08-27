#!/usr/bin/env python3

# Starter for The Imaging Source cameras in ros.
# Needs Python 3
# Please refer to https://github.com/TheImagingSource/tiscamera
# Goto: http://wiki.ros.org/gscam for info.
# Install sudo apt-get install ros-kinetic-gscam

import tiscamera
import sys

# Open the camera. Parameters are serial number, width, height, frame rate, color and liveview.
# cam = tiscamera.Camera("27614314", 1280, 1024, 30, True, False)
cam = tiscamera.Camera(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), sys.argv[5], sys.argv[6])

# Start the live stream from the camera and also "rosrun"
cam.start_pipeline()

# Set some properties
cam.set_property("Exposure Auto", True)
cam.set_property("Gain Auto", True)
cam.set_property("Brightness Reference", 128)


input("Press Enter to end program\n")

# Stop the camera pipeline.
cam.stop_pipeline()

print('Program ended\n')
