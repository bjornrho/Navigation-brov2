#!/bin/bash
export LD_LIBRARY_PATH=/usr/local/lib/

WIDTH=1920
HEIGHT=1080
FRAMERATE=30
DEVICE=/dev/video2

gst-launch-1.0 -v v4l2src device=$DEVICE do-timestamp=true ! video/x-h264,width=$WIDTH,height=$HEIGHT, framerate=$FRAMERATE/1 ! h264parse ! queue ! rtph264pay config-interval=10 pt=96 ! multiudpsink clients=192.168.2.1:5600,localhost:2200
