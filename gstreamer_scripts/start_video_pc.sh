#!/bin/bash

gst-launch-1.0 -v udpsrc port=5600 ! 'application/x-rtp, encoding-name=H264, payload=96' ! rtpjitterbuffer ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! xvimagesink sync=false
