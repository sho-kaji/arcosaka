#!/bin/bash
path=/usr/local/lib/mjpg-streamer/
mjpg_streamer -i "$path/input_uvc.so -f 10 -r 640x360 -d /dev/video0 -y" -o "$path/output_http.so -w /usr/local/share/mjpg-streamer/www -p 8494"
mjpg_streamer -i "$path/input_uvc.so -f 10 -r 640x360 -d /dev/video1 -y" -o "$path/output_http.so -w /usr/local/share/mjpg-streamer/www -p 8495"
