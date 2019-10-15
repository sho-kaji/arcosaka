path=~/tool/mjpg-streamer/mjpg-streamer-experimental/
mjpg_streamer -i "$path/input_uvc.so -f 10 -r 640x360 -d /dev/video0 -y" -o "$path/output_http.so -w ./www -p 8494"
mjpg_streamer -i "$path/input_uvc.so -f 10 -r 640x360 -d /dev/video2 -y" -o "$path/output_http.so -w ./www -p 8495"
