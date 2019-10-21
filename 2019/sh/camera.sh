#!/bin/bash
cd /dev/
vdo=$(ls video*)
path=/usr/local/lib/mjpg-streamer/

for i in ${vdo[@]};do
    port=${i#video}
    port=$((10#$port + 8494))
    mjpg_streamer -i "$path/input_uvc.so -f 10 -r 640x360 -d /dev/$i -y" -o "$path/output_http.so -w /usr/local/share/mjpg-streamer/www -p $port" &
done