#!/bin/bash

for prgname in test_webrtc test_opencv
do
    echo "[${prgname}]を設定開始" &&
    mkdir -p ~/arcosaka/2019/${prgname}/src &&
    cd ~/arcosaka/2019/${prgname}/src &&
    catkin_init_workspace &&
    cd ~/arcosaka/2019/${prgname} &&
    catkin_make
done &&

echo "OK!"