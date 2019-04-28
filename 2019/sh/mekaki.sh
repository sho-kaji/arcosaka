prgname="mekaki"
if ! pgrep roslaunch > /dev/null; then
    source /opt/ros/melodic/setup.bash
    source ~/arcosaka/2019/${prgname}/devel/setup.bash
    export ROS_HOSTNAME=osakarp.local
    export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311
    alias cw='cd ~/arcosaka/2019/${prgname}'
    alias cs='cd ~/arcosaka/2019/${prgname}/src'
    alias cm='cd ~/arcosaka/2019/${prgname} && catkin_make'

    /opt/ros/melodic/bin/roslaunch ${prgname} ${prgname}.launch
    
else
    echo "roslaunch killing"
    killall -w roslaunch
fi
