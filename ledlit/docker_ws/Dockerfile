# image
FROM ros:latest

WORKDIR /root/catkin_ws 
# install packages
RUN set -x &&\
    apt-get update &&\
    apt-get install \
        ros-"$ROS_DISTRO"-roswww  libffi-dev python-pip \
        libjpeg-dev zlib1g-dev -y
RUN set -x &&\
    pip install twisted tornado pigpio pyopenssl autobahn image \ 
        bson pymongo service_identity
RUN ["/bin/bash","-c","source /ros_entrypoint.sh"]
