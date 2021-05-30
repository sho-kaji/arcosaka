# image
FROM ros:melodic

WORKDIR /root/catkin_ws 
# install packages
RUN set -x &&\
    apt-get update &&\
    apt-get install \
        sqlite3 libsqlite3-dev -y &&\
    apt-get install \
        ros-"$ROS_DISTRO"-roswww libffi-dev python-pip \
        libjpeg-dev zlib1g-dev -y &&\
    apt-get install \
        python-smbus -y
RUN set -x &&\
    pip install twisted tornado pigpio pyopenssl autobahn image \ 
        bson pymongo service_identity
RUN set -x catkin_make
RUN set -x catkin_make install
ENV PYTHONPATH=${PYTHONPATH}:/root/catkin_ws/src/arc2020/scripts
