#!/bin/bash

#echo "doker container name ?"
#read containername

#echo "launch file name ?"
#read launchname

#sudo docker start "$containername"

#sudo docker exec -it "$containername" /bin/bash -c "catkin_make && catkin_make install"

# sudo docker exec -it "$containername" /bin/bash -c "source devel/setup.sh && roslaunch "$containername" "$launchname".launch"


sudo docker start debugcont

sudo docker exec -it debugcont /bin/bash -c "source devel/setup.sh && roslaunch arc2020 main.launch"