#!/bin/bash
rosrun=false
res0=$(gpio -1 read 40)
res=1
while [ ${res} = ${res0} ]
do
    sleep 2s
    res=$(gpio -1 read 40)
    #echo ${res}
done
wall < echo "shutdown now"
#shutdown -h now
