#!/bin/bash
rosrun=false
res0=$(gpio -1 read 40)
if [ ${res0} = 0 ]; then
    exit 1
fi
res=1
while [ ${res0} = ${res} ]
do
    sleep 2s
    res=$(gpio -1 read 40)
    #echo ${res}
done
echo "shutdown now" | wall
#shutdown -h now
