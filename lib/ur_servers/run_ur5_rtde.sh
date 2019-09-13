#!/bin/bash

function close_running(){
    echo "Cleaning up"
#     A=$(ps ax | grep -m1 'python2 ur5_rtde.py' | awk '{print $1}')
    A=$(ps ax | grep 'python2 ur5_rtde.py' | awk '{print $1}')
    for value in $A
    do
        echo "Killing process: $value"
        kill $value
    done
    echo "Killed!"
}

close_running

cd ~/rmstudio/lib/ur_servers
python2 ur5_rtde.py &
echo "RTDE Server Running"

