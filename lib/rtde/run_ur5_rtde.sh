#!/bin/bash

function close_running(){
    echo "Cleaning up"
    A=$(ps ax | grep -m1 'python2 ur5_rtde.py' | awk '{print $1}')
    kill $A
}

close_running

cd ~/rmstudio/lib/rtde
python2 ur5_rtde.py &
echo "Interfaces running"
