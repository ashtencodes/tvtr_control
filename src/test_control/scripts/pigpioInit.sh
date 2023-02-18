#!/bin/bash
pigpioDaemonId=$(cat /var/run/pigpio.pid)

re='^[0-9]+$'
if  [[ $pigpioDaemonId =~ $re ]] ; then
    sudo kill -9 $pigpioDaemonId
fi