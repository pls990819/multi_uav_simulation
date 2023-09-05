#!/bin/bash
uav_num=1
while(( $uav_num<= $1 )) 
do
    python controller.py $uav_num&
    let "uav_num++"
done
