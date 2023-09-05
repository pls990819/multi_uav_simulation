# multi UAV simulation
* Author: Pulishen <2130732@tongji.edu.cn>

## Enviornment
- Ubuntu 18.04
- ROS Melodic

## Install ROS and PX4
- If you need to install ROS and PX4, please visit this website [xtdrone](https://www.yuque.com/xtdrone/manual_cn/basic_config)

## How to Use
- load px4 Gazebo environment
```
roslaunch multi_uav_simulation three_uav.launch
```
- open communication node and making UAV hovering
```
cd scripts
bash multi_vehicle_communication.sh
python multirotor_keyboard_control.py iris 3 vel
```
- obtain the true pose of UAV
```
cd scripts
python get_local_pose.py iris 3
```
- run the formation simulation script
```
cd scripts
bash run_formation.sh 3
```

## Acknowledgements
Thanks for XTDrone(Xiao K, Tan S, Wang G, et al. XTDrone: A customizable multi-rotor UAVs simulation
platform[C]//2020 4th International Conference on Robotics and Automation Sciences (ICRAS).
IEEE, 2020: 55-61), the simulator of my experiment is developed based on this platform.
