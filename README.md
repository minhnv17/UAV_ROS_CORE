# UAV_ROS_CORE
This is ros package for PX4 autoplot with remote control, offboard mode, Slam autonomous...

## Installation

```bash
git clone https://github.com/minhnv17/UAV_ROS_CORE.git
cd UAV_ROS_CORE
catkin_make
source devel/setup.bash
```
## Usage

```bash
roslaunch uavlab411 uavlink.launch
```
## Navigate
Example navigate in uavlab411/Example.
```bash
python3 navigate.py
```

## Simulation
We use native simulation from [Clover](https://clover.coex.tech/en/).

Simulation setup can be found [here](https://clover.coex.tech/en/simulation_native.html).

Parameter change:
```bash
cd clover/clover/launch && vim clover.launch
<arg name="aruco" default="true"/>
cd clover/clover/launch && vim aruco.launch
<arg name="aruco_detect" default="true"/>
<arg name="aruco_map" default="true"/>
<arg name="map" default="cmit.txt"/>
```

Run simulation:
```bash
roslaunch clover_simulation simulator.launch
```
