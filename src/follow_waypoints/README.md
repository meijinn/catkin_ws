# follow_waypoints

**This package follws waypoints from csv**

## Requirement
+ Ubuntu 18.04
+ ROS (Melodic)
+ Python 2.7.x

## Set Up
1. Download `follow_waypoints` package.

```shell
$ cd ~/catkin_ws/src/
$ git clone https://github.com/HHorimoto/follow_waypoints.git
$ cd ~/catkin_ws
$ catkin_make
```

## How To Use

1. Set waypoints `x,y` location in csv file. Here is example.

```shell
$ roscd follow_waypoints/csv/
cat waypoints.csv
num,x,y # header name, you must write it head of your csv.
0,-4.8252396827,-1.2530685796 # num, x, y
1,-18.9302072058,-3.27433665634
2,-31.9823555043,-11.34231223581
```

2. Launch this launch file.

```shell
$ roslaunch follow_waypoints run.launch
```

### Parameters

+ ***use_py*** : if you use python script or cpp script. `true->python`, `false->cpp`
    default : `true`

+ ***duration*** : wait time after robot reaches goal.
    default : `0.0` [s]

+ ***distance_tolerance*** : distance from goal to be considered goal.
    default : `1.0` [m]

+ ***waypoints_name*** : csv file where waypoints are stored.
    default : `waypoints`

+ ***path_to_waypoints*** : path to waypoints csv. I belive that  you dont need to change this param.
    default : `$(find follow_waypoints)/csv/$(arg waypoints_name).csv`

## References
Daniel Snider, 2017. http://wiki.ros.org/follow_waypoints (accessed 2022 Aug 17)