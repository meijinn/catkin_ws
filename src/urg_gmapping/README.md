## About

Example for using URG-04LX with gmapping and laser_scan_matcher of ROS node

## Install the dependency packages

```sh
$ sudo apt-get install ros-<distro>-laser-scan-matcher
$ sudo apt-get install ros-<distro>-gmapping
$ sudo apt-get install ros-<distro>-urg-node
```

## Usage

```sh
$ roslaunch urg_gmapping urg_simple_gmapping.launch
```
or
```sh
$ roslaunch urg_gmapping urg_run.launch
$ roslaunch urg_gmapping laser_odometry.launch
$ roslaunch urg_gmapping gmapping.launch
```

## Rviz

![urg_simple_gmapping](https://github.com/meijinn/urg_gmapping/assets/27892408/5396cdd4-3910-4741-aee6-87b784ba5e5a)
