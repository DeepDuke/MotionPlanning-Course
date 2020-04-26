# A* algorithm implementations 

## About A*

- [ ] introduction about A*

## 2D version (implemented in MATLAB Environment)
### Prerequisites

- MATLAB

### Usage

Run  `main.m`  in your MATLAB. The algorithm implementation is in `A_star_search.m` 

### Result Example
<img src="Matlab\result\result2.png" style="zoom:30%;" />

## 3D version (implemented in ROS Environment)

### Prerequisites

- Ubuntu (>=16.04)
- ROS (>= kinetic)

### Usage

Put  `grid_path_searcher` , `rviz_plugins` and `waypoint_generator` in your catkin work space, e.g. :`~/catkin_ws/src` . Then run  the following commands and you can see the similar  result:

```shell
cd ~/catkin_ws
catkin_make
source ~/.catkin_ws/devel/setup.bash
roslaunch grid_path_searcher demo.launch
```

### Result Example

<img src="ROS\images\euclidean.png" style="zoom:30%;" />