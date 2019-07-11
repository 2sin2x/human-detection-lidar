# human-detection-lidar

## For simulation:

```
roslaunch velodyne_description start_simulation.launch

rosrun my_pcl_tutorial sub_simulation

rosrun velodyne_height_map heightmap_node _grid_dimensions:=40 _cell_size:=0.25 _full_clouds:=true _height_threshold:=0.6

rosrun my_pcl_tutorial velocity_addon

rosrun robot_pose_publisher robot_pose_publisher_modified

roslaunch kuka_kr10_moveit_config demo.launch
```

## For use with .bag:

```
roscore

rosbag play -l bognar_bag.bag

rviz

rosrun my_pcl_tutorial velocity_addon

rosrun my_pcl_tutorial sub_borongaj

rosrun velodyne_height_map heightmap_node _grid_dimensions:=40 _cell_size:=0.25 _full_clouds:=true _height_threshold:=0.2
```
