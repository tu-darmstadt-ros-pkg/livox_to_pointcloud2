# livox_to_pointcloud2

ROS2 node to convert customized pointcloud data in livox_ros_driver2 to sensor_msgs/PointCloud2 type messages
https://github.com/Livox-SDK/livox_ros_driver2

Launch:
```ros2 launch livox_to_pointcloud2 livox_to_pointcloud2_launch.py```

Arguments:
 - ```input_topic```
 - ```output_topic```

If no arguments are provided, the node will subscribe to all topics of type livox_ros_driver2/CustomMsg and publish them as sensor_msg/PointCloud2 on "<original_topic>_pc2"
