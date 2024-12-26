# TopicCompare

此部分用于对比LIO-SAM中订阅话题与实际项目话题

## 1、当前话题

```sheel
/camera/camera_info
/camera/image_raw
/camera/image_raw/compressed
/camera/image_raw/compressed/parameter_descriptions
/camera/image_raw/compressed/parameter_updates
/camera/image_raw/compressedDepth
/camera/image_raw/compressedDepth/parameter_descriptions
/camera/image_raw/compressedDepth/parameter_updates
/camera/image_raw/theora
/camera/image_raw/theora/parameter_descriptions
/camera/image_raw/theora/parameter_updates
/camera/parameter_descriptions
/camera/parameter_updates
/clicked_point
/clock
/cmd_vel
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/imu
/initialpose
/joint_states
/move_base_simple/goal
/odom
/rosout
/rosout_agg
/tf
/tf_static
/velodyne_points
```

## 2、LIO—SAM播放包时的话题

```sheel
/clock
/diagnostics
/gx5/gps/fix
/gx5/nav/odom
/gx5/nav/status
/imu_correct
/imu_raw
/points_raw
/rosout
/rosout_agg
/velodyne_nodelet_manager/bond
/velodyne_nodelet_manager_cloud/parameter_descriptions
/velodyne_nodelet_manager_cloud/parameter_updates
/velodyne_nodelet_manager_driver/parameter_descriptions
/velodyne_nodelet_manager_driver/parameter_updates
/velodyne_packets
```

LIO-SAM实际运行时的topic：

```sheel
zhao@zhao:~/WS/msfl_ws$ rostopic list 
/clock
/diagnostics
/gps/fix
/gx5/gps/fix
/gx5/nav/odom
/gx5/nav/status
/imu_correct
/imu_raw
/joint_states
/lio_loop/loop_closure_detection
/lio_sam/deskew/cloud_deskewed
/lio_sam/deskew/cloud_info
/lio_sam/feature/cloud_corner
/lio_sam/feature/cloud_info
/lio_sam/feature/cloud_surface
/lio_sam/imu/path
/lio_sam/mapping/cloud_registered
/lio_sam/mapping/cloud_registered_raw
/lio_sam/mapping/icp_loop_closure_corrected_cloud
/lio_sam/mapping/icp_loop_closure_history_cloud
/lio_sam/mapping/loop_closure_constraints
/lio_sam/mapping/map_global
/lio_sam/mapping/map_local
/lio_sam/mapping/odometry
/lio_sam/mapping/odometry_incremental
/lio_sam/mapping/path
/lio_sam/mapping/slam_info
/lio_sam/mapping/trajectory
/odometry/gps
/odometry/gpsz
/odometry/imu
/odometry/imu_incremental
/odometry/navsat
/points_raw
/rosout
/rosout_agg
/set_pose
/tf
/tf_static
/velodyne_nodelet_manager/bond
/velodyne_nodelet_manager_cloud/parameter_descriptions
/velodyne_nodelet_manager_cloud/parameter_updates
/velodyne_nodelet_manager_driver/parameter_descriptions
/velodyne_nodelet_manager_driver/parameter_updates
/velodyne_packets
```

其中rosbag info 输出得到的详细信息：

```sheel
zhao@zhao:~/WS/msfl_ws$ rosbag info data/ROSBAG/walking_dataset.bag 
path:        data/ROSBAG/walking_dataset.bag
version:     2.0
duration:    10:55s (655s)
start:       Nov 22 2019 04:16:18.46 (1574367378.46)
end:         Nov 22 2019 04:27:14.40 (1574368034.40)
size:        3.7 GB
messages:    689090
compression: none [3312/3312 chunks]
types:       bond/Status                           [eacc84bf5d65b6777d4c50f463dfb9c8]
             diagnostic_msgs/DiagnosticArray       [60810da900de1dd6ddd437c3503511da]
             dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
             dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
             nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
             rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/NavSatFix                 [2d3a8cd499b9b4a0249fb98fd05cfa48]
             sensor_msgs/PointCloud2               [1158d486dd51d683ce2f1be655c3c181]
             std_msgs/Int16MultiArray              [d9338d7f523fcb692fae9d0a0e9f067c]
             velodyne_msgs/VelodyneScan            [50804fc9533a0e579e6322c04ae70566]
topics:      /diagnostics                                                1299 msgs    : diagnostic_msgs/DiagnosticArray      
             /gx5/gps/fix                                                2623 msgs    : sensor_msgs/NavSatFix                
             /gx5/nav/odom                                               6557 msgs    : nav_msgs/Odometry                    
             /gx5/nav/status                                             6557 msgs    : std_msgs/Int16MultiArray             
             /imu_correct                                              327859 msgs    : sensor_msgs/Imu                      
             /imu_raw                                                  327870 msgs    : sensor_msgs/Imu                      
             /points_raw                                                 6502 msgs    : sensor_msgs/PointCloud2              
             /rosout                                                      355 msgs    : rosgraph_msgs/Log                     (6 connections)
             /rosout_agg                                                  338 msgs    : rosgraph_msgs/Log                    
             /velodyne_nodelet_manager/bond                              2624 msgs    : bond/Status                           (3 connections)
             /velodyne_nodelet_manager_cloud/parameter_descriptions         1 msg     : dynamic_reconfigure/ConfigDescription
             /velodyne_nodelet_manager_cloud/parameter_updates              1 msg     : dynamic_reconfigure/Config           
             /velodyne_nodelet_manager_driver/parameter_descriptions        1 msg     : dynamic_reconfigure/ConfigDescription
             /velodyne_nodelet_manager_driver/parameter_updates             1 msg     : dynamic_reconfigure/Config           
             /velodyne_packets                                           6502 msgs    : velodyne_msgs/VelodyneScan
```
