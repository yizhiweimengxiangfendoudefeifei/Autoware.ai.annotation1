# Autoware.ai.annotation1
这是我们将autoware部署在车辆过程中写的代码注释！
# 基本操作指南
https://blog.csdn.net/qq_40252459/article/details/123287437
# 测试功能
1.配置can卡  
2.可以添加激光雷达的聚类，并且画盒子表示  
3.问题是current lane doesn't have change flag   节点是lane_select  
Necessary topics are not subscribed yet ...     节点是pure_pursuit
4.发一个节点让can发送消息：rostopic pub -r 20 /Control_acc std_msgs/Float64 "data: 0.1"  
5.128线激光雷达、将128线激光雷达加入qt界面中,将激光雷达加到autoware中，注意将launch文件改回log  
6.修改qt界面中的配置文件在/home/navigation/autoware.ai/install/runtime_manager/lib/runtime_manager  
7.单独编译一个节点AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select pure_pursuit  
8.如果要改寻迹点的速度，需要修改以下：  
<arg name="const_velocity" default="5.0"/>
 private_nh_.param("const_velocity", const_velocity_, 5.0);  
 9.适配速腾激光雷达：要下载rslidar_sdk，修改config中的config.yaml的frame_id为velodyne,再<remap from="rslidar_points" to="/points_raw" />  
 10.节点waypoint_loader注释 
 11.最重要的发现！！！vehicle_cmd包含所有的控制消息，包含四元数，线速度，加速度，转角  
 12.标定命令 roslaunch autoware_camera_lidar_calibrator camera_lidar_calibration.launch intrinsics_file:=/home/ne0/Desktop/calib_heat_camera1_rear_center_fisheye.yaml compressed_stream:=True camera_id:=camera1  
 13.astar_avoid节点配置说明:  
 主要的内容在大循环while (ros::ok())中，重新规划的方法在其中有详细的描述，需要详细看，2022-0320前看完  
14.relaying->stoping->planning->avoiding->relaying  
15. /points_no_ground (points above ground)，是指在地面以上的点云，那么车应该是points_no_ground？  
16. 注意看博客园中大佬写的openplanner文章。  
# 问题
1.steering_robot_有一个节点在哪？  
2.问题For frame [velodyne]: No transform to fixed frame [world].  TF error: [Could not find a connection between 'world' and 'velodyne' because they are not part of the same tree.Tf has two or more unconnected trees.]  
3.融合
range_vision_fusion.launch   
问题：相机没有标定  
4.打开ring_ground_filter和ray_ground_filter后会出现错误  
[ WARN] [1640874586.661332263]: Lookup would require extrapolation into the past.  Requested time 1640874461.921009664 but the earliest data is at time 1640874576.669836191, when looking up transform from frame [velodyne] to frame [base_link]  
[ERROR] [1640874622.966605713]: Failed transform from base_link to velodyne  
打开voxel_grid_filter后出现错误：  
Failed to find match for field 'intensity'.  
[ WARN] [1640876153.420807323]: Cannot get closest waypoints. All closest waypoints are changed to -1.  
[ INFO] [1640876270.511634298]: Found GOAL at index = 298  
[ INFO] [1640876270.517372378]: PLANNING -> AVOIDING, Found path  
avoiding  
[ INFO] [1640876270.310769791]: AVOIDING -> STOPPING, Abort avoiding  
[ INFO] [1640876270.411066248]: STOPPING -> PLANNING, Start A* planning  
planning  
错误：
Error: out of memory /home/navigation/autoware.ai/src/autoware/core_perception/ndt_gpu/src/SymmetricEigenSolver.cu 13  
Error: driver shutting down /home/navigation/autoware.ai/src/autoware/core_perception/ndt_gpu/src/MatrixDevice.cu 30  
[ndt_mapping-2] process has died [pid 23726, exit code 1, cmd /home/navigation/autoware.ai/install/lidar_localizer/lib/lidar_localizer/ndt_mapping __name:=ndt_mapping __log:=/home/navigation/.ros/log/93ca424c-6fba-11ec-b1fd-78d00426050d/ndt_mapping-2.log].  
log file: /home/navigation/.ros/log/93ca424c-6fba-11ec-b1fd-78d00426050d/ndt_mapping-2*.log
# 教程
安装autoware 1.14的教程：  
https://www.cnblogs.com/hgl0417/p/14617025.html  
https://gitlab.com/autowarefoundation/autoware.ai/autoware/-/wikis/Source-Build  
关于避障astar_avoid的网站：  
https://zhuanlan.zhihu.com/p/163844578  
https://blog.csdn.net/xiaoxiao123jun/article/details/111089307  
关于适配速腾激光雷达：  
https://blog.csdn.net/yz2630570484/article/details/114232333  
关于融合：  
https://blog.csdn.net/muyiyushan/article/details/119760428  

