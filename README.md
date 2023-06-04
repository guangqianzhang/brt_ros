仓库地址：https://github.com/guangqianzhang/BRT_ROS.git
## 项目介绍
1. 输入：速腾16线雷达、3个 海斯康威网络摄像头（实际只用了一个）
2. 检测公交等特殊车辆 输出：融合后的检测结果以及位置 

## code
控制框架：ROS
其中利用nodelet加快数据传递。
修改了 rockauto_msgs 中 ImageObj.msg 等文件， 
### 代码结构

|  输入   | 算法  | 融合 |输出|
|  ----  | ----  |----|----|
| 摄像头  | YOLO（nodelet） | |  |
|| |检测框投影融合|检测目标 `rockauto_msgs::DetectedObjectArray(fused_objects)`|
| 雷达  | Apollo（cnn_seg） | ||

## 使用 

 启动
    
    
    
   

如果不能启动请利用  `rqt` 依次检查 节点消息
1. 海康摄像头是否接入 `hikvision_ros` 、网设置是否正确，官方提供的动态库是否需要更新。
    仓库为  https://github.com/guangqianzhang/hika_ros.git  
    启动相机  进入ros目录 `roslaunch hikvision_ros hik.launch`
2. 雷达是否接入  `ros_rslidar` 、雷达网口设置是否正确。
    仓库为 https://github.com/RoboSense-LiDAR/ros_rslidar.git

    启动雷达  进入ros目录 `roslaunch rslidar_pointcloud rs_lidar_16.launch`
3. 雷达检测  `lidar_cnn_seg_detect`
4. 消息同步节点是否由数据输入输出。 `detect_nodelet`
     
     启动检测 融合节点 进入ros目录  `roslaunch detect_nodelet nodelet.launch`
 开发

融合消息后消息通过话题 `sync/fused_msg` 发出，消息类型是功能包中`DetectedObjectArray`，**所以二次开发时需要引入消息功能包`rockauto_msgs`**。
