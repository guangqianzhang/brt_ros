仓库地址：https://github.com/guangqianzhang/BRT_ROS.git
## 项目介绍
1. 输入：速腾16线雷达、3个 海斯康威网络摄像头（实际只用了一个）
2. 检测公交等特殊车辆 输出：融合后的检测结果以及位置 

## 项目更新
+ 项目：BRT
+ 时间：2023.6.27
+ 这是python 融合版本
  + yolo-trt算法c++实现 参看https://github.com/wang-xinyu/tensorrtx.git **以及权重生成放置于weights**
  + 点云检测依靠于autoware 的聚类检测。 https://github.com/guangqianzhang/preception_lidar_autoware.git
  + 融合依据为目标iou。由于点云稀疏，远处目标较小。故iou做了写改变。
  + 整个融合消息使用`autoware_msgs`。文件位于上述autoware仓库。
  + tracker实现了ros框架下的byteTrack算法。需要根据需要修改配置检测消息。

+ 使用
  + 每个功能包目前需要单独运行。c++实现功能包只需执行该包`launch`文件。
  + 融合功能请使用`fusionpy`功能包，且运行指令为`rosrun fusionpy fusion_node.py`
  + 其他请参照正常的ros使用命令。

## code
时间：2022.5.27
控制框架：ROS
其中利用nodelet加快数据传递。
修改了 rockauto_msgs 中 ImageObj.msg 等文件， 
### 代码结构

|  输入   | 算法  | 融合 |track|输出|
|  ----  | ----  |---|---|----|
| 摄像头  | YOLO（nodelet） | |  | |
|| |检测框投影融合|检测目标 `fused_objects`|byteTrack|
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
5. `2023/6/16` 启动跟踪。 目前跟踪输入为融合后目标 。`rosun tracker track_node`
     启动检测 融合节点 进入ros目录  `roslaunch detect_nodelet nodelet.launch`
 开发

融合消息后消息通过话题 `sync/fused_msg` 发出，消息类型是功能包中`DetectedObjectArray`，**所以二次开发时需要引入消息功能包`rockauto_msgs`**。

