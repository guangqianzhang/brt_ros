## 说明
+ 项目：BRT
+ 时间：2023.6.27
+ 这是python 融合版本
  + yolo-trt算法c++实现 参看https://github.com/wang-xinyu/tensorrtx.git **以及权重生成放置于weights**
  + 点云检测依靠于autoware 的聚类检测。 https://github.com/guangqianzhang/preception_lidar_autoware.git
  + 融合依据为目标iou。由于点云稀疏，远处目标较小。故iou做了写改变。
  + 整个融合消息使用`autoware_msgs`。文件位于上述autoware仓库。

+ 使用
  + 每个功能包目前需要单独运行。c++实现功能包只需执行该包`launch`文件。
  + 融合功能请使用`fusionpy`功能包，且运行指令为`rosrun fusionpy fusion_node.py`
  + 其他请参照正常的ros使用命令。