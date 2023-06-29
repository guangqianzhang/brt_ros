
#include <std_msgs/String.h>
#include "tracker/BYTETracker.h"
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <float.h>
#include <stdio.h>
#include <vector>

#include "tracker/track_ros.hpp"
using namespace std;


int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "tracker_node");

    // 创建节点句柄
    ros::NodeHandle nh;
    int fps = 30;
    Track_Ros track(fps,nh);
    ros::Rate loop_rate(10); // 发布频率为 10Hz
    while (ros::ok())
    {

        // cv_ptr->image = raw_img;
        // camera_pub.publish(cv_ptr->toImageMsg());
        // 循环等待消息和回调
        // ros::spin();
        ROS_INFO("in while.....................");

        loop_rate.sleep();
    }
    // 等待退出信号
    ros::waitForShutdown();

    return 0;
}