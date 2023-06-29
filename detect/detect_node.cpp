
#include <ros/ros.h>
#include "detect/detect.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_camera_detect");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    detect vision_yolov4(nh, private_nh);

 

    return 0;

}