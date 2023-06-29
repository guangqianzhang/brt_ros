#include <ros/ros.h>
#include "fusion.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    subscriberANDpublisher l2c(private_nh);

 

    return 0;

}