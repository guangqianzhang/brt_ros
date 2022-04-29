#include "ros/ros.h"
#include "lidar/pcl_brt.h"
int main(int argc, char  *argv[])
{
    ros::init(argc,argv,"read_lidar");
    ROS_INFO("hello lidar");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    pcl_brt lidarRead(nh);
    while(ros::ok()){
    loop_rate.sleep();
        lidarRead.Run();
    }
    return 0;
}
