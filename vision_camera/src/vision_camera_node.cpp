#include<ros/ros.h>
#include "vision_camera/Yolo.h"
#include "glog/logging.h"
// #include "vision_camera/SubscriberANDPublisher.hpp"
int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "/home/brt/Documents/catktin_brt/src/vision_camera/Log";
    FLAGS_alsologtostderr = 1;
    LOG(INFO)<<FLAGS_log_dir;
    ros::init(argc,argv,"vision_camera_detect");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
     detection::Yolo vision_yolov4(nh,private_nh);
  
    // Yolo vision_yolov4;
        // subscriberANDpublisher sp;

        // ros::spin();

    return 0;

}