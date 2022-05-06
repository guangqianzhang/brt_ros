#include "ros/ros.h"

#include "glog/logging.h"
#include "brt_detection/global_defination/global_defination.h"
#include "brt_detection/project/project.hpp"
// #include "ros_detection/sensor_data/cloud_data.hpp"
using namespace DetectandTract;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;
    LOG(INFO)<<FLAGS_log_dir;
    ros::init(argc, argv, "test_frame_node");
    ros::NodeHandle nh;
    DetectandTract::projector projector;
    

}