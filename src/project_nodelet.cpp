#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


class DarknetRosNodelet : public nodelet::Nodelet {
 public:
  DarknetRosNodelet() = default;
  ~DarknetRosNodelet() {
    
  }

 private:
  virtual void onInit() {
    ros::NodeHandle NodeHandle("~");
    NodeHandle = getPrivateNodeHandle();
    // darknetRos_ = new darknet_ros::YoloObjectDetector(NodeHandle);
    ROS_INFO("successful!!!!++++++++++++++++++");
  }

//   darknet_ros::YoloObjectDetector* darknetRos_;
};

// Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(DarknetRosNodelet, nodelet::Nodelet);