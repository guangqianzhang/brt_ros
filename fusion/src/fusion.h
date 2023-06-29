#pragma once

#include <ros/ros.h>
#include <ros/package.h>
// 时间同步
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
// 传感器消息
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <autoware_msgs/ImageObj.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"


using namespace std;
using namespace autoware_msgs;
class subscriberANDpublisher
{
public:
    subscriberANDpublisher(ros::NodeHandle &nh);
    void callback(const autoware_msgs::ImageObjConstPtr &image, const autoware_msgs::DetectedObjectArrayConstPtr &in_range_detections);
    int adjust_x_;
    int adjust_y_;
    cv::Mat camera_instrinsics_;
    cv::Mat CameraExtrinsicMat_;

private:
    ros::NodeHandle nh;
    ros::Publisher camera_pub;
    ros::Publisher lidar_pub;
    ros::Publisher fused_pub;
    message_filters::Subscriber<autoware_msgs::DetectedObjectArray> lidar_sub; // 雷达订阅
    message_filters::Subscriber<autoware_msgs::ImageObj> camera_sub;           // 相机订阅

    std::string camera_topic_;
    std::string lidar_topic_;

    string lidar_camera_calibration;
    cv::Rect imageBox{0,0,0,0};
    cv::Size image_size_;
    double overlap_threshold_;

    bool Projet_ALL;
    bool Projet_Rect;

    bool DEBUG=false;
    typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::ImageObj, autoware_msgs::DetectedObjectArray> syncpolicy; // 时间戳对齐规则
    typedef message_filters::Synchronizer<syncpolicy> Sync;
    boost::shared_ptr<Sync> sync_; // 时间同步器

    bool IsObjectInImage(const autoware_msgs::DetectedObject &in_detection);
    void TransformRangeToVision(const autoware_msgs::DetectedObjectArrayConstPtr &in_range_detections,
                                autoware_msgs::DetectedObjectArray &out_in_cv_range_detections,
                                autoware_msgs::DetectedObjectArray &out_out_cv_range_detections);
    cv::Rect ProjectDetectionToRect(autoware_msgs::DetectedObject &in_detection);
};