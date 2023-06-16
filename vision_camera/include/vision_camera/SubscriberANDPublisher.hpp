#include <ros/ros.h>
//时间同步
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
//传感器消息
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include "Yolo.h"
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <termios.h>
#include "fusion_utils.hpp"
class subscriberANDpublisher : public Yolo()
{
public:
    subscriberANDpublisher();
    void callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &pointcloud);

private:
    cv::Rect imageBox;
    int adjust_x_;
    int adjust_y_;

    ros::NodeHandle nh;
    ros::Publisher camera_pub;
    ros::Publisher lidar_pub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub; //雷达订阅
    message_filters::Subscriber<sensor_msgs::Image> camera_sub;      //相机订阅

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> syncpolicy; //时间戳对齐规则
    typedef message_filters::Synchronizer<syncpolicy> Sync;
    boost::shared_ptr<Sync> sync_; //时间同步器
};
/* ————————————————
版权声明：本文为CSDN博主「火柴的初心」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/He3he3he/article/details/109643391 */

subscriberANDpublisher::subscriberANDpublisher()
{

    //订阅话题
    lidar_sub.subscribe(nh, "/points_raw", 1);
    camera_sub.subscribe(nh, Yolo::camera_topic, 1);

    //发布者
    camera_pub = nh.advertise<sensor_msgs::Image>("sync/img", 1000);
    lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("sync/lidar", 1000);

    //回调
    sync_.reset(new Sync(syncpolicy(10), camera_sub, lidar_sub));
    sync_->registerCallback(boost::bind(&subscriberANDpublisher::callback, this, _1, _2));

    imageBox.x = 0;
    imageBox.y = 0;
    imageBox.width = 640;
    imageBox.height = 480;
    ros::NodeHandle nh_("~");
    nh_.param<int>("/vision_camera_node/adjust_x", adjust_x_, 25);
    LOG(INFO) << "adjust_x_:" << adjust_x_;
    nh_.param<int>("/vision_camera_node/adjust_y", adjust_y_, 120);
    LOG(INFO) << "adjust_y_:" << adjust_y_;
}
void subscriberANDpublisher::callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &pointcloud)
{
    ROS_INFO("done! ");

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, "bgr8");
    }
    catch (cv_bridge::Exception &e)
    {
        LOG(INFO) << "cv_bridge Copy failed!";
        return;
    }
    cv::Mat raw_img = cv_ptr->image;
    // ros::Time time = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pointcloud, *cloud);
    LOG(INFO) << "1. LOAD the Lidar and image frame Done!";

    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);
    for (auto it1 = cloud->points.begin(); it1 != cloud->points.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;
        Y = CV_Transformlidar(X, Yolo::camera_instrinsics_, Yolo::CameraExtrinsicMat_);
        // std::cout<<"\nX:"<<X<<std::endl;
        // std::cout<<"Y"<<Y<<std::endl;
        // std::cout<<"00:"<<Y.at<double>(0, 0)<<" 01:"<<Y.at<double>(0, 1)<<" 02:"<<Y.at<double>(0, 2)<<std::endl;
        // std::cout<<"10:"<<Y.at<double>(1, 0)<<" 11:"<<Y.at<double>(1,1)<<" 12:"<<Y.at<double>(1, 2)<<std::endl;

        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);
        pt.x = pt.x + adjust_x_;
        pt.y = pt.y + adjust_y_;

        if (imageBox.contains(pt))
        {
            drawPoint(raw_img, pt);
        }
    }
    std::cout << imageBox << std::endl;
    cv::rectangle(raw_img, cv::Point(0, 0), cv::Point(imageBox.width, imageBox.height), cv::Scalar(0, 255, 0), 2);

    // cv_ptr->encoding = "bgr8";
    // cv_ptr->header.stamp = time;
    // cv_ptr->header.frame_id = "rslidar";
    cv_ptr->image = raw_img;
    camera_pub.publish(cv_ptr->toImageMsg());
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    lidar_pub.publish(ros_cloud);
}