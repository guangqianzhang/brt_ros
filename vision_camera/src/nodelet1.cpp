#include "vision_camera/nodelet1.h"
// #include "vision_camera/fusion_utils.hpp"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <termios.h>
//时间同步
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
//传感器消息
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
// detect
#include "vision_camera/Yolo.h"
 #include <rockauto_msgs/ImageObj.h>
namespace projection
{
    class nodeletclass1 : public nodelet::Nodelet //继承父类nodelet::Nodelet
    {
    public:
        nodeletclass1(); //构造函数，可有可无？
    private:
        virtual void onInit(); //这个虚函数，在启动本Nodelet节点时，自动调用
        void callback(const rockauto_msgs::ImageObjConstPtr &image,
                      const sensor_msgs::PointCloud2ConstPtr &pointcloud);
        cv::Rect imageBox;
        int adjust_x_;
        int adjust_y_;
        std::string form_vision_img_topic_;
        std::string from_lidar_cluster_topic_;

        // Yolo yolo;
        // ros::NodeHandle nh;
        ros::Publisher camera_pub;
        ros::Publisher lidar_pub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub; //雷达订阅
        message_filters::Subscriber<rockauto_msgs::ImageObj> camera_sub;      //相机订阅

        typedef message_filters::sync_policies::ApproximateTime<rockauto_msgs::ImageObj, sensor_msgs::PointCloud2> syncpolicy; //时间戳对齐规则
        typedef message_filters::Synchronizer<syncpolicy> Sync;
        boost::shared_ptr<Sync> sync_; //时间同步器
    };

    nodeletclass1::nodeletclass1()
    {
        ROS_INFO("gouzaohanshu!!");
        // get nodehandle ,dont need init
        // ros::NodeHandle &private_nh = getPrivateNodeHandle();
        ros::NodeHandle nh_("~");
        imageBox.x = 0;
        imageBox.y = 0;
        imageBox.width = 640;
        imageBox.height = 480;
        nh_.param<int>("adjust_x", adjust_x_, 25);
        // LOG(INFO) << "adjust_x_:" << adjust_x_;
        nh_.param<int>("adjust_y", adjust_y_, 120);
        // LOG(INFO) << "adjust_y_:" << adjust_y_;
        nh_.param<std::string>("rs_lidar_topic", from_lidar_cluster_topic_, "/rslidar_points");
        nh_.param<std::string>("camera_topic_raw", form_vision_img_topic_, "/hik_cam_node/hik_camera");
        nh_.param<int>("camera_height", imageBox.height, 480);
        nh_.param<int>("camera_width", imageBox.height, 640);
        ROS_INFO("ad_x:%d", adjust_x_);
        ROS_INFO("ad_y:%d", adjust_y_);
        ROS_INFO("lidar_topic:%s", from_lidar_cluster_topic_.c_str());
        ROS_INFO("camera_topic:%s", form_vision_img_topic_.c_str());
        ROS_INFO("camera_height:%d", imageBox.height);

        /* ————————————————
        版权声明：本文为CSDN博主「kint_zhao」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
        原文链接：https://blog.csdn.net/zyh821351004/article/details/52143309 */
    }

    //重载虚函数，启动时自动调用
    void nodeletclass1::onInit()
    {
        //输出信息
        NODELET_DEBUG("Init nodelet...");
        ROS_INFO("Nodelet is OK for test");
        ros::NodeHandle &private_nh = getPrivateNodeHandle();
        //订阅话题
        lidar_sub.subscribe(private_nh, from_lidar_cluster_topic_, 1);
        camera_sub.subscribe(private_nh, form_vision_img_topic_, 1);

        //发布者
        camera_pub = private_nh.advertise<sensor_msgs::Image>("sync/img", 1000);
        lidar_pub = private_nh.advertise<sensor_msgs::PointCloud2>("sync/lidar", 1000);

        //回调
        sync_.reset(new Sync(syncpolicy(10), camera_sub, lidar_sub));
        sync_->registerCallback(boost::bind(&projection::nodeletclass1::callback, this, _1, _2));
    }

    void nodeletclass1::callback(const rockauto_msgs::ImageObjConstPtr &image,
                                 const sensor_msgs::PointCloud2ConstPtr &pointcloud)
    {
        cv_bridge::CvImagePtr cv_ptr;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image->roi_image, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            std::cout << "cv_bridge Copy failed!" << std::endl;
            return;
        }
        cv::Mat raw_img = cv_ptr->image;
        pcl::fromROSMsg(*pointcloud, *cloud);

        std::vector<detection::YoloDetSt> yoloRet;
        cv::Mat det_img;
        if(0)
        {// det_img=yolo.detect(raw_img, yoloRet);
        }
        else det_img=raw_img;

        cv_ptr->image = det_img;
        camera_pub.publish(cv_ptr->toImageMsg());
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*cloud, ros_cloud);
        lidar_pub.publish(ros_cloud);
    }

}
// nodelet的本质是把节点作为插件来调用，因此需要PLUGINLIB的宏定义、
//第一个参数是类名，第二个参数是父类
PLUGINLIB_EXPORT_CLASS(projection::nodeletclass1, nodelet::Nodelet);