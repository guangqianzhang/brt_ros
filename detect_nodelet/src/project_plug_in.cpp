#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <rockauto_msgs/DetectedObjectArray.h>
#include <rockauto_msgs/ImageObj.h>
#include "SubscriberANDPublisher.hpp"
namespace projetction
{
    class project_plug : public nodelet::Nodelet
    {
    private:
        virtual void onInit();
        void CallBack(const rockauto_msgs::ImageObjConstPtr &msg);
        ros::Subscriber form_vision_msg_;
        ros::Publisher projeted_image_;
        std::string vision_msgs_topic_;
        std::string projeted_image_topic_;
        std::string form_lidar_topic_;
        std::string __APP_NAME__;
        boost::shared_ptr<subscriberANDpublisher> projetor_;

    public:
        project_plug(/* args */);
        ~project_plug();
    };

    project_plug::project_plug(/* args */)
    {
                __APP_NAME__ = ros::this_node::getName();
    }

    project_plug::~project_plug()
    {
    }
    void project_plug::onInit()
    {
        NODELET_INFO("SampleNodeletClass - %s", __FUNCTION__);
        ros::NodeHandle nh=getNodeHandle();
        ros::NodeHandle private_nh = getPrivateNodeHandle();
        private_nh.param<std::string>("vision_msgs_topic", vision_msgs_topic_, "/camera_main");
        std::cout << __APP_NAME__ << "vision_msgs_topic:" << vision_msgs_topic_.c_str() << std::endl;
        private_nh.param<std::string>("pointcluster",form_lidar_topic_,"rslidar_points");
        std::cout << __APP_NAME__ << "pointcluster:" << form_lidar_topic_.c_str() << std::endl;
        // private_nh.param<std::string>("projeted_image_topic", projeted_image_topic_, "/camera_topic_msg");
        // std::cout << __APP_NAME__ << "projeted_image_topic:" << projeted_image_topic_.c_str() << std::endl;

        // form_vision_msg_ = private_nh.subscribe<rockauto_msgs::ImageObj>(vision_msgs_topic_, 1, &project_plug::CallBack, this);
        // projeted_image_ = private_nh.advertise<sensor_msgs::Image>("projeted_image", 10,true);

        projetor_.reset(new subscriberANDpublisher(form_lidar_topic_,vision_msgs_topic_));
        // projetor_->set_camera_topic(vision_msgs_topic_);
        // projetor_->set_lidar_topic(form_lidar_topic_);
    }
    void project_plug::CallBack(const rockauto_msgs::ImageObjConstPtr &msg)
    {

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg->roi_image, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_INFO("cv_bridge Copy failed!");
            return;
        }
        cv::Mat raw_img = cv_ptr->image;

        projeted_image_.publish(cv_ptr->toImageMsg());
    }

}
PLUGINLIB_EXPORT_CLASS(projetction::project_plug, nodelet::Nodelet)