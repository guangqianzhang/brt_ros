// #include "vision_nodelet/yolo_plug_in.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "Yolo_plug.hpp"
#include <rockauto_msgs/ImageObj.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

namespace vision_detection
{
    class yolo_plug_in : public nodelet::Nodelet
    {
    private:
        /* data */
        virtual void onInit();
        void image_cb(const sensor_msgs::Image::ConstPtr &img);
        volatile bool running_;
        boost::shared_ptr<Yolo> yolo_;
        // Yolo_plug yolo;

        std::string __APP_NAME__;
        std::string camera_topic_out;
        std::string camera_topic;
        std::string camera_topic_msg;

        image_transport::Subscriber camera_raw_sub_;
        image_transport::Publisher vision_detect_img_pub_;
        ros::Publisher vision_detect_msg_pub_;

    public:
        yolo_plug_in(/* args */);
        ~yolo_plug_in();
    };

    yolo_plug_in::yolo_plug_in(/* args */)
    {
        ROS_INFO("SampleNodeletClass Constructor");
        __APP_NAME__ = ros::this_node::getName();
    }

    yolo_plug_in::~yolo_plug_in()
    {
        ROS_INFO("SampleNodeletClass Destructor");
    }
    void yolo_plug_in::onInit()
    {
        running_ = true;
        yolo_.reset(new Yolo);
        NODELET_INFO("SampleNodeletClass - %s", __FUNCTION__);
        ros::NodeHandle private_nh = getPrivateNodeHandle();
        private_nh.param<std::string>("camera_topic", camera_topic, "/kitti/camera_gray_left/image_raw");
        std::cout << __APP_NAME__ << "camera_topic:" << camera_topic.c_str() << std::endl;
        private_nh.param<std::string>("camera_topic_vision", camera_topic_out, "/camera_main");
        std::cout << __APP_NAME__ << "camera_topic_vision:" << camera_topic_out.c_str() << std::endl;
        private_nh.param<std::string>("camera_topic_msg", camera_topic_msg, "/camera_topic_msg");
        std::cout << __APP_NAME__ << "camera_topic_msg:" << camera_topic_msg.c_str() << std::endl;

        image_transport::ImageTransport it(private_nh);
        camera_raw_sub_ = it.subscribe(camera_topic, 1, &yolo_plug_in::image_cb, this);
        vision_detect_img_pub_ = it.advertise(camera_topic_out, 1);
        vision_detect_msg_pub_ = private_nh.advertise<rockauto_msgs::ImageObj>(camera_topic_msg, 1);
        // ros::spin();
    }
    void yolo_plug_in::image_cb(const sensor_msgs::Image::ConstPtr &img)
	{

// auto time = img->header.stamp;
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_INFO( "cv_bridge Copy failed!");
			return;
		}
		cv::Mat raw_img = cv_ptr->image;

		// std::cout << "have done detect image! spend time" << spend_time * 1000 << "ms" << std::endl;
		std::vector<YoloDetSt> yoloRet;
        cv::Mat current_frame;
		current_frame=yolo_->detect(raw_img, yoloRet);
		// ROS_INFO("detect done!");
		// ros::Time time = ros::Time::now();
		cv_ptr->encoding = "bgr8";
		cv_ptr->header.stamp = img->header.stamp;
		cv_ptr->header.frame_id = img->header.frame_id;
		cv_ptr->image = current_frame;
		    sensor_msgs::Image::ConstPtr  image=cv_ptr->toImageMsg();
		vision_detect_img_pub_.publish(image);
		rockauto_msgs::ImageObj imagObj;
			imagObj.header = img->header;
			// imagObj.roi_image=*image;
			imagObj.roi_image=*img;
			
	for (auto it = yoloRet.begin(); it != yoloRet.end(); ++it)
	{
		rockauto_msgs::ImageRect imagrect;
		imagObj.type.push_back((*it).label);
		imagrect.x = (*it).rect.x;
		imagrect.y = (*it).rect.y;
		imagrect.height = (*it).rect.height;
		imagrect.width = (*it).rect.width;
		imagrect.score = (*it).confidences;
		imagObj.obj.push_back(imagrect);
	}
	vision_detect_msg_pub_.publish(imagObj);

	}

}
PLUGINLIB_EXPORT_CLASS(vision_detection::yolo_plug_in, nodelet::Nodelet)
