#include "Yolo_plug.hpp"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
boost::shared_ptr<Yolo> yolo_;
// Yolo_plug yolo_;
image_transport::Publisher vision_detect_img_pub_;
void image_cb(const sensor_msgs::Image::ConstPtr &img){

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
		ROS_INFO("detect done!");
		ros::Time time = ros::Time::now();
		cv_ptr->encoding = "bgr8";
		cv_ptr->header.stamp = time;
		cv_ptr->header.frame_id = img->header.frame_id;
		cv_ptr->image = current_frame;
		vision_detect_img_pub_.publish(cv_ptr->toImageMsg());
}
int main(int argc, char *argv[])
{

    std::string __APP_NAME__;
    std::string camera_topic_out;
    std::string camera_topic;
    std::string camera_topic_msg;

    image_transport::Subscriber camera_raw_sub_;
    
    ros::Publisher vision_detect_msg_pub_;


    ros::init(argc, argv, "node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    // NODELET_INFO("SampleNodeletClass - %s", __FUNCTION__);
    private_nh.param<std::string>("camera_topic", camera_topic, "/hik_cam_node/hik_camera");
    std::cout << __APP_NAME__ << "camera_topic:" << camera_topic.c_str() << std::endl;
    private_nh.param<std::string>("camera_topic_vision", camera_topic_out, "/camera_main");
    std::cout << __APP_NAME__ << "camera_topic_vision:" << camera_topic_out.c_str() << std::endl;
    private_nh.param<std::string>("camera_topic_msg", camera_topic_msg, "/camera_topic_msg");
    std::cout << __APP_NAME__ << "camera_topic_msg:" << camera_topic_msg.c_str() << std::endl;
    yolo_.reset((new Yolo));
    image_transport::ImageTransport it(private_nh);
    camera_raw_sub_ = it.subscribe(camera_topic, 1, image_cb);
    vision_detect_img_pub_ = it.advertise(camera_topic_out, 1);
    // vision_detect_msg_pub_ = private_nh.advertise<rockauto_msgs::ImageObj>(camera_topic_msg, 1);
    ros::spin();

    return 0;
}
