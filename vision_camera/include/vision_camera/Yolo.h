#pragma once
#include <iostream>
#include "opencv2/dnn.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <time.h>
#include <ctime>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
// #include <rockauto_msgs/DetectedObjectArray.h>
#include <rockauto_msgs/ImageObj.h>
namespace detection
{
	typedef struct YoloDetSt
	{
		std::string label;
		float confidences;
		// 	 　　int x; /* 方形的左上角的x-坐标 */
		// 　　int y; /* 方形的左上角的y-坐标*/
		// 　　int width; /* 宽 */
		// 　　int height; /* 高 */
		cv::Rect rect;
	} YoloDetSt;

	class Yolo
	{
	public:
		Yolo(ros::NodeHandle node,
                               ros::NodeHandle private_nh,
							   std::string const & node_name = ros::this_node::getName());
		~Yolo();
		cv::Mat detect(cv::Mat raw_img, std::vector<YoloDetSt> &yoloRet);
		std::string camera_topic;
		cv::Size image_size_;
		cv::Mat camera_instrinsics_;
		cv::Mat distortion_coefficients_;
		cv::Mat CameraExtrinsicMat_;

	private:
		std::string medel_path;
		std::string yoloModelWeights;
		std::string yoloModelConfiguration;
		std::string yoloclassfile;
		std::vector<std::string> m_outNames;
		bool m_isGpu = false;
		std::vector<std::string> m_classes;
		cv::dnn::Net m_net;

		// Yolo��������
		float m_confThreshold = 0.5;
		float m_nmsThreshold = 0.4;
		float m_scale = 0.00392;
		cv::Scalar m_mean = {0, 0, 0};
		bool m_swapRB = true;
		int m_inpWidth = 416;
		int m_inpHeight = 416;

		// ����ͼƬ�������Ƶ�Ĳ���
		int m_saveH = 0;
		int m_saveW = 0;
		cv::VideoWriter m_viderWriter;
		std::string m_viderName;
		int m_frames = 0;
		cv::Mat current_frame_;

		std::string __APP_NAME__;
		std::string camera_topic_out;
		// std::string camera_topic;
		std::string camera_topic_msg;
		std::string camera_topic_info;

		image_transport::Subscriber camera_raw_sub_;
		image_transport::Publisher vision_detect_img_pub_;
		ros::Publisher vision_detect_msg_pub_;
		ros::Publisher vision_camera_info_pub_;

		void initParams(ros::NodeHandle private_nh,std::string const & node_name);
		// void postprocess(cv::dnn:Net& net,cv::Mat& img);
		void image_cb(const sensor_msgs::Image::ConstPtr &img);

		void postprocess(cv::Mat &frame, const std::vector<cv::Mat> &outs, cv::dnn::Net &net, std::vector<YoloDetSt> &yoloRet);
		std::string getLocNameTime(); // ���ظ�ʽ��ʱ�䣺20200426_150925
		void setViderWriterPara(const cv::Mat &img);
		int loadModel();
		int runningYolo(cv::Mat &img, std::vector<YoloDetSt> &yoloRet);
		void drowBoxes(cv::Mat &img, std::vector<YoloDetSt> &yoloRet);
		void saveVider(cv::Mat img, std::vector<YoloDetSt> &yoloRet);
	};
}