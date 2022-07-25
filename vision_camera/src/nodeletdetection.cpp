
// #include "vision_camera/fusion_utils.hpp"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

// detect
#include "vision_camera/Yolo.h"
// #include "vision_camera/Yolo_plug_in.hpp"
#include <boost/thread.hpp>
namespace detection
{
    class detectionclass1 : public nodelet::Nodelet  //继承父类nodelet::Nodelet
                            
    {
    public:
        detectionclass1(); 
        ~detectionclass1(){
            if(running_){
                running_=false;
            }
        }
    private:
        virtual void onInit(); //这个虚函数，在启动本Nodelet节点时，自动调用
        void callback(const sensor_msgs::ImageConstPtr &image );
        cv::Rect imageBox;
        std::string form_vision_img_topic_;
        std::string from_lidar_cluster_topic_;
        // ros::NodeHandle nh;
        ros::Publisher camera_pub;
        // ros::Publisher lidar_pub;
        ros::Subscriber camera_sub;
volatile bool running_; 
       boost::shared_ptr<Yolo> yolo_;
       boost::shared_ptr<boost::thread> deviceThread_;
    };

    detectionclass1::detectionclass1():running_(false)
    {
        ROS_INFO(" yolo detection gouzaohanshu!!");
        // get nodehandle ,dont need init
        // ros::NodeHandle &private_nh = getPrivateNodeHandle();
        ros::NodeHandle nh_("~");
   
        // LOG(INFO) << "adjust_y_:" << adjust_y_;
        nh_.param<std::string>("rs_lidar_topic", from_lidar_cluster_topic_, "/rslidar_points");
        nh_.param<std::string>("camera_topic_raw", form_vision_img_topic_, "/hik_cam_node/hik_camera");
        nh_.param<int>("camera_height", imageBox.height, 480);
        nh_.param<int>("camera_width", imageBox.height, 640);

        ROS_INFO("lidar_topic:%s", from_lidar_cluster_topic_.c_str());
        ROS_INFO("camera_topic:%s", form_vision_img_topic_.c_str());
        ROS_INFO("camera_height:%d", imageBox.height);
        

    }

    //重载虚函数，启动时自动调用
    void detectionclass1::onInit()
    {
        //输出信息
        NODELET_DEBUG("Init yolo detection nodelet...");
        ROS_INFO("Nodelet is yolo detection OK for test");
        // ros::NodeHandle &private_nh = getPrivateNodeHandle();
        // // 订阅话题
        // // lidar_sub.subscribe(private_nh, from_lidar_cluster_topic_, 1);
        // camera_sub=private_nh.subscribe(form_vision_img_topic_, 1, &detectionclass1::callback,this);

        // // 发布者
        // camera_pub = private_nh.advertise<sensor_msgs::Image>("sync/img", 1000);
        // lidar_pub = private_nh.advertise<sensor_msgs::PointCloud2>("sync/lidar", 1000);
        yolo_.reset(new Yolo(getNodeHandle(), getPrivateNodeHandle(), getName()));
        // yolo_->initParams();
        running_ = true;
    //     deviceThread_ = boost::shared_ptr< boost::thread >
    // (new boost::thread(boost::bind(&DriverNodelet::callback, this)));

        // ros::spin();
        
   
    }

    void detectionclass1::callback(const sensor_msgs::ImageConstPtr &image)
    {
        ROS_INFO("get image!");
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            std::cout << "cv_bridge Copy failed!" << std::endl;
            return;
        }
        cv::Mat raw_img = cv_ptr->image;
      

        // std::vector<YoloDetSt> yoloRet;
        cv::Mat det_img;
        if(0)
        {
            // det_img=yolo->detect(raw_img, yoloRet);
        }
        else det_img=raw_img;

        cv_ptr->image = det_img;
        camera_pub.publish(cv_ptr->toImageMsg());
        // sensor_msgs::PointCloud2 ros_cloud;
    }

}
// nodelet的本质是把节点作为插件来调用，因此需要PLUGINLIB的宏定义、
//第一个参数是类名，第二个参数是父类
PLUGINLIB_EXPORT_CLASS(detection::detectionclass1, nodelet::Nodelet);