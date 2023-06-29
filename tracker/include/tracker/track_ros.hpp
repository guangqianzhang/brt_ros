#ifndef TRACK_ROS_HPP__
#define TRACK_ROS_HPP__
#include "BYTETracker.h"
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
using namespace std;
using namespace cv;

class Track_Ros
{
public:
    Track_Ros(int fps, ros::NodeHandle nh)
    {
         trackerPtr.reset(new BYTETracker(fps, 30));
        std::vector<Object> objects;

        sub = nh.subscribe<autoware_msgs::DetectedObjectArray>("/fusion_node/sync/fused_msg", 10, &Track_Ros::callback, this);

        // ros::AsyncSpinner spinner(1);
        // spinner.start();
        camera_pub = nh.advertise<sensor_msgs::Image>("trackImage", 10);
        chatter_pub = nh.advertise<autoware_msgs::DetectedObjectArray>("chatter", 10);
        ros::spin();
    }

private:
    ros::NodeHandle nh;
    ros::Publisher camera_pub;
    ros::Publisher chatter_pub;
    ros::Subscriber sub;
    boost::shared_ptr<BYTETracker>  trackerPtr;
    cv::Mat raw_img;
    int GetTypeId(std::string label);
    std::string Getlabel(int id);
    void callback(const autoware_msgs::DetectedObjectArray::ConstPtr &msg);
    Scalar get_color(int idx);
    void drawrectangle(cv::Mat &img, vector<float> tlwh, int track_id,float x);
};

int Track_Ros::GetTypeId(std::string label)
{
    if (label == "unknown")
    {
        return 0;
    }
    else if (label == "car")
    {
        return 1;
    }
    else if (label == "truck")
    {
        return 2;
    }
    else if (label == "bike")
    {
        return 3;
    }
    else if (label == "pedestrian")
    {
        return 4;
    }
    else
    {
        return 0;
    }
}

std::string Track_Ros::Getlabel(int id)
{
    std::string label;
    switch (id)
    {
    case 0:
        label = "unknown";
        break;
    case 1:
        label = "car";
        break;
    case 2:
        label = "truck";
        break;
    case 3:
        label = "bike";
        break;
    case 4:
        label = "pedestrian";
        break;
    default:
        label = "unknown";
        break;
    }
    return label;
}

void Track_Ros::callback(const autoware_msgs::DetectedObjectArray::ConstPtr &msg)
{
    //////////////////////////////////////////////////////////
    /////////////////////数据预处理////////////////////////////
    ROS_INFO("Received message: %lu", sizeof(*msg));
        sensor_msgs::Image img =msg->image;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
    }
    catch (cv_bridge::Exception &e)
    {
        cout << "cv_bridge Copy failed!";
        return;
    }
    raw_img = cv_ptr->image;
    std::vector<Object> objects;
    std::vector<autoware_msgs::DetectedObject> objs = msg->objects;
    for (auto obj = objs.begin(); obj != objs.end(); ++obj)
    {
        Object object{};
        object.label=obj->id;
        object.prob = obj->score;

        object.position[0] = obj->pose.position.x;
        object.position[1] = obj->pose.position.y;
        object.position[2] = obj->pose.position.z;
        object.rect.x = obj->x;
        object.rect.y = obj->y;
        object.rect.width = obj->width;
        object.rect.height = obj->height;

        objects.push_back(object);
    }
    ////////////////////////////////////////////////////////////////////
    //////////////////////开始跟踪///////////////////////////////////////
    auto track_start = chrono::system_clock::now();
    vector<STrack> output_stracks = trackerPtr->update(objects);
    auto end = chrono::system_clock::now();
    int tack_ms = chrono::duration_cast<chrono::microseconds>(end - track_start).count();
    cout << "tracker takes (MS): " << (float)tack_ms / 1000 << endl;

    ////////////////////////////////////////////////////////////////
    ///////////////////////数据发送/////////////////////////////////
    autoware_msgs::DetectedObjectArray ObjectArray;
    autoware_msgs::DetectedObject Object;
    for (auto it = output_stracks.begin(); it != output_stracks.end(); ++it)
    {

        Object.x = it->tlwh[0];
        Object.y = it->tlwh[1];
        Object.width = it->tlwh[2];
        Object.height = it->tlwh[3];
        Object.score = it->score;
        // Object.id = it->track_id;
        Object.pose.position.x = it->position[0];
        Object.pose.position.y = it->position[1];
        Object.pose.position.z = it->position[2];
        // Object.label=Getlabel(it->label);
        drawrectangle(raw_img, it->tlwh, it->track_id,float(Object.pose.position.y));
        ObjectArray.objects.push_back(Object);
    }
    chatter_pub.publish(ObjectArray);
                cv_ptr->image = raw_img;
        camera_pub.publish(cv_ptr->toImageMsg());
}


Scalar Track_Ros::get_color(int idx)
{
    idx += 3;
    return Scalar(37 * idx % 255, 17 * idx % 255, 29 * idx % 255);
}
void Track_Ros::drawrectangle(cv::Mat &img, vector<float> tlwh, int track_id,float x)
{
    cv::Scalar s = get_color(track_id);
    putText(img, format("%d y:%.1f", track_id,x), Point(tlwh[0], tlwh[1] - 5),
            0, 0.6, Scalar(0, 0, 255), 2, cv::LINE_AA);
    cv::rectangle(img, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
}
#endif