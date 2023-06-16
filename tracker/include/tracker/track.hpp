#ifndef __TRACK_ROS_HPP__
#define __TRACK_ROS_HPP__
#include "tracker/BYTETracker.h"
#include <rockauto_msgs/DetectedObjectArray.h>
#include <rockauto_msgs/DetectedObject.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
class track_ros
{
public:

ros::Subscriber sub = nh.subscribe<rockauto_msgs::DetectedObjectArray>("/vision_nodelet_manager/sync/fused_msg", 10,
                                                                                                                                                                     boost::bind(callback, _1, &objects,camera_pub),this);  
private:
    ros::NodeHandle nh;
    ros::Publisher camera_pub;

    ros::Publisher  chatter_pub;
        int fps = 30;

track_ros::track_ros(){
        BYTETracker tracker(fps, 30);
    std::vector<Object> objects;
          ros::Publisher camera_pub=nh.advertise<sensor_msgs::Image>("trackImage", 10);
    ros::Publisher chatter_pub = nh.advertise<rockauto_msgs::DetectedObjectArray>("chatter", 10);

    int GetTypeId(std::string label)
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

std::string Getlabel(int id)
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
void drawrectangle(cv::Mat &img, vector<float> tlwh, int track_id)
{
    cv::Scalar s = get_color(track_id);
    putText(img, format("%d", track_id), Point(tlwh[0], tlwh[1] - 5),
            0, 0.6, Scalar(0, 0, 255), 2, cv::LINE_AA);
    cv::rectangle(img, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
}


Scalar get_color(int idx)
{
	idx += 3;
	return Scalar(37 * idx % 255, 17 * idx % 255, 29 * idx % 255);
}

}
    

void track_ros::callback(const rockauto_msgs::DetectedObjectArray::ConstPtr &msg,
              std::vector<Object> *objects ,ros::Publisher camera_pub)
{
    ROS_INFO("Received message: %lu", sizeof(*msg));
    cv_bridge::CvImagePtr cv_ptr;
        sensor_msgs::Image  img =msg->image;
  try
    {
        cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
    }
    catch (cv_bridge::Exception &e)
    {
        cout << "cv_bridge Copy failed!";
        // return;
    }
    cv::Mat raw_img = cv_ptr->image;

    std::vector<rockauto_msgs::DetectedObject> objs = msg->objects;
    for (auto obj = objs.begin(); obj != objs.end(); ++obj)
    {
        Object object{};
        /*    //     cv::Rect_<float> rect;
           // int label;
           // float prob; */
        object.label = GetTypeId(obj->label);
        object.prob = obj->score;

        object.position[0] = obj->pose.position.x;
        object.position[1] = obj->pose.position.y;
        object.position[2] = obj->pose.position.z;
        object.rect.x = obj->x;
        object.rect.y = obj->y;
        object.rect.width = obj->width;
        object.rect.height = obj->height;

        objects->push_back(object);
    }

        auto track_start = chrono::system_clock::now();
        vector<STrack> output_stracks = tracker.update(objects);
        auto end = chrono::system_clock::now();
        int tack_ms = chrono::duration_cast<chrono::microseconds>(end - track_start).count();
        cout << "tracker takes (MS): " << (float)tack_ms / 1000 << endl;
        rockauto_msgs::DetectedObjectArray ObjectArray;
        rockauto_msgs::DetectedObject Object;
        for (auto it = output_stracks.begin(); it != output_stracks.end(); ++it)
        {

            Object.x = it->tlwh[0];
            Object.y = it->tlwh[1];
            Object.width = it->tlwh[2];
            Object.height = it->tlwh[3];
            Object.score = it->score;
            Object.id = it->track_id;
            Object.pose.position.x = it->position[0];
            Object.pose.position.y = it->position[1];
            Object.pose.position.z = it->position[2];
        //    Object.label=Getlabel(it->label);
            drawrectangle(raw_img, it->tlwh, it->track_id);
            ObjectArray.objects.push_back(Object);
        }
            cv_ptr->image = raw_img;
        camera_pub.publish(cv_ptr->toImageMsg());
}
};


#endif
