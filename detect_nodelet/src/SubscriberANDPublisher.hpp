#include <ros/ros.h>
//时间同步
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
//传感器消息
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <rockauto_msgs/ImageObj.h>
#include <rockauto_msgs/DetectedObjectArray.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include "fusion_utils.hpp"

using namespace std;
class subscriberANDpublisher
{
public:
    subscriberANDpublisher(std::string lidar_topic, std::string camera_topic);
    void callback(const rockauto_msgs::ImageObjConstPtr &image, const rockauto_msgs::DetectedObjectArrayConstPtr &in_range_detections);
    int adjust_x_;
    int adjust_y_;
    cv::Mat camera_instrinsics_;
    cv::Mat CameraExtrinsicMat_;

private:
    ros::NodeHandle nh;
    ros::Publisher camera_pub;
    ros::Publisher lidar_pub;
    ros::Publisher fused_pub;
    message_filters::Subscriber<rockauto_msgs::DetectedObjectArray> lidar_sub; //雷达订阅
    message_filters::Subscriber<rockauto_msgs::ImageObj> camera_sub;           //相机订阅

    std::string camera_topic_;
    std::string lidar_topic_;

    cv::Rect imageBox;
    cv::Size image_size_;
    double overlap_threshold_;

    bool Projet_ALL;
    bool Projet_Rect;
    typedef message_filters::sync_policies::ApproximateTime<rockauto_msgs::ImageObj, rockauto_msgs::DetectedObjectArray> syncpolicy; //时间戳对齐规则
    typedef message_filters::Synchronizer<syncpolicy> Sync;
    boost::shared_ptr<Sync> sync_; //时间同步器

    bool IsObjectInImage(const rockauto_msgs::DetectedObject &in_detection);
    void TransformRangeToVision(const rockauto_msgs::DetectedObjectArrayConstPtr &in_range_detections,
                                rockauto_msgs::DetectedObjectArray &out_in_cv_range_detections,
                                rockauto_msgs::DetectedObjectArray &out_out_cv_range_detections);
    cv::Rect ProjectDetectionToRect(rockauto_msgs::DetectedObject &in_detection);
};
/* ————————————————
版权声明：本文为CSDN博主「火柴的初心」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/He3he3he/article/details/109643391 */

subscriberANDpublisher::subscriberANDpublisher(std::string lidar_topic, std::string camera_topic) : nh("~")
{
    //订阅话题
    lidar_sub.subscribe(nh, lidar_topic, 1);
    camera_sub.subscribe(nh, camera_topic, 1);
    std::cout << "lidar:" << lidar_topic.c_str() << " "
              << "camera:" << camera_topic.c_str() << std::endl;

    //发布者
    camera_pub = nh.advertise<sensor_msgs::Image>("sync/img", 1000);
    lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("sync/lidar", 1000);
    fused_pub = nh.advertise<rockauto_msgs::DetectedObjectArray>("sync/fused_msg", 1000);

    //回调
    sync_.reset(new Sync(syncpolicy(10), camera_sub, lidar_sub));
    sync_->registerCallback(boost::bind(&subscriberANDpublisher::callback, this, _1, _2));

    imageBox.x = 0;
    imageBox.y = 0;
    imageBox.width = 640;
    imageBox.height = 480;
    image_size_ = cv::Size(imageBox.width, imageBox.height);
    nh.param<int>("/vision_camera_node/adjust_x", adjust_x_, 25);
    cout << "adjust_x_:" << adjust_x_;
    nh.param<int>("/vision_camera_node/adjust_y", adjust_y_, 120);
    cout << "adjust_y_:" << adjust_y_;
    nh.param<double>("/overlap_threshold", overlap_threshold_, 0.5);
    cout << "overlap_threshold:" << overlap_threshold_;

    std::string config_file_Camera_path = "/home/brt/Documents/catktin_brt/src/vision_camera/cfg/lidar_camera_calibration64.yaml";
    // YAML::Node config_camera = YAML::LoadFile(config_file_Camera_path);
    cv::FileStorage config_camera(config_file_Camera_path, cv::FileStorage::READ);
    config_camera["CameraMat"] >> camera_instrinsics_;
    config_camera["CameraExtrinsicMat"] >> CameraExtrinsicMat_;
    Projet_ALL = false;
    Projet_Rect = true;
}
void subscriberANDpublisher::callback(const rockauto_msgs::ImageObjConstPtr &image,
                                      const rockauto_msgs::DetectedObjectArrayConstPtr &in_range_detections)
{

    sensor_msgs::Image img = image->roi_image;
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
    // cv::Mat raw_img = cv_ptr->image;
    cv::Mat raw_img = cv_ptr->image;
    // ros::Time time = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    rockauto_msgs::DetectedObjectArray objects = *in_range_detections;

    cout << "1. LOAD the Lidar and image frame Done!" << endl;

    //过滤非图中点云
    rockauto_msgs::DetectedObjectArray out_in_cv_range_detections;
    rockauto_msgs::DetectedObjectArray out_out_cv_range_detections;
    out_in_cv_range_detections.header = in_range_detections->header;
    out_in_cv_range_detections.objects.clear();
    TransformRangeToVision(in_range_detections, out_in_cv_range_detections, out_out_cv_range_detections);

    // *cloud = Obj2PointCloud(out_in_cv_range_detections);
    rockauto_msgs::DetectedObjectArray fused_objects;
    fused_objects.header = image->header;

    for (auto it = out_in_cv_range_detections.objects.begin(); it != out_in_cv_range_detections.objects.end(); ++it)
    {

        cv::Rect range_rect = ProjectDetectionToRect(*it);
        int range_rect_area = range_rect.area();
        // if(range_rect_area>(imageBox.area()/5)) continue;
        // drawrectangle(raw_img, range_rect);// 点云画框

        // pointers to all bounding boxes
        // for (auto it2 = image->obj.begin(); it2 != image->obj.end(); ++it2)
        const auto image_obj=image->obj;
        const auto image_labe=image->type;
        for (size_t i=0;i<image_obj.size();++i)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect vision_rect;
            vision_rect.x = image_obj[i].x;
            vision_rect.y = image_obj[i].y;
            vision_rect.width = image_obj[i].width;
            vision_rect.height = image_obj[i].height;
            int vision_rect_area = vision_rect.area();
            // cout << smallerBox << endl;
            // cv::rectangle(raw_img, cv::Point(smallerBox.x, smallerBox.y), cv::Point(smallerBox.x+smallerBox.width,smallerBox.y+smallerBox.height), cv::Scalar(0, 255, 0), 2);
            // check wether point is within current bounding box
            rockauto_msgs::DetectedObject fused_object;
            //交并集
            cv::Rect overlap = range_rect & vision_rect;
            cv::Rect Rect_final = overlap;
            double iou = 0.0;
            if ((overlap.area() > range_rect_area * overlap_threshold_) || (overlap.area() > vision_rect_area * overlap_threshold_))
            {
                fused_object.valid = it->valid;
                fused_object.pose.position = it->pose.position;
                fused_object.dimensions = it->dimensions;
                fused_object.velocity=it->velocity;
                Rect_final.x = (range_rect.x < vision_rect.x) ? range_rect.x : vision_rect.x;
                Rect_final.y = (range_rect.y < vision_rect.y) ? range_rect.y : vision_rect.y;
                Rect_final.width = range_rect.width + vision_rect.width - overlap.width;
                Rect_final.height = range_rect.height + vision_rect.height - overlap.height;

                iou=(double)overlap.area()/vision_rect_area;
                drawrectangleIOU(raw_img, Rect_final, image_labe[i],iou);

                fused_objects.objects.push_back(fused_object);
            }

        } // eof loop over all bounding boxes

    } // end of objs

    // std::cout << imageBox << std::endl;
    // cv::rectangle(raw_img, cv::Point(0, 0), cv::Point(imageBox.width, imageBox.height), cv::Scalar(0, 255, 0), 2);
    cv_ptr->image = raw_img;
    camera_pub.publish(cv_ptr->toImageMsg());
    fused_pub.publish(fused_objects);
    // camera_pub.publish(img);
    // lidar_pub.publish(pointcloud);
    ROS_INFO("done! ");
}

//判断目标是否在图像范围内。先将目标坐标转换到相机坐标系下transformPoint，
//再将相机坐标系下的位置点转换到图像内projectpoint，如果像素在图像内，则返回true。
bool subscriberANDpublisher::IsObjectInImage(const rockauto_msgs::DetectedObject &in_detection)
{
    // cv::Point3f image_space_point = TransformPoint(in_detection.pose.position, camera_lidar_tf_);
    pcl::PointXYZ position;
    position.x = in_detection.pose.position.x;
    position.y = in_detection.pose.position.y;
    position.z = in_detection.pose.position.z;
    cv::Point2i image_pixel;
    TransformPoint2D(position, image_pixel, camera_instrinsics_, CameraExtrinsicMat_, adjust_x_, adjust_y_);
    return subscriberANDpublisher::imageBox.contains(image_pixel);
}

//在点云聚类后的目标中，区分图像内的和图像外的，分别存储
void subscriberANDpublisher::TransformRangeToVision(const rockauto_msgs::DetectedObjectArrayConstPtr &in_range_detections,
                                                    rockauto_msgs::DetectedObjectArray &out_in_cv_range_detections,
                                                    rockauto_msgs::DetectedObjectArray &out_out_cv_range_detections)
{
    out_in_cv_range_detections.header = in_range_detections->header;
    out_in_cv_range_detections.objects.clear();
    out_out_cv_range_detections.header = in_range_detections->header;
    out_out_cv_range_detections.objects.clear();
    for (auto it = in_range_detections->objects.begin(); it != in_range_detections->objects.end(); it++)
    {
        if (IsObjectInImage(*it))
        {
            out_in_cv_range_detections.objects.push_back(*it);
        }
        else
        {
            out_out_cv_range_detections.objects.push_back(*it);
        }
    }
}
cv::Rect subscriberANDpublisher::ProjectDetectionToRect(rockauto_msgs::DetectedObject &in_detection)
{
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = Obj2PointCloud(in_detection);
    std::vector<cv::Point> points; // vector容器存放点坐标

    for (auto it1 = cloud->points.begin(); it1 != cloud->points.end(); ++it1)
    {

        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;
        Y = CV_Transformlidar(X, camera_instrinsics_, CameraExtrinsicMat_);

        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);
        pt.x = pt.x + adjust_x_;
        pt.y = pt.y + adjust_y_;

        if (Projet_ALL)
        {
            Projet_Rect = false;

            if (imageBox.contains(pt))
            {
                // points.push_back(pt);
                // drawPoint(raw_img, pt);
            }
        }
        if (Projet_Rect)
        {
            Projet_ALL = false;
            if (imageBox.contains(pt))
            {
                points.push_back(pt);
            }
        }
    } // end of clouds
      // drawMinRect(raw_img, points);//此函数报错为解决
    cv::Rect range_rect = boundingRect(points);
    // int range_rect_area = range_rect.area();
    // drawrectangle(raw_img, range_rect);
    return range_rect;
}
