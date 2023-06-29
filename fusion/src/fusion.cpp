
#include "fusion.h"
#include "fusion_utils.hpp"
subscriberANDpublisher::subscriberANDpublisher(ros::NodeHandle &nh) : nh(nh)
{
    // 订阅话题
    camera_topic_ = "/detect_msg_image";
    lidar_topic_ = "/detection/lidar_detector/objects";
    lidar_sub.subscribe(nh, lidar_topic_, 1);
    camera_sub.subscribe(nh, camera_topic_, 1);
    std::cout << "lidar:" << camera_topic_.c_str() << " "
              << "camera:" << camera_topic_.c_str() << std::endl;

    // 发布者
    camera_pub = nh.advertise<sensor_msgs::Image>("sync/img", 1000);
    lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("sync/lidar", 1000);
    fused_pub = nh.advertise<DetectedObjectArray>("sync/fused_msg", 1000);

    // 回调
    sync_.reset(new Sync(syncpolicy(10), camera_sub, lidar_sub));
    sync_->registerCallback(boost::bind(&subscriberANDpublisher::callback, this, _1, _2));

    nh.param<int>("/vision_camera_node/adjust_x", adjust_x_, 25);
    cout << "adjust_x_:" << adjust_x_ << endl;
    nh.param<int>("/vision_camera_node/adjust_y", adjust_y_, 120);
    cout << "adjust_y_:" << adjust_y_ << endl;
    nh.param<bool>("/DEBUG", DEBUG, false);
    cout << "DEBUG:" << DEBUG << endl;
    nh.param<double>("/overlap_threshold", overlap_threshold_, 0.5);
    cout << "overlap_threshold:" << overlap_threshold_ << endl;
    nh.param<string>("/lidar_camera_calibration", lidar_camera_calibration, "lidar_camera_calibration65.yaml");
    cout << "lidar_camera_calibration:" << lidar_camera_calibration.c_str() << endl;
    string packgpath = ros::package::getPath("fusion");
    int pos = packgpath.find_last_of("/");
    string srcpath = packgpath.substr(0, pos);
    std::string config_file_Camera_path = srcpath + "/cfg/" + lidar_camera_calibration;
    cout << config_file_Camera_path << endl;

    cv::FileStorage config_camera(config_file_Camera_path, cv::FileStorage::READ);
    config_camera["CameraMat"] >> camera_instrinsics_;
    config_camera["CameraExtrinsicMat"] >> CameraExtrinsicMat_;

    cout << "CameraMat:" << camera_instrinsics_ << endl;
    cout << "CameraExtrinsicMat:" << CameraExtrinsicMat_ << endl;
    Projet_ALL = false;
    Projet_Rect = true;

    ros::spin();
}

void subscriberANDpublisher::callback(const ImageObjConstPtr &image,
                                      const DetectedObjectArrayConstPtr &in_range_detections)
{
    ROS_INFO("CALLBACK!");
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
    imageBox.width = raw_img.cols;
    imageBox.height = raw_img.rows;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    DetectedObjectArray objects = *in_range_detections;

    // 过滤非图中点云
    DetectedObjectArray out_in_cv_range_detections;
    DetectedObjectArray out_out_cv_range_detections;
    out_in_cv_range_detections.header = in_range_detections->header;
    out_in_cv_range_detections.objects.clear();
    TransformRangeToVision(in_range_detections, out_in_cv_range_detections, out_out_cv_range_detections);

    DetectedObjectArray fused_objects;

    fused_objects.header = image->header;
    fused_objects.image = img; ////for tack

    const auto image_obj = image->obj;
    // const auto image_labe = image->type;
    cout << "image_obj:" << image_obj.size() << endl;
    
    for (size_t i = 0; i < image_obj.size(); ++i)
    {

        DetectedObject fused_object;
        cv::Rect vision_rect(image_obj[i].x, image_obj[i].y, image_obj[i].width, image_obj[i].height);
        // cout<<"vision rect:"<<vision_rect<<endl;
        fused_object.x = vision_rect.x;
        fused_object.y = vision_rect.y;
        fused_object.width = vision_rect.width;
        fused_object.height = vision_rect.height;

        fused_object.score = image_obj[i].score;
        fused_object.id = image_obj[i].class_id;
        int vision_rect_area = vision_rect.area();
        for (auto it = out_in_cv_range_detections.objects.begin(); it != out_in_cv_range_detections.objects.end(); ++it)
        {
            cv::Rect range_rect = ProjectDetectionToRect(*it);
            int range_rect_area = range_rect.area();
            
            if (range_rect_area < 10)
            {
                cout << "too small box..." << range_rect << endl;
                continue;
            }
            if (it->pose.position.y < 1.5)
            {
                continue;
            }
            if (it->pose.position.x > 9.5)
            {
                continue;
            }
            // if(range_rect_area>(imageBox.area()/5)) continue;
            if (!raw_img.empty())
            {cout << "box:" << range_rect << endl;
                drawrectangle(raw_img, range_rect); // 点云画框
            }
            else
                cout << "empty img!!!" << endl;

            cv::Rect overlap = range_rect & vision_rect;
            int con = range_rect.area() + vision_rect.area() - overlap.area();
            float iou = overlap.area() / con;
            cv::Rect Rect_final = overlap;
            if (iou > overlap_threshold_)
            {
                fused_object.pose.position = it->pose.position;
                fused_object.dimensions = it->dimensions;
                fused_object.velocity = it->velocity;

                Rect_final.x = (range_rect.x < vision_rect.x) ? range_rect.x : vision_rect.x;
                Rect_final.y = (range_rect.y < vision_rect.y) ? range_rect.y : vision_rect.y;
                Rect_final.width = range_rect.width + vision_rect.width - overlap.width;
                Rect_final.height = range_rect.height + vision_rect.height - overlap.height;
                drawrectangleIOU(raw_img, Rect_final, image_obj[i].class_id, iou);
                std::cout << "iou: " << iou << std::endl;
            }
        }
        fused_objects.objects.push_back(fused_object);
    }
    fused_pub.publish(fused_objects);
    cv_ptr->image = raw_img;
    camera_pub.publish(cv_ptr->toImageMsg());
}

// 判断目标是否在图像范围内。先将目标坐标转换到相机坐标系下transformPoint，
// 再将相机坐标系下的位置点转换到图像内projectpoint，如果像素在图像内，则返回true。
bool subscriberANDpublisher::IsObjectInImage(const DetectedObject &in_detection)
{
    // cv::Point3f image_space_point = TransformPoint(in_detection.pose.position, camera_lidar_tf_);
    pcl::PointXYZ position;
    position.x = in_detection.pose.position.x;
    position.y = in_detection.pose.position.y;
    position.z = in_detection.pose.position.z;
    cv::Point2i image_pixel;
    // cout<<"image_pixel:"<<image_pixel<<endl;
    TransformPoint2D(position, image_pixel, camera_instrinsics_, CameraExtrinsicMat_, adjust_x_, adjust_y_);
    return subscriberANDpublisher::imageBox.contains(image_pixel);
}

// 在点云聚类后的目标中，区分图像内的和图像外的，分别存储
void subscriberANDpublisher::TransformRangeToVision(const DetectedObjectArrayConstPtr &in_range_detections,
                                                    DetectedObjectArray &out_in_cv_range_detections,
                                                    DetectedObjectArray &out_out_cv_range_detections)
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
cv::Rect subscriberANDpublisher::ProjectDetectionToRect(DetectedObject &in_detection)
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
