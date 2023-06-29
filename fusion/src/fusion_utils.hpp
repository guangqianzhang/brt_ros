#ifndef FUSION_UTILS_HPP_
#define FUSION_UTILS_HPP_
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <autoware_msgs/DetectedObjectArray.h>

using namespace cv;
Eigen::VectorXf TransformVision(Eigen::VectorXf in_points, const Eigen::MatrixXd Mat)
{
    // cv::Mat camera_instrinsics_inver=camera_instrinsics_.inv();

    Eigen::VectorXf out_point;
    // must match the date type !!
    // out_point = Mat.inverse().cast<float>() * in_points;
    auto size = Mat.size();
    if (size == 9)
    {

        Eigen::Vector3d vec;
        Eigen::MatrixXd mat;
        vec << 0,
            0,
            0;
        mat << Mat, vec;
        std::cout << vec.cols() << " " << vec.rows() << std::endl;
        std::cout << mat.cols() << " " << mat.rows() << std::endl;
        std::cout << "points_in31:" << in_points << std::endl;
        std::cout << "point:" << in_points << std::endl;

        // out_point = mat.inverse().cast<float>() * in_points;
    }
    else if (size == 16)
    {
        std::cout << "points_in41:" << in_points << std::endl;
        Eigen::Vector4f point4;
        point4 = in_points;
        std::cout << "point:" << point4 << std::endl;
        // out_point = Mat.inverse().cast<float>() * point4;
    }
    return out_point;
}
Eigen::Vector3f Transformlidar(const Eigen::Vector4f in_points, const Eigen::MatrixXd MatC, const Eigen::MatrixXd MatE)
{
    Eigen::Vector3f out_point;
    Eigen::Vector3d vec(0, 0, 0);
    Eigen::Matrix<double, 3, 4, Eigen::RowMajor> matc;
    matc << MatC, vec;
    std::cout << matc.cols() << std::endl;
    // must match the date type !!
    out_point = matc.cast<float>() * MatE.cast<float>() * in_points;
    return out_point;
}

cv::Mat CV_Transformlidar(cv::Mat x, cv::Mat camera_instrinsics_, cv::Mat CameraExtrinsicMat_)
{

    cv::Mat Y(3, 1, cv::DataType<double>::type);
    cv::Mat mDst = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
    cv::Mat mdst;
    cv::hconcat(camera_instrinsics_, mDst, mdst);
    // std::cout<<mdst<<std::endl;
    cv::Mat CameraExtrinsicMat;
    bool ret=cv::invert(CameraExtrinsicMat_,CameraExtrinsicMat);
    Y = mdst * CameraExtrinsicMat * x;
    return Y;
}
void drawPoint(cv::Mat &img, cv::Point pt)
{
    float val = pt.x;
    float maxVal = 20.0;
    int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
    int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
    cv::circle(img, pt, 1, cv::Scalar(red, green, 0), -1);
}
pcl::PointCloud<pcl::PointXYZ> Obj2PointCloud(autoware_msgs::DetectedObject &objects)
{

    pcl::PointCloud<pcl::PointXYZ> object_cloud;
    pcl::fromROSMsg(objects.pointcloud, object_cloud);
    return object_cloud;
}
pcl::PointCloud<pcl::PointXYZRGB> Obj2PointCloud(autoware_msgs::DetectedObjectArray &objects_array)
{
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
    for (size_t object_i = 0; object_i < objects_array.objects.size(); object_i++)
    {
        // std::cout << "objct i" << object_i << std::endl;
        pcl::PointCloud<pcl::PointXYZI> object_cloud;
        pcl::fromROSMsg(objects_array.objects[object_i].pointcloud, object_cloud);
        int red = (object_i) % 256;
        int green = (object_i * 7) % 256;
        int blue = (object_i * 13) % 256;

        for (size_t i = 0; i < object_cloud.size(); i++)
        {
            // std::cout << "point i" << i << "/ size: "<<object_cloud.size()  << std::endl;
            pcl::PointXYZRGB colored_point;
            colored_point.x = object_cloud[i].x;
            colored_point.y = object_cloud[i].y;
            colored_point.z = object_cloud[i].z;
            colored_point.r = red;
            colored_point.g = green;
            colored_point.b = blue;
            colored_cloud.push_back(colored_point);
        }
    }
    // sensor_msgs::PointCloud2 output_colored_cloud;
    // pcl::toROSMsg(colored_cloud, output_colored_cloud);
    //   output_colored_cloud.header = message_header_;
    return colored_cloud;
}
void TransformPoint2D(pcl::PointXYZ point3, cv::Point point2, cv::Mat camera_instrinsics_, cv::Mat CameraExtrinsicMat_, int adjust_x_, int adjust_y_)
{
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);
    X.at<double>(0, 0) = point3.x;
    X.at<double>(1, 0) = point3.y;
    X.at<double>(2, 0) = point3.z;
    X.at<double>(3, 0) = 1;
    Y = CV_Transformlidar(X, camera_instrinsics_, CameraExtrinsicMat_);
    point2.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
    point2.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);
    point2.x = point2.x + adjust_x_;
    point2.y = point2.y + adjust_y_;
}
void drawMinRect(cv::Mat &image, std::vector<cv::Point> points)
{
    cv::RotatedRect minRect = minAreaRect(cv::Mat(points));
    cv::Point2f vertex[4];  //用于存放最小矩形的四个顶点
    minRect.points(vertex); //返回矩形的四个顶点给vertex
    //绘制最小面积包围矩形
    for (int i = 0; i < 4; i++)
        line(image, vertex[i], vertex[(i + 1) % 4], cv::Scalar(255, 0, 0), 1, 8); //非常巧妙的表达式
}
void drawrectangle(cv::Mat &img, cv::Rect rect)
{

    cv::rectangle(img, rect, cv::Scalar(0,  0,  255), 1);
}
void drawrectangleIOU(cv::Mat &img, cv::Rect rect,int id, double iou)
{
    cv::rectangle(img, rect, cv::Scalar(0, 255, 0), 2);
    std::string labels = " IOU:" + cv::format("%.2f", iou);
    labels = cv::format("%.d", id) + labels;
    int baseLine;
    cv::Size labelSize = cv::getTextSize(labels, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    int top = cv::max(rect.y, labelSize.height);
    rectangle(img, cv::Point(rect.x, top - labelSize.height),
              cv::Point(rect.x + labelSize.width, top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
    putText(img, labels, cv::Point(rect.x, top), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar());
}

#endif