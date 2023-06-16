#ifndef FUSION_UTILS_HPP_
#define FUSION_UTILS_HPP_
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

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
    Eigen::Vector3d vec(0,0,0);
    Eigen::Matrix<double, 3, 4, Eigen::RowMajor> matc;
    matc << MatC, vec;
    std::cout<<matc.cols()<<std::endl;
    // must match the date type !!
    out_point = matc.cast<float>() * MatE.cast<float>() * in_points;
    return out_point;
}

cv::Mat  CV_Transformlidar(cv::Mat x,cv::Mat camera_instrinsics_,cv::Mat CameraExtrinsicMat_){
    
    cv::Mat Y(3, 1, cv::DataType<double>::type);
    cv::Mat mDst=cv::Mat::zeros(3,1,cv::DataType<double>::type);
    cv::Mat mdst;
    cv::hconcat(camera_instrinsics_,mDst,mdst);
    // std::cout<<mdst<<std::endl;
    Y= mdst*CameraExtrinsicMat_*x;
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
#endif