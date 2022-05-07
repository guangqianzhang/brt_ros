#pragma once
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <numeric>
#include <termios.h>

#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <Eigen/Dense>
#include <ros/package.h>
#include <Eigen/Core>
#include <opencv/cv.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "brt_detection/global_defination/global_defination.h"
#include "dataStructures.h"
#include <opencv2/dnn.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#define POINTXYZ
namespace DetectandTract
{

#ifdef POINTXYZI
    typedef pcl::PointXYZI Point;
    typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
    typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud_P;
#endif
#ifdef POINTXYZ
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud_P;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
#endif

    class projector
    {
    private:
        cv::Mat image_proj;
        image_transport::Publisher image_publisher;
        tf::TransformBroadcaster tr_br;

        std::string __APP_NAME__;
        std::string camera_topic;
        std::string lidar_topic;
        std::string camera_topic_out;
        std::string camera_frame_id;

        std::string medel_path;
        std::string yoloclassfile;
        std::string yoloModelConfiguration;
        std::string yoloModelWeights;
        std::vector<std::string> classes;

        float confThreshold;
        float nmsThreshold; // Non-maximum suppression threshold
        float shrinkFactor;
        bool bVis;

        // crop lidar points
        float maxX = 25.0, maxY = 6.0, minZ = -1.4;

        struct initial_parameters
        {
            /* data */
            cv::Mat camtocam_mat;
            cv::Mat cameraIn;
            cv::Mat RT;
        } i_params;

        struct out_messages
        {
            LidarPoint centroid_;
            Eigen::Vector4cf centroid;

        } out_msgs;
        void projection_callback(const sensor_msgs::Image::ConstPtr &img,
                                 const sensor_msgs::PointCloud2::ConstPtr &pc);
        void initParams();
        void matrix_to_transfrom(Eigen::MatrixXf &matrix, tf::Transform &trans);
        void VoxelGridPoints(PointCloud_P cloud_, float size, PointCloud_P cloud_out);
        void clip_down(double clip_height, const PointCloud_P in, const PointCloud_P cloud_out);

        void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, pcl::PointCloud<pcl::PointXYZ>::Ptr lidarPoints);

        void drawImage(BoundingBox box, cv::Mat &img);
        void drawPoint(cv::Mat &img, cv::Point pt);
        void centroid(LidarPoint &centroid, BoundingBox *box);
        void centroid(Eigen::Vector4cf centroid, PointCloud_P cloud_p);
        void outMessage(DataFrame &frame);

    public:
        projector();
    };

}