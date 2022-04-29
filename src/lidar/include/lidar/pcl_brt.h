//
// Created by zgq on 2022/4/22.
//

#ifndef PCL_BRT_PCL_BRT_H
#define PCL_BRT_PCL_BRT_H

#include "ros/ros.h"
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "datastruct.h"

// #include <thread>
// #include <mutex>
// #include <atomic>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud_P;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class pcl_brt
{
private:
    std::string DATA_ROOT_PATH = "/home/zgq/Documents/dataset/2011_09_26";
    std::string PCD_PATH = "/pcd/key_frame_";
    std::string FILE_TYPE = ".pcd";
    int pcdStartIndex = 0;

    ros::NodeHandle nh_;
    ros::Publisher pcl_pub_;
    ros::Subscriber pcl_sub_;

    PointCloud cloud_;
    PointCloud_P cloud_p_;
    sensor_msgs::PointCloud2 msg_;
    sensor_msgs::PointCloud2::Ptr pcl_msg_p_;
    std::vector<DataFrame> *dataBuffer_;

    float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane

    void cb(const PointCloud::ConstPtr &cloud);

public:
    pcl_brt(ros::NodeHandle &nh);
    ~pcl_brt();
    void readPcdFile();
    void pub_points();
    void Run();
    void CROP_Points();

    void cropLidarPoints(PointCloud_P cloud_, float minX, float maxX, float maxY, float minZ, float maxZ,
                         float minR);
};

#endif // PCL_BRT_PCL_BRT_H
