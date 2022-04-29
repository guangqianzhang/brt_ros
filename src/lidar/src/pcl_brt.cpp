//
// Created by zgq on 2022/4/22.
//

#include "lidar/pcl_brt.h"

pcl_brt::pcl_brt(ros::NodeHandle &nh) : nh_(nh)
{

    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("brt_points", 1);
    
    // pcl_sub_ = nh_.subscribe<POINT>("brt_lidar", 1000, &pcl_brt::cb, this);

}
pcl_brt::~pcl_brt() {}
void pcl_brt::readPcdFile()
{
    static int pcdIndex= 0;
    PointCloud_P cloud(new PointCloud);
    std::ostringstream pcdNumber;
    pcdNumber << pcdStartIndex + pcdIndex;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(DATA_ROOT_PATH + PCD_PATH + pcdNumber.str() + FILE_TYPE, *cloud) == -1)
    {
        PCL_ERROR("could not read the file\n");
        
    }else
    std::cout << " get the cloud file\n";
    cloud_p_ = cloud;
    pcdIndex++;
    pcdIndex%=145;
    
}
void pcl_brt::pub_points()
{

    pcl::toROSMsg(*cloud_p_, msg_);
    msg_.header.stamp = ros::Time::now();
    msg_.header.frame_id = "brt_frame";

    pcl_pub_.publish(msg_);
}
void pcl_brt::Run(){
    readPcdFile();
     
    // cropLidarPoints(cloud_p_,minX, maxX, maxY, minZ, maxZ, minR);
    pub_points();

}
void pcl_brt::cb(const PointCloud::ConstPtr &cloud)
{
}
// remove Lidar points based on min. and max distance in X, Y and Z
void pcl_brt::cropLidarPoints(PointCloud_P cloud, float minX, float maxX, float maxY, float minZ, float maxZ, float minR)
{
    // // std::vector<LidarPoint> newLidarPts;
    // for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
    // {

    //     if ((*it).x >= minX && (*it).x <= maxX && (*it).z >= minZ && (*it).z <= maxZ && (*it).z <= 0.0 && abs((*it).y) <= maxY && (*it).r >= minR) // Check if Lidar point is outside of boundaries
    //     {
    //         // newLidarPts.push_back(*it);
    //     }else{
    //         lidarPoints.erase(it);
    //     }

    // }
    std::cout<<"star removing points outside "<<std::endl;

  



    // lidarPoints = newLidarPts;
}
void pcl_brt::CROP_Points()
{
    //得到感兴趣区域的点云
    // load 3D Lidar points from file

    std::vector<LidarPoint> lidarPoints;
    // remove Lidar points based on distance properties
    float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
    // cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);

    (dataBuffer_->end() - 1)->lidarPoints = lidarPoints;
    std::cout << "#3 : CROP LIDAR POINTS done" << std::endl;
}