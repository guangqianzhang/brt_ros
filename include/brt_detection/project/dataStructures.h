#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
struct LidarPoint
{                      // single lidar point in space
    double x, y, z, r; // x,y,z in [m], r is point reflectivity
};
struct out_messages
{
};
struct BoundingBox
{ // bounding box around a classified object (contains both 2D and 3D data)

    int boxID;   // unique identifier for this bounding box
    int trackID; // unique identifier for the track to which this bounding box belongs

    cv::Rect roi;      // 2D region-of-interest in image coordinates
    int classID;       // ID based on class file provided to YOLO framework
    double confidence; // classification trust
    
    LidarPoint centroid;
    float distance; // distance from

    std::vector<LidarPoint> lidarPoints; // Lidar 3D points which project into 2D image roi
};
struct DataFrame
{ // represents the available sensor information at the same time instance
    // std::vector<out_messages> frame_out;
    cv::Mat cameraImg;                      // camera image
    std::vector<BoundingBox> boundingBoxes; // ROI around detected objects in 2D image coordinates
};
#endif /* dataStructures_h */
