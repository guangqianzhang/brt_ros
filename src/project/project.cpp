#include "brt_detection/project/project.hpp"
#include <yaml-cpp/yaml.h>
#include "glog/logging.h"
#include "brt_detection/project/detection.hpp"

namespace DetectandTract
{

    void projector::initParams()
    {

        __APP_NAME__ = ros::this_node::getName();
        ros::NodeHandle private_node_handle("~");
        private_node_handle.param<std::string>("camera_topic", camera_topic, "/kitti/camera_gray_left/image_raw");
        LOG(INFO) << __APP_NAME__ << "camera_topic:" << camera_topic.c_str();

        private_node_handle.param<std::string>("lidar_topic", lidar_topic, "/kitti/velo/pointcloud");
        LOG(INFO) << __APP_NAME__ << "lidar_topic:" << lidar_topic.c_str();

        private_node_handle.param<std::string>("camera_frame_id", camera_frame_id, "/camera_main");
        LOG(INFO) << __APP_NAME__ << "camera_frame_id:" << camera_frame_id.c_str();

        private_node_handle.param<std::string>("camera_topic_out", camera_topic_out, "/camera_main");
        LOG(INFO) << __APP_NAME__ << "camera_topic_out:" << camera_topic_out.c_str();
        // modelconfig
        std::string config_file_path = WORK_SPACE_PATH + "/cfg/modelconfig.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);
        medel_path = config["medel_path"].as<std::string>();
        yoloclassfile = medel_path + config["yoloclassfile"].as<std::string>();
        yoloModelConfiguration = medel_path + config["yoloModelConfiguration"].as<std::string>();
        yoloModelWeights = medel_path + config["yoloModelWeights"].as<std::string>();
        LOG(INFO) << __APP_NAME__ << "medel_path:" << medel_path.c_str();
        LOG(INFO) << __APP_NAME__ << "yoloclassfile:" << yoloclassfile.c_str();
        LOG(INFO) << __APP_NAME__ << "yoloModelConfiguration:" << yoloModelConfiguration.c_str();
        LOG(INFO) << __APP_NAME__ << "yoloModelWeights:" << yoloModelWeights.c_str();

        confThreshold = config["confThreshold"].as<float>();
        nmsThreshold = config["nmsThreshold"].as<float>();
        bVis = config["bVis"].as<bool>();
        std::string pkg_loc = ros::package::getPath("brt_detection");
        std::ifstream infile(pkg_loc + "/cfg/initial_params.txt");

        double_t camtocam[12];
        double_t cameraIn[16];
        double_t RT[16];
        for (int i = 0; i < 16; i++)
        {
            infile >> camtocam[i];
        }
        cv::Mat(4, 4, 6, &camtocam).copyTo(i_params.camtocam_mat); // cameratocamera params
        if (!i_params.camtocam_mat.data)
        {
            LOG(INFO) << "Don not get the camtocam_mat!";
            exit(1);
        }
        else
        {
            LOG(INFO) << "get the camtocam_mat";
        }

        for (int i = 0; i < 12; i++)
        {
            infile >> cameraIn[i];
        }
        cv::Mat(4, 4, 6, &cameraIn).copyTo(i_params.cameraIn); // cameraIn params
        if (!i_params.cameraIn.data)
        {
            LOG(INFO) << "Don not get the cameraIn!";
            exit(1);
        }
        else
        {
            LOG(INFO) << "get the cameraIn";
        }
        for (int i = 0; i < 16; i++)
        {
            infile >> RT[i];
        }
        cv::Mat(4, 4, 6, &RT).copyTo(i_params.RT); // lidar to camera params
        if (!i_params.camtocam_mat.data)
        {
            LOG(INFO) << "Don not get the RT!";
            exit(1);
        }
        else
        {
            LOG(INFO) << "get the RT";
        }
    }
    void projector::drawImage(BoundingBox box, cv::Mat &img, std::vector<std::string> classes)
    {

        int top, left, width, height;
        top = box.roi.y;
        left = box.roi.x;
        width = box.roi.width;
        height = box.roi.height;
        cv::rectangle(img, cv::Point(left, top), cv::Point(left + width, top + height), cv::Scalar(0, 255, 0), 2);

        std::string label = cv::format("%.2f", box.confidence);
        label = classes[(box.classID)] + ":" + label;

        // Display label at the top of the bounding box
        int baseLine;
        cv::Size labelSize = getTextSize(label, cv::FONT_ITALIC, 0.5, 1, &baseLine);
        top = std::max(top, labelSize.height);
        rectangle(img, cv::Point(left, top - round(1.5 * labelSize.height)), cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
        cv::putText(img, label, cv::Point(left, top), cv::FONT_ITALIC, 0.75, cv::Scalar(0, 0, 0), 1);
    }
    void projector::projection_callback(const sensor_msgs::Image::ConstPtr &img,
                                        const sensor_msgs::PointCloud2::ConstPtr &pc)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            LOG(INFO) << "cv_bridge Copy failed!";
            return;
        }
        cv::Mat raw_img = cv_ptr->image;
        DataFrame frame;
        frame.cameraImg = raw_img;
        std::vector<std::string> classes;
        detectObjects(frame.cameraImg, frame.boundingBoxes, classes, confThreshold, nmsThreshold, yoloclassfile,
                      yoloModelConfiguration, yoloModelWeights, bVis);

        for (auto it = frame.boundingBoxes.begin(); it != frame.boundingBoxes.end(); ++it)
        {

            drawImage(*it, frame.cameraImg, classes);
        }
        // show("detect_after", frame.cameraImg);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pc, *cloud);
        // VoxelGridPoints(cloud, 0.2, cloud);
        //      clip_down();

        cv::Mat visImg = frame.cameraImg.clone(); //不明白为什么要复制
        // cv::Mat overlay = visImg.clone();
        std::cout << "get pc and image data" << std::endl;

        cv::Mat X(4, 1, cv::DataType<double>::type);
        cv::Mat Y(3, 1, cv::DataType<double>::type);
        for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); it++)
        {
            if (it->x > maxX || it->x < 0.0 || abs(it->y) > maxY || it->z < minZ)
            {
                continue;
            }

            X.at<double>(0, 0) = it->x;
            X.at<double>(1, 0) = it->y;
            X.at<double>(2, 0) = it->z;
            X.at<double>(3, 0) = 1;

            Y = i_params.cameraIn * i_params.camtocam_mat * i_params.RT * X; // tranform the point to the camera coordinate

            cv::Point pt;
            pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);
            pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

            float val = it->x;
            float maxVal = 20.0;
            int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
            int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
            cv::circle(visImg, pt, 1, cv::Scalar(0, 0, 0), -1);
        }

        // Publish the image projection
        ros::Time time = ros::Time::now();
        cv_ptr->encoding = "bgr8";
        cv_ptr->header.stamp = time;
        cv_ptr->header.frame_id = camera_frame_id;
        cv_ptr->image = visImg;
        image_publisher.publish(cv_ptr->toImageMsg());
        std::cout << "project picture is published!" << std::endl;
    }

    void projector::matrix_to_transfrom(Eigen::MatrixXf &matrix, tf::Transform &trans)
    {
    }
    /* Voxel Grid Filter将输入点云使用0.2m*0.2m*0.2m的立方体进行分割，
        使用小立方体的 形心（centroid） 来表示这个立方体的所有点，保留这些点作为降采样的输出。 */
    void projector::VoxelGridPoints(PointCloud_P cloud, float size, PointCloud_P cloud_out)
    {
        PointCloud_P filtered_pc_ptr(new PointCloud);
        pcl::VoxelGrid<Point> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(size, size, size);
        vg.filter(*filtered_pc_ptr);
        filtered_pc_ptr->header = cloud->header;
        cloud_out = filtered_pc_ptr;
    }
    /* 要分割地面和非地面，那么过di的区域首先就可以忽略不计 */
    void projector::clip_down(double clip_height, const PointCloud_P in, const PointCloud_P cloud_out)
    {

        pcl::ExtractIndices<Point> cliper;
        cliper.setInputCloud(in);
        pcl::PointIndices indices;
        float minz, maxz;
#pragma omp for
        for (size_t i = 0; i < in->points.size(); i++)
        {
            if (minz > in->points[i].z)
            {
                minz = in->points[i].z;
            };
            if (maxz < in->points[i].z)
            {
                maxz = in->points[i].z;
            }

            if (in->points[i].z < clip_height)
            {

                indices.indices.push_back(i);
            }
        }
        cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
        cliper.setNegative(true); // ture to remove the indices
        cliper.filter(*cloud_out);
        std::cout << "minz=" << minz << " "
                  << "maxz=" << maxz << std::endl;
    }
    projector::projector()
    {
        ros::NodeHandle nh("~");
        initParams();

        ros::Publisher project_img_pub;

        message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, camera_topic, 5);
        message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, lidar_topic, 5);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image_sub, pcl_sub);
        sync.registerCallback(boost::bind(&projector::projection_callback, this, _1, _2));

        image_transport::ImageTransport imageTransport(nh);
        image_publisher = imageTransport.advertise(camera_topic_out, 20);

        ros::spin();
    }

}