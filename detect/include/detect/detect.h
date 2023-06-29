#pragma once

#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include "model.h"

#include <iostream>
#include <chrono>
#include <cmath>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <yaml-cpp/yaml.h>
#include <autoware_msgs/ImageObj.h>
#include <ros/package.h>
using namespace std;
using namespace nvinfer1;
typedef struct YoloDetSt
{
    std::string label;
    float confidences;
    // 	 　　int x; /* 方形的左上角的x-坐标 */
    // 　　int y; /* 方形的左上角的y-坐标*/
    // 　　int width; /* 宽 */
    // 　　int height; /* 高 */
    cv::Rect rect;
} YoloDetSt;

class detect
{
public:
    detect(ros::NodeHandle node,
           ros::NodeHandle private_nh,
           std::string const &node_name = ros::this_node::getName());
    ~detect();

    // cv::Mat inference(cv::Mat raw_img, std::vector<YoloDetSt> &yoloRet);
    std::string camera_topic;
    cv::Size image_size_;
    cv::Mat camera_instrinsics_;
    cv::Mat distortion_coefficients_;
    cv::Mat CameraExtrinsicMat_;

private:
    std::string medel_path;
    std::string yoloModelWeights;
    std::string yoloModelConfiguration;
    std::string yoloclassfile;
    bool m_isGpu = false;

    std::string __APP_NAME__;
    std::string camera_topic_out;
    // std::string camera_topic;
    std::string camera_topic_msg;
    std::string camera_topic_info;

    image_transport::Subscriber camera_raw_sub_;
    image_transport::Publisher vision_detect_img_pub_;
    ros::Publisher vision_detect_msg_pub_;
    ros::Publisher vision_camera_pub_;

    std::string wts_name;
    std::string engine_name;
     std::string subpath;
    bool is_p6 = false;
    float gd = 0.0f, gw = 0.0f;

    Logger gLogger;
    // Deserialize the engine from file
    cudaStream_t stream;
    IRuntime *runtime = nullptr;
    ICudaEngine *engine = nullptr;
    IExecutionContext *context = nullptr;

    cv::Mat raw_img;
    
    

      // Prepare cpu and gpu buffers
  float *gpu_buffers[2];
  float *cpu_output_buffer = nullptr;
    const static int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;

    void initParams(ros::NodeHandle private_nh, std::string const &node_name);
    void serialize_engine(unsigned int max_batchsize, bool &is_p6, float &gd, float &gw, std::string &wts_name, std::string &engine_name);
    void deserialize_engine(std::string &engine_name, IRuntime **runtime, ICudaEngine **engine, IExecutionContext **context);
    void prepare_buffers(ICudaEngine *engine, float **gpu_input_buffer, float **gpu_output_buffer, float **cpu_output_buffer);
    int loadModel();

    void image_cb(const sensor_msgs::Image::ConstPtr &img);
    void inference(IExecutionContext& context, cudaStream_t& stream, void** gpu_buffers, float* output, int batchsize) ;
    void do_detect(std::vector<cv::Mat> img_batch, std::vector<std::vector<Detection>> &res_batch);
};
