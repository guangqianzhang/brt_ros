#include "detect/detect.h"

using namespace autoware_msgs;
using namespace cv;
using namespace std;
using namespace nvinfer1;

void detect::inference(IExecutionContext &context, cudaStream_t &stream, void **gpu_buffers, float *output, int batchsize)
{
  context.enqueue(batchsize, gpu_buffers, stream, nullptr);
  CUDA_CHECK(cudaMemcpyAsync(output, gpu_buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost, stream));
  cudaStreamSynchronize(stream);
}


void detect::do_detect(std::vector<cv::Mat> img_batch, std::vector<std::vector<Detection>> &res_batch)
{
  // Preprocess
  cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

  // Run inference
  auto start = std::chrono::system_clock::now();
  inference(*context, stream, (void **)gpu_buffers, cpu_output_buffer, kBatchSize);
  auto end = std::chrono::system_clock::now();

  // std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

  // NMS
  // std::vector<std::vector<Detection>> res_batch;
  batch_nms(res_batch, cpu_output_buffer, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);
  auto end_nms = std::chrono::system_clock::now();

  auto expendtime = std::chrono::duration_cast<std::chrono::milliseconds>(end_nms - start).count();
}
detect::detect(ros::NodeHandle node,
               ros::NodeHandle private_nh,
               std::string const &node_name)
{
  initParams(private_nh, node_name);
  image_transport::ImageTransport it(private_nh);
  camera_raw_sub_ = it.subscribe(camera_topic, 1, &detect::image_cb, this);
  vision_detect_img_pub_ = it.advertise(camera_topic_out, 1);
  vision_detect_msg_pub_ = private_nh.advertise<ImageObj>(camera_topic_msg, 1);
  // vision_camera_info_pub_=nh.advertise<sensor_msgs::CameraInfo>(camera_topic_info,1);

  loadModel();
  ros::spin();
}

detect::~detect()
{
  // Release stream and buffers
  cudaStreamDestroy(stream);
  CUDA_CHECK(cudaFree(gpu_buffers[0]));
  CUDA_CHECK(cudaFree(gpu_buffers[1]));
  delete[] cpu_output_buffer;
  cuda_preprocess_destroy();
  // Destroy the engine
  context->destroy();
  engine->destroy();
  runtime->destroy();
}

void detect::serialize_engine(unsigned int max_batchsize, bool &is_p6, float &gd, float &gw, std::string &wts_name, std::string &engine_name)
{
  // Create builder
  IBuilder *builder = createInferBuilder(gLogger);
  IBuilderConfig *config = builder->createBuilderConfig();

  // Create model to populate the network, then set the outputs and create an engine
  ICudaEngine *engine = nullptr;
  if (is_p6)
  {
    engine = build_det_p6_engine(max_batchsize, builder, config, nvinfer1::DataType::kFLOAT, gd, gw, wts_name);
  }
  else
  {
    engine = build_det_engine(max_batchsize, builder, config, nvinfer1::DataType::kFLOAT, gd, gw, wts_name);
  }
  assert(engine != nullptr);

  // Serialize the engine
  IHostMemory *serialized_engine = engine->serialize();
  assert(serialized_engine != nullptr);

  // Save engine to file
  std::ofstream p(engine_name, std::ios::binary);
  if (!p)
  {
    std::cerr << "Could not open plan output file" << std::endl;
    assert(false);
  }
  p.write(reinterpret_cast<const char *>(serialized_engine->data()), serialized_engine->size());

  // Close everything down
  engine->destroy();
  builder->destroy();
  config->destroy();
  serialized_engine->destroy();
}
void detect::deserialize_engine(std::string &engine_name, IRuntime **runtime, ICudaEngine **engine, IExecutionContext **context)
{
  std::ifstream file(engine_name, std::ios::binary);
  if (!file.good())
  {
    std::cerr << "read " << engine_name << " error!" << std::endl;
    assert(false);
  }
  size_t size = 0;
  file.seekg(0, file.end);
  size = file.tellg();
  file.seekg(0, file.beg);
  char *serialized_engine = new char[size];
  assert(serialized_engine);
  file.read(serialized_engine, size);
  file.close();

  *runtime = createInferRuntime(gLogger);
  assert(*runtime);
  *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
  assert(*engine);
  *context = (*engine)->createExecutionContext();
  assert(*context);
  delete[] serialized_engine;
}
void detect::prepare_buffers(ICudaEngine *engine, float **gpu_input_buffer, float **gpu_output_buffer, float **cpu_output_buffer)
{
  assert(engine->getNbBindings() == 2);
  // In order to bind the buffers, we need to know the names of the input and output tensors.
  // Note that indices are guaranteed to be less than IEngine::getNbBindings()
  const int inputIndex = engine->getBindingIndex(kInputTensorName);
  const int outputIndex = engine->getBindingIndex(kOutputTensorName);
  assert(inputIndex == 0);
  assert(outputIndex == 1);
  // Create GPU buffers on device
  CUDA_CHECK(cudaMalloc((void **)gpu_input_buffer, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
  CUDA_CHECK(cudaMalloc((void **)gpu_output_buffer, kBatchSize * kOutputSize * sizeof(float)));

  *cpu_output_buffer = new float[kBatchSize * kOutputSize];
}
int detect::loadModel()
{
  cudaSetDevice(0);
  cout<<"wts_name:"<<wts_name<<endl;
  if (!wts_name.empty())
  {
    serialize_engine(kBatchSize, is_p6, gd, gw, wts_name, engine_name);
    return 0;
  }
  
  // Deserialize the engine from file
cout<<"engine_name:"<<engine_name<<endl;
  deserialize_engine(engine_name, &runtime, &engine, &context);

  CUDA_CHECK(cudaStreamCreate(&stream));

  // Init CUDA preprocessing
  cuda_preprocess_init(kMaxInputImageSize);

  // Prepare cpu and gpu buffers

  prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &cpu_output_buffer);

  return 0;
}
void detect::initParams(ros::NodeHandle private_nh, std::string const &node_name)
{
  __APP_NAME__ = node_name;

  private_nh.param<std::string>("camera_topic", camera_topic, "/hik_cam_node/hik_camera");
  cout << __APP_NAME__ << ":camera_topic:" << camera_topic.c_str()<<endl;

  private_nh.param<std::string>("camera_topic_vision", camera_topic_out, "/detect_vision_image");
  cout << __APP_NAME__ << ":camera_topic_vision:" << camera_topic_out.c_str()<<endl;
  private_nh.param<std::string>("camera_topic_msg", camera_topic_msg, "/detect_msg_image");
  cout << __APP_NAME__ << ":camera_topic_msg:" << camera_topic_msg.c_str()<<endl;
    private_nh.param<std::string>("wts_name", wts_name, "");
  cout << __APP_NAME__ << ":wts_name:" << wts_name.c_str()<<endl;

  std::string detectPackpath = ros::package::getPath("detect");
  int npos = detectPackpath.find_last_of("/");
  subpath = detectPackpath.substr(0, npos);
  string cfgpath = subpath + "/cfg/";
  
  engine_name = subpath+"/detect/weghts/yolov5s.engine";
  // std::string config_file_path = "/home/cqjtu/Documents/catktin_ros/src/cfg/modelconfig.yaml";
  // cout << config_file_path << endl;
  // // YAML::Node config = YAML::LoadFile(config_file_path);
  // // medel_path = config["medel_path"].as<std::string>();
  // // yoloclassfile = medel_path + config["yoloclassfile"].as<std::string>();
  // // yoloModelConfiguration = medel_path + config["yoloModelConfiguration"].as<std::string>();
  // // yoloModelWeights = medel_path + config["yoloModelWeights"].as<std::string>();
  // // private_nh.param<bool>("isGpu", m_isGpu, true);
  
  // cout<<"engine_name:"<<"/home/cqjtu/Documents/catktin_ros/src/detect/weghts/yolov5s.engine"<<endl;
  // // private_node_handle.param<bool>("isGpu", m_isGpu, true);
  // cout << __APP_NAME__ << ":isGpu:" << m_isGpu;
  // cout << __APP_NAME__ << ":medel_path:" << medel_path.c_str();
  // cout << __APP_NAME__ << ":yoloclassfile:" << yoloclassfile.c_str();
  // cout << __APP_NAME__ << ":yoloModelConfiguration:" << yoloModelConfiguration.c_str();
  // cout << __APP_NAME__ << ":yoloModelWeights:" << yoloModelWeights.c_str();

  std::string config_file_Camera_path = "/home/cqjtu/Documents/catktin_ros/src/cfg/lidar_camera_calibration64.yaml";
  cout << config_file_Camera_path << endl;


  // cv::FileStorage config_camera(config_file_Camera_path, cv::FileStorage::READ);

  // config_camera["CameraMat"] >> camera_instrinsics_;
  // config_camera["DistCoeff"] >> distortion_coefficients_;
  // config_camera["CameraExtrinsicMat"] >> CameraExtrinsicMat_;
}



void detect::image_cb(const sensor_msgs::Image::ConstPtr &img)
{
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
  cv::Mat image = cv_ptr->image;

  image.copyTo(raw_img);
  // Get a batch of images
  std::vector<cv::Mat> img_batch;
  img_batch.push_back(image);
//////////////detet messages///////////////////
  ros::Time time = ros::Time::now();
  cv_ptr->encoding = "bgr8";
  cv_ptr->header=img->header;
 
  cv_ptr->image = raw_img;
  sensor_msgs::Image::ConstPtr imagept = cv_ptr->toImageMsg();
  ImageObj imagObj;
  imagObj.header = img->header;
  imagObj.roi_image = *imagept;
  ///////////////////////detect!!!!//////////////////
  std::vector<std::vector<Detection>> res_batch;
  do_detect(img_batch, res_batch);

  // Draw bounding boxes
  std::vector<cv::Rect> ress;
  ress=draw_bbox(img_batch, res_batch);
  // cv::imshow("hala",img_batch[0]);
  cv_ptr->image = img_batch.front();
  vision_detect_img_pub_.publish(cv_ptr->toImageMsg());
// cout<<"img_batch size :"<<img_batch.size()<<endl;
  for (size_t j = 0; j < img_batch.size(); j++)
  {
    auto res_obj=res_batch[j];
   for (size_t i=0;i<res_obj.size();i++)
	{
    cv::Rect rect=get_rect(raw_img,res_obj[i].bbox);
		ImageRect imagrect;
		imagrect.x = rect.x;
		imagrect.y = rect.y;
		imagrect.height = rect.height;
		imagrect.width = rect.width;
		imagrect.score = res_obj[i].conf;
    imagrect.class_id=res_obj[i].class_id;
		imagObj.obj.push_back(imagrect);
    // cv::rectangle(raw_img, ress[i], cv::Scalar(255, 255, 255), 2);
    // cout<<"imagerect:"<<imagrect<<endl;
    // cout<<"ress:"<<ress[i]<<endl;
    // cout<<"rect:"<<rect<<endl;
	}
  }
  // cv::imshow("'hal'",raw_img);
  // cv::waitKey(1);
  vision_detect_msg_pub_.publish(imagObj);
}



