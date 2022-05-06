
#ifndef objectDetection2D_hpp
#define objectDetection2D_hpp

#include <stdio.h>
#include <opencv2/core.hpp>

#include "dataStructures.h"

void detectObjects(cv::Mat& img, std::vector<BoundingBox>& bBoxes,std::vector<std::string>&classes, float confThreshold, float nmsThreshold, 
                    std::string classesFile, std::string modelConfiguration, std::string modelWeights, bool bVis);
void show(std::string name,cv::Mat img);
#endif /* objectDetection2D_hpp */