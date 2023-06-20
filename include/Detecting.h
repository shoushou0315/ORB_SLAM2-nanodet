#ifndef DETECTING_H
#define DETECTING_H
#include <iostream>
#include <mutex>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Nanodet.h"
namespace ORB_SLAM2
{

class Detecting
{
public:
    float prob_threshold = 0.5f;
    float nms_threshold = 0.5f;
    std::vector<Object> objects;
    int powersave=0;
    int thread=4;
    int target_size = 320;
    ncnn::Net nanodet;
    cv::Mat detImg;
    bool mbHasImg;
    bool mbDetFin;
    bool mbPuted;
    Detecting* mpDetector;
    Detecting(const std::string &strSettingPath);
    void Run();
    void insertImage(const cv::Mat &im);
    cv::Mat outputObj(cv::Mat &im);
    int detect_nanodet(const cv::Mat& bgr, std::vector<Object>& objects);
 };   
} //namespace ORB_SLAM


#endif
