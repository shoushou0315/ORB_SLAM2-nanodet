#include "Detecting.h"
#include <iostream>
#include <vector>
#include <mutex>
namespace ORB_SLAM2
{
Detecting::Detecting(const std::string &strSettingPath){
    mbHasImg=0;mbDetFin=0;mbPuted=0;
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    powersave= fSettings["powersave"];
    thread= fSettings["thread"];
    target_size= fSettings["target_size"];
    ncnn::set_cpu_powersave(powersave);
    std::string param="./models/nanodet.param",bin="./models/nanodet.bin";
    nanodet.load_param(param.c_str());
    nanodet.load_model(bin.c_str());
}
void Detecting::Run(){
    while(1){
        mbHasImg=0;mbPuted=0;mbDetFin=0;
        while(!mbHasImg){std::this_thread::sleep_for(std::chrono::milliseconds(1));}
        detect_nanodet(detImg,objects);
        mbDetFin=1;
        while(!mbPuted){std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    }
}
void Detecting::insertImage(const cv::Mat &im){
    detImg=im;
    //objects.clear();
    mbHasImg=1;mbDetFin=0;
}


cv::Mat Detecting::outputObj(cv::Mat &im){
  int color_index = 0;
  cv::Mat image = im.clone();

  for (size_t i = 0; i < objects.size(); i++)
  {
      const Object& obj = objects[i];

      const unsigned char* color = colors[color_index % 19];
      color_index++;
      cv::Scalar cc(color[0], color[1], color[2]);
      cv::rectangle(image, obj.rect, cc, 2);

      char text[256];
      sprintf(text, "%s %.1f%%", class_names[obj.label], obj.prob * 100);

      int baseLine = 0;
      cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

      int x = obj.rect.x;
      int y = obj.rect.y - label_size.height - baseLine;
      if (y < 0)
          y = 0;
      if (x + label_size.width > image.cols)
          x = image.cols - label_size.width;

      cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                    cv::Scalar(255, 255, 255), -1);

      cv::putText(image, text, cv::Point(x, y + label_size.height),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
  }
 
    mbPuted=1;
    return image;
}







int Detecting::detect_nanodet(const cv::Mat& bgr, std::vector<Object>& objects)
{
    nanodet.opt.use_vulkan_compute = false;
    // nanodet.opt.use_bf16_storage = true;

    int width = bgr.cols;
    int height = bgr.rows;

    // pad to multiple of 32
    int w = width;
    int h = height;
    float scale = 1.f;
    if (w > h)
    {
        scale = (float)target_size / w;
        w = target_size;
        h = h * scale;
    }
    else
    {
        scale = (float)target_size / h;
        h = target_size;
        w = w * scale;
    }

    ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR, width, height, w, h);

    // pad to target_size rectangle
    int wpad = (w + 31) / 32 * 32 - w;
    int hpad = (h + 31) / 32 * 32 - h;
    ncnn::Mat in_pad;
    ncnn::copy_make_border(in, in_pad, hpad / 2, hpad - hpad / 2, wpad / 2, wpad - wpad / 2, ncnn::BORDER_CONSTANT, 0.f);

    const float mean_vals[3] = {103.53f, 116.28f, 123.675f};
    const float norm_vals[3] = {0.017429f, 0.017507f, 0.017125f};
    in_pad.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Extractor ex = nanodet.create_extractor();
    ex.set_num_threads(thread);
    ex.input("input.1", in_pad);

    std::vector<Object> proposals;

    // stride 8
    {
        ncnn::Mat cls_pred;
        ncnn::Mat dis_pred;
        ex.extract("792", cls_pred);
        ex.extract("795", dis_pred);

        std::vector<Object> objects8;
        generate_proposals(cls_pred, dis_pred, 8, in_pad, prob_threshold, objects8);

        proposals.insert(proposals.end(), objects8.begin(), objects8.end());
    }

    // stride 16
    {
        ncnn::Mat cls_pred;
        ncnn::Mat dis_pred;
        ex.extract("814", cls_pred);
        ex.extract("817", dis_pred);

        std::vector<Object> objects16;
        generate_proposals(cls_pred, dis_pred, 16, in_pad, prob_threshold, objects16);

        proposals.insert(proposals.end(), objects16.begin(), objects16.end());
    }

    // stride 32
    {
        ncnn::Mat cls_pred;
        ncnn::Mat dis_pred;
        ex.extract("836", cls_pred);
        ex.extract("839", dis_pred);

        std::vector<Object> objects32;
        generate_proposals(cls_pred, dis_pred, 32, in_pad, prob_threshold, objects32);

        proposals.insert(proposals.end(), objects32.begin(), objects32.end());
    }

    // sort all proposals by score from highest to lowest
    qsort_descent_inplace(proposals);

    // apply nms with nms_threshold
    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked, nms_threshold);

    int count = picked.size();

    objects.resize(count);
    for (int i = 0; i < count; i++)
    {
        objects[i] = proposals[picked[i]];

        // adjust offset to original unpadded
        float x0 = (objects[i].rect.x - (wpad / 2)) / scale;
        float y0 = (objects[i].rect.y - (hpad / 2)) / scale;
        float x1 = (objects[i].rect.x + objects[i].rect.width - (wpad / 2)) / scale;
        float y1 = (objects[i].rect.y + objects[i].rect.height - (hpad / 2)) / scale;

        // clip
        x0 = std::max(std::min(x0, (float)(width - 1)), 0.f);
        y0 = std::max(std::min(y0, (float)(height - 1)), 0.f);
        x1 = std::max(std::min(x1, (float)(width - 1)), 0.f);
        y1 = std::max(std::min(y1, (float)(height - 1)), 0.f);

        objects[i].rect.x = x0;
        objects[i].rect.y = y0;
        objects[i].rect.width = x1 - x0;
        objects[i].rect.height = y1 - y0;
    }
    return 0;
}
}

