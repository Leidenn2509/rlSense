//
// Created by alrai on 15.10.2021.
//

#include "DepthCalculator.h"

cv::Mat depth_calculator::getImage(const rs2::frame& frame)  {
    const int w = frame.as<rs2::video_frame>().get_width();
    const int h = frame.as<rs2::video_frame>().get_height();

//    cv::Mat image(cv::Size(w, h), CV_8UC1, (void *) frame.get_data(), cv::Mat::AUTO_STEP);
    return {cv::Size(w, h), CV_8UC1, (void *) frame.get_data(), cv::Mat::AUTO_STEP};
}
