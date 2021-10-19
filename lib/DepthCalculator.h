//
// Created by alrai on 15.10.2021.
//

#ifndef RSTEST_DEPTHCALCULATOR_H
#define RSTEST_DEPTHCALCULATOR_H

#include <opencv2/opencv.hpp>

namespace depth_calculator {
    cv::Mat again(cv::Mat &image1, cv::Mat &image2);

    cv::Mat findAndDrawMatches(cv::Mat &image1, cv::Mat &image2);

    std::vector<cv::DMatch> filterMatches(std::vector<cv::KeyPoint> key_points1,
                                          std::vector<cv::KeyPoint> key_points2,
                                          std::vector<cv::DMatch> &matches);

}

class DepthCalculator {

};


#endif //RSTEST_DEPTHCALCULATOR_H
