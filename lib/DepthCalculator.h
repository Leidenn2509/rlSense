//
// Created by alrai on 15.10.2021.
//

#ifndef RSTEST_DEPTHCALCULATOR_H
#define RSTEST_DEPTHCALCULATOR_H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

namespace depth_calculator {
    cv::Mat getImage(const rs2::frame& frame);
}

class DepthCalculator {

};


#endif //RSTEST_DEPTHCALCULATOR_H
