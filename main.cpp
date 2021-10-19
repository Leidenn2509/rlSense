#include <iostream>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include "lib/DepthCalculator.h"

using namespace rs2;
using namespace cv;

const int BASELINE = 50;

cv::Mat getImage(const rs2::frame &frame) {
    const int w = frame.as<rs2::video_frame>().get_width();
    const int h = frame.as<rs2::video_frame>().get_height();

//    cv::Mat image(cv::Size(w, h), CV_8UC1, (void *) frame.get_data(), cv::Mat::AUTO_STEP);
    return {cv::Size(w, h), CV_8UC1, (void *) frame.get_data(), cv::Mat::AUTO_STEP};
}

int main() {
    std::cout << "Hello, World!" << std::endl;

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    context ctx;
    config cfg;
    cfg.enable_all_streams();
    pipeline p;

    p.start(cfg);


    bool end = false;
    int all = 0;
    while (!end) {
        rs2::frameset frames = p.wait_for_frames();

        std::vector<frame> results;

        frames.foreach_rs([&results](frame frm) {
            if (frm.get_profile().stream_type() == RS2_STREAM_INFRARED) {
                results.push_back(std::move(frm));
            }
        });


        cv::Mat image1 = getImage(results[0]);
        cv::Mat image2 = getImage(results[1]);

        cv::imshow("test1", image1);
        cv::imshow("test2", image2);

        Mat res = depth_calculator::again(image1, image2);
        cv::imshow("res", res);

        int k = cv::waitKey(1);
        if (k == 'x') {
            end = true;
        }


    }

    return 0;
}


Mat depth(Mat first, Mat second) {

}