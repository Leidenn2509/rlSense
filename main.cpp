#include <iostream>

#include <librealsense2/rs.hpp>
#include <string.h>
#include <opencv2/opencv.hpp>

using namespace rs2;
using namespace cv;

const int BASELINE = 50;

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


        Ptr<SIFT> sift = cv::SIFT::create();



        int i = 0;
        for (auto &a: results) {
            const int w = a.as<rs2::video_frame>().get_width();
            const int h = a.as<rs2::video_frame>().get_height();

            cv::Mat image(cv::Size(w, h), CV_8UC1, (void *) a.get_data(), cv::Mat::AUTO_STEP);
            cv::imshow("test" + std::__cxx11::to_string(i), image);
//            cv::imwrite(std::__cxx11::to_string(all) + "_" + std::__cxx11::to_string(i) + "_.png", image);

            SiftFeatureDetector siftFeatureDetector;
            std::vector<KeyPoint> keyPoints;

            siftFeatureDetector.detect(image, keyPoints)



            i++;
            int k = cv::waitKey(1);
            if (k == 'x') {
                end = true;
            }
        }
        all++;


    }

    return 0;
}

Mat getImage(frame frame) {
    return const int w = a.as<rs2::video_frame>().get_width();
    const int h = a.as<rs2::video_frame>().get_height();

    cv::Mat image(cv::Size(w, h), CV_8UC1, (void *) a.get_data(), cv::Mat::AUTO_STEP);

}

Mat depth(Mat first, Mat second) {

}