#include <iostream>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include "lib/DepthCalculator.h"

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


        cv::Mat image1 = depth_calculator::getImage(results[0]);
        cv::Mat image2 = depth_calculator::getImage(results[1]);

        cv::imshow("test1", image1);
        cv::imshow("test2", image2);

        Ptr<SiftFeatureDetector> siftFeatureDetector = SiftFeatureDetector::create();
        std::vector<KeyPoint> keyPoints1, keyPoints2;

//        siftFeatureDetector.detect(image1, keyPoints1);
//        siftFeatureDetector.detect(image2, keyPoints2);
//
        SiftDescriptorExtractor siftDescriptorExtractor;
        Mat descriptor1, descriptor2;
//        siftDescriptorExtractor.compute(image1, keyPoints1, descriptor1);
//        siftDescriptorExtractor.compute(image2, keyPoints2, descriptor2);
        siftFeatureDetector->detectAndCompute(image1, Mat::ones(image1.rows, image1.cols, image1.type()), keyPoints1, descriptor1);
        siftFeatureDetector->detectAndCompute(image2, Mat::ones(image2.rows, image2.cols, image2.type()), keyPoints2, descriptor2);

        BFMatcher matcher(NORM_L2);
        std::vector<DMatch> matches;
        matcher.match(descriptor1, descriptor2, matches);
        Mat res(image1.rows, image1.cols + image2.cols, image1.type());
        drawMatches(image1, keyPoints1, image2, keyPoints2, matches, res);
        imshow("blati", res);


        int k = cv::waitKey(1);
        if (k == 'x') {
            end = true;
        }
        all++;


    }

    return 0;
}


Mat depth(Mat first, Mat second) {

}