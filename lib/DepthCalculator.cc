//
// Created by alrai on 15.10.2021.
//

#include "DepthCalculator.h"


cv::Mat depth_calculator::again(cv::Mat &image1, cv::Mat &image2) {
    using namespace cv;

    GaussianBlur(image1, image1, Size(11, 11), 1);
    GaussianBlur(image2, image2, Size(11, 11), 1);


    int minDisparity = 0;
    int numDisparities = 64;
    int blockSize = 8;
    int P1 = 0;
    int P2 = 0;
    int disp12MaxDiff = 1;
    int preFilterCap = 0;
    int uniquenessRatio = 10;
    int speckleWindowSize = 10;
    int speckleRange = 8;
    int mode = StereoSGBM::MODE_SGBM;

    Ptr<StereoSGBM> stereo = StereoSGBM::create(
            minDisparity,
            numDisparities,
            blockSize,
            P1,
            P2,
            disp12MaxDiff,
            preFilterCap,
            uniquenessRatio,
            speckleWindowSize,
            speckleRange,
            mode
    );

    Mat disp;
    stereo->compute(image1, image2, disp);
    normalize(disp, disp, 0, 255, NORM_MINMAX);
    Mat disparity;
//    disp.convertTo(disparity, CV_32F);


    double B = 50;
    double f = 1.88;
    double Bf = B * f;
    double min = 300;
    double max = 0;

    Mat depths(disp.rows, disp.cols, CV_64F);
    disp.forEach<int16_t>([Bf, &depths, &min, &max](int16_t &p, const int position[]) -> void {
        if (p != 0) {
            double tmp = (Bf / (double)p);
            min = std::min(min, tmp);
            max = std::max(max, tmp);
            depths.at<double>(position[0], position[1]) = tmp;
        }
    });
//    cv::minMaxLoc(depths, &min, &max);
    std::cout << "min = " << min << std::endl;
    std::cout << "max = " << max << std::endl;
    Mat res(disp.rows, disp.cols, CV_8UC1);
    disp.forEach<int16_t>([&res, &depths, &min, &max](int16_t &p, const int position[]) -> void {
//        double tmp = (depths.at<double>(position[0], position[1]) - min)/(max - min)*255;
        res.at<uint8_t>(position[0], position[1]) = (uint8_t )(255*(depths.at<double>(position[0], position[1]) - min)/(max - min));
    });

//    std::vector<Mat> m = {disp, disp, disp};
//    merge(m, res);
//    cvtColor(res, res, COLORMAP_RAINBOW);
//    applyColorMap(res, res, COLORMAP_RAINBOW);
    return res;
}

cv::Mat depth_calculator::findAndDrawMatches(cv::Mat &image1, cv::Mat &image2) {
    using namespace cv;

    Ptr<SiftFeatureDetector> siftFeatureDetector = SiftFeatureDetector::create();
    std::vector<KeyPoint> keyPoints1, keyPoints2;

    SiftDescriptorExtractor siftDescriptorExtractor;
    Mat descriptor1, descriptor2;
    siftFeatureDetector->detectAndCompute(image1, Mat::ones(image1.rows, image1.cols, image1.type()), keyPoints1,
                                          descriptor1);
    siftFeatureDetector->detectAndCompute(image2, Mat::ones(image2.rows, image2.cols, image2.type()), keyPoints2,
                                          descriptor2);

    BFMatcher matcher(NORM_L2);
    std::vector<DMatch> matches;
    matcher.match(descriptor1, descriptor2, matches);

    std::vector<DMatch> goodMatches = filterMatches(keyPoints1, keyPoints2, matches);

    double mean = 0;
    int count = 0;
    double max_dist = 0;
    double min_dist = 100000;
    for (auto &match: goodMatches) {
        double dist = match.distance;
        mean = (dist + mean * count) / (count + 1);
        count++;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }
    std::cout << "Max distance = " << max_dist << std::endl;
    std::cout << "Min distance = " << min_dist << std::endl;
    std::cout << "size = " << matches.size() << std::endl;
    std::cout << "mean = " << mean << std::endl;

    Mat res(image1.rows, image1.cols + image2.cols, image1.type());
    drawMatches(image1, keyPoints1, image2, keyPoints2, goodMatches, res);
//    drawMatches(image1, goodKeyPoints1, image2, goodKeyPoints2, goodMatches, res);
    return res;
}

std::vector<cv::DMatch>
depth_calculator::filterMatches(std::vector<cv::KeyPoint> key_points1, std::vector<cv::KeyPoint> key_points2,
                                std::vector<cv::DMatch> &matches) {
    using namespace cv;
    std::vector<DMatch> good_matches;
//    std::vector<KeyPoint> good_key_points1;
//    std::vector<KeyPoint> good_key_points2;

    for (auto &match: matches) {
        KeyPoint left = key_points1[match.queryIdx];
        KeyPoint right = key_points2[match.trainIdx];
        if (abs(left.pt.y - right.pt.y) < 10 && left.pt.x < right.pt.x) {
//            good_key_points1.push_back(left);
//            good_key_points2.push_back(right);
//            match.queryIdx = (int) good_key_points1.size() - 1;
//            match.trainIdx = (int) good_key_points2.size() - 1;
            good_matches.push_back(match);
        }
    }

    return good_matches;
}

