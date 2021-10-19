//
// Created by alrai on 15.10.2021.
//

#include <iostream>

#include <opencv2/opencv.hpp>

#include "lib/DepthCalculator.h"

#include <string>

const int BASELINE = 50;

std::pair<std::string, std::string> getImageFileName(int imageCode) {
    std::string imageName = "res/" + std::to_string(imageCode) + "_";
    return std::make_pair(imageName + "0_.png", imageName + "1_.png");
}

std::pair<cv::Mat, cv::Mat> readImages(int imageCode) {
    auto[a, b] = getImageFileName(imageCode);
    return std::make_pair(cv::imread(a, cv::IMREAD_GRAYSCALE), cv::imread(b, cv::IMREAD_GRAYSCALE));
}

void updateCode(cv::Mat &image1, cv::Mat &image2, int imageCode, cv::Mat &out) {
    std::cout << imageCode << std::endl;
    auto images = readImages(imageCode);
    image1 = images.first;
    image2 = images.second;
    cv::hconcat(image1, image2, out);

}

int main() {
    std::cout << "Hello, World!" << std::endl;
    int imageCode = 142;
    cv::Mat image1;
    cv::Mat image2;
    cv::Mat out;
    cv::Mat matches;
    updateCode(image1, image2, imageCode, out);
    matches = depth_calculator::findAndDrawMatches(image1, image2);

    bool updated = true;
    while (true) {


        cv::imshow("matches", matches);
        cv::imshow("img", out);
        int c = cv::waitKeyEx(100);
        if (c == 120) {
            break;
        } else if (c == 97) {
            if (imageCode > 0) {
                updateCode(image1, image2, --imageCode, out);
                updated = true;
            }
        } else if (c == 100) {
            if (imageCode < 224) {
                updateCode(image1, image2, ++imageCode, out);
                updated = true;
            }
        }

        if (updated) {
            updated = false;
//            matches = depth_calculator::findAndDrawMatches(image1, image2);
            matches = depth_calculator::again(image1, image2);

        }
    }
}