#ifndef LIGHT_DETECTOR__DETECTOR_HPP_
#define LIGHT_DETECTOR__DETECTOR_HPP_
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cstdio>
namespace rm_auto_aim_dart
{
    const int RED = 0;
    const int GREEN = 1;
    const int BLUE = 2;
    class Detector
    {
    public:
        int binary_threshold = 100; // 灰度255代表白色，0代表黑色
        Detector() = default;
        cv::Mat binary_image(const cv::Mat &color_image); // convert color image to binary image
        struct Light : public cv::RotatedRect
        {
            Light() = default;
            explicit Light(cv::Point2f center, float radius) : center(center), radius(radius) { initializeLight(center, radius); };
            cv::Point2f center;
            float radius;
            double width;
            double height;
            double title_angle;
            cv::Point2f top, bottom;
            cv::Point2f left, right;

        private:
            void initializeLight(const cv::Point2f &center, float radius);
        };
        struct LightParams
        {
            double min_minus = 100; // 调试时暂时修改为100,原来为40
            double max_minus = 200; // 调试时暂时修改为200，原来为50
        };
        std::vector<Light> find_lights(const cv::Mat &color_image, const cv::Mat &binary_image); // find lights in binary image
    };
}

#endif
