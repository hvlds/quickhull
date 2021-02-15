#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "quickhull.hpp"
#include "kmeans.hpp"
#include "bbox.hpp"
#include <random>

int main() {

    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> distr(100, 600); // define the range

    cv::Mat image = cv::Mat(cv::Size(700, 700), CV_8UC3, cv::Scalar(255, 255, 255));

    std::vector<cv::Point> points;
    for (int i = 0; i < 100; i++) {
        cv::Point new_point(distr(gen), distr(gen));
        points.push_back(new_point);
    }

    auto clusters = k_means(points, 4, 10);
    for (auto cluster : clusters) {
        for (auto point : cluster.second) {
            cv::circle(image, point, 2, cv::Scalar(0, 0, 255));
        }

        auto convex_hull = quickhull(cluster.second);
        cv::polylines(image, convex_hull, true, cv::Scalar(255, 0 ,0));
        for (auto point : convex_hull) {
            cv::circle(image, point, 2, cv::Scalar(0, 0, 0), -1);
        }

        auto bbox = min_bbox(cluster.second);

        cv::polylines(image, bbox.get_points(), true, cv::Scalar(0, 0, 255));
        cv::circle(image, bbox.get_center(), 5, cv::Scalar(0, 0, 255), -1);   
    }

    cv::imshow("Quickhull", image);
    cv::waitKey(0);

    return EXIT_SUCCESS;
}