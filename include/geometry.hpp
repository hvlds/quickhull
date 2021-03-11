#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

typedef struct {
    cv::Point top_left;
    cv::Point bottom_left;
    cv::Point top_right;
    cv::Point bottom_right;
    void print() {
        std::cout << "--- Rectangle ---" << std::endl;
        std::cout << "top left: " << top_left << std::endl;
        std::cout << "top right: " << top_right << std::endl;
        std::cout << "bottom left: " << bottom_left << std::endl;
        std::cout << "bottom right: " << bottom_right << std::endl;
    }
    std::vector<cv::Point> get_points() {
        std::vector<cv::Point> points{
            top_left,
            bottom_left,
            bottom_right,
            top_right};
        return points;
    }
    cv::Point get_center() {
        cv::Point center;
        center.x = top_left.x + bottom_left.x + bottom_right.x + top_right.x;
        center.y = top_left.y + bottom_left.y + bottom_right.y + top_right.y;
        center.x = static_cast<int>(center.x / 4);
        center.y = static_cast<int>(center.y / 4);
        return center;
    }

} Rectangle;

double cross_product(cv::Point v1, cv::Point v2) {
    return v1.x * v2.y - v1.y * v2.x;
}

double distance_to_line(cv::Point p0, cv::Point p1, cv::Point p2) {
    double distance;
    double numerator = std::abs(
        (p2.x - p1.x) * (p1.y - p0.y) - (p1.x - p0.x) * (p2.y - p1.y));
    double denominator = std::sqrt(
        std::pow((p2.x - p1.x), 2) + std::pow((p2.y - p1.y), 2));

    distance = numerator / denominator;

    return distance;
}

cv::Point get_rotated_point(double angle, const cv::Point point) {
    double rotated_x = cos(angle) * point.x - sin(angle) * point.y;
    double rotated_y = sin(angle) * point.x + cos(angle) * point.y;

    return cv::Point(static_cast<int>(rotated_x), static_cast<int>(rotated_y));
}

double get_angle(cv::Point p1, cv::Point p2) {
    return atan2(p2.y - p1.y, p2.x - p1.x);
}

cv::Point get_farthest_point(std::vector<cv::Point> points, cv::Point p1, cv::Point p2) {
    cv::Point farthest_point;
    double biggest_distance = 0;

    for (auto point : points) {
        double distance = distance_to_line(point, p1, p2);
        if (distance > biggest_distance) {
            biggest_distance = distance;
            farthest_point = point;
        }
    }

    return farthest_point;
}

cv::Point get_center(const std::vector<cv::Point> points) {
    cv::Point center;
    double x = 0;
    double y = 0;
    double count = 0;
    for (auto point : points) {
        x += point.x;
        y += point.y;
        count++;
    }
    center.x = static_cast<int>(x / count);
    center.y = static_cast<int>(y / count);

    return center;
}

std::vector<cv::Point> get_ordered_points(const std::vector<cv::Point> points) {
    auto center = get_center(points);
    // std::cout << "Center: " << center << std::endl;
    std::map<double, cv::Point> points_map;
    for (auto point : points) {
        points_map.insert(
            {get_angle(center, point), point});
    }

    std::vector<cv::Point> ordered_points;
    for (auto item : points_map) {
        ordered_points.push_back(item.second);
    }

    return ordered_points;
}

double get_distance(cv::Point p1, cv::Point p2) {
    double distance = std::sqrt(
        std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));

    return distance;
}

double get_angle_vectors(cv::Point v1, cv::Point v2) {
    double m1 = std::sqrt(std::pow(v1.x, 2) + std::pow(v1.y, 2));
    double m2 = std::sqrt(std::pow(v2.x, 2) + std::pow(v2.y, 2));
    double angle = std::acos(
        (v1.x * v2.x + v1.y * v2.y) / (m1 * m2));
        
    return angle;
}