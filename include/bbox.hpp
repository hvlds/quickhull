#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

#include "geometry.hpp"
#include "quickhull.hpp"

std::pair<double, Rectangle> get_rectangle(double angle, const std::vector<cv::Point> points) {
    std::vector<cv::Point> rotated_points;

    for (auto point : points) {
        rotated_points.push_back(get_rotated_point(angle, point));
    }

    int max_x = 0;
    int min_x = 0;
    int max_y = 0;
    int min_y = 0;
    bool is_first = true;

    // Calculate the max and min points for the rotated points
    for (auto point : rotated_points) {
        if (is_first) {
            max_x = point.x;
            min_x = point.x;
            max_y = point.y;
            min_y = point.y;
            is_first = false;
        } else {
            if (point.x > max_x) {
                max_x = point.x;
            } else if (point.x < min_x) {
                min_x = point.x;
            }

            if (point.y > max_y) {
                max_y = point.y;
            } else if (point.y < min_y) {
                min_y = point.y;
            }
        }
    }

    double area = std::abs(max_x - min_x) * std::abs(max_y - min_y);

    cv::Point bottom_left = cv::Point(min_x, min_y);
    cv::Point bottom_right = cv::Point(max_x, min_y);
    cv::Point top_left = cv::Point(min_x, max_y);
    cv::Point top_right = cv::Point(max_x, max_y);

    Rectangle rect;
    rect.bottom_left = get_rotated_point(-1 * angle, bottom_left);
    rect.bottom_right = get_rotated_point(-1 * angle, bottom_right);
    rect.top_left = get_rotated_point(-1 * angle, top_left);
    rect.top_right = get_rotated_point(-1 * angle, top_right);

    return {area, rect};
}

Rectangle min_bbox(const std::vector<cv::Point> points) {
    auto convex_hull = quickhull(points);

    auto first_point = convex_hull.front();
    // auto last_point = convex_hull.back();

    cv::Point temp_point;
    std::map<double, Rectangle> areas;

    for (auto point : convex_hull) {
        auto temp_points = convex_hull;
        if (point != first_point) {
            double angle = get_angle(temp_point, point);
            areas.insert(get_rectangle(angle, convex_hull));
            temp_point = point;
        } else {
            temp_point = point;
        }
    }

    return areas.begin()->second;
}
