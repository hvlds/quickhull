#pragma once

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

#include "geometry.hpp"

std::pair<cv::Point, cv::Point> get_borders(
    const std::vector<cv::Point> points) {
    cv::Point left_point;
    cv::Point right_point;
    bool is_first = true;
    for (auto point : points) {
        if (point.x < left_point.x && !is_first) {
            left_point = point;
        } else if (point.x > right_point.x && !is_first) {
            right_point = point;
        } else if (is_first) {
            left_point = point;
            right_point = point;
            is_first = false;
        }
    }
    return {left_point, right_point};
}

void findhull(
    std::shared_ptr<std::vector<cv::Point>> convex_hull,
    std::vector<cv::Point> side_points,
    cv::Point p_point,
    cv::Point q_point) {
    if (side_points.size() == 0)
        return;

    cv::Point c_point = get_farthest_point(
        side_points, p_point, q_point);

    // convex_hull->pop_back();
    convex_hull->push_back(c_point);
    // convex_hull->push_back(q_point);

    // side > 0 -> left
    // side < 0 -> right
    double side = cross_product(
        (q_point - p_point), (c_point - p_point));

    // S1
    std::vector<cv::Point> s1;
    for (auto point : side_points) {
        if (point != p_point && point != q_point) {
            double product = cross_product(
                (c_point - p_point), (point - p_point));
            if ((product * side) > 0) {
                s1.push_back(point);
            }
        }
    }

    // S2
    std::vector<cv::Point> s2;
    for (auto point : side_points) {
        if (point != p_point && point != q_point) {
            double product = cross_product(
                (q_point - c_point), (point - c_point));
            if ((product * side) > 0) {
                s2.push_back(point);
            }
        }
    }

    findhull(convex_hull, s1, p_point, c_point);
    findhull(convex_hull, s2, c_point, q_point);
}

std::vector<cv::Point> quickhull(const std::vector<cv::Point> points) {
    auto convex_hull = std::make_shared<std::vector<cv::Point>>();

    if (points.size() < 2)
        return *convex_hull;

    auto border = get_borders(points);

    cv::Point left_point = border.first;
    cv::Point right_point = border.second;

    convex_hull->push_back(left_point);
    convex_hull->push_back(right_point);

    std::vector<cv::Point> right_side;
    std::vector<cv::Point> left_side;

    for (auto point : points) {
        if (point != left_point && point != right_point) {
            double product = cross_product(
                (right_point - left_point), (point - left_point));
            if (product > 0) {
                left_side.push_back(point);
            } else if (product < 0) {
                right_side.push_back(point);
            }
        }
    }
    findhull(convex_hull, left_side, left_point, right_point);
    findhull(convex_hull, right_side, left_point, right_point);

    std::vector<cv::Point> ordered_points;
    if ((*convex_hull).size() > 0)
        ordered_points = get_ordered_points(*convex_hull);

    return ordered_points;
}
