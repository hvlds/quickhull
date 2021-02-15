/**
 * @file kmeans.hpp
 * @brief Modification of the K-Means Algorithms from 
 * https://github.com/goldsborough/k-means
 * @version 0.1
 * @date 2020-07-30
 * 
 */

#ifndef KMEANS_HPP
#define KMEANS_HPP

#include <opencv2/core.hpp>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <vector>
#include <map>

float square(float value) {
    return value * value;
}

float squared_l2_distance(cv::Point first, cv::Point second) {
    return square(first.x - second.x) + square(first.y - second.y);
}

std::map<size_t, std::vector<cv::Point>> k_means(
    const std::vector<cv::Point>& data,
    size_t k,
    size_t number_of_iterations) {

    static std::random_device seed;
    static std::mt19937 random_number_generator(seed());
    std::uniform_int_distribution<size_t> indices(0, data.size() - 1);

    // Pick centroids as random points from the dataset.
    std::vector<cv::Point> means(k);
    for (auto& cluster : means) {
        cluster = data[indices(random_number_generator)];
    }

    std::vector<size_t> assignments(data.size());
    for (size_t iteration = 0; iteration < number_of_iterations; ++iteration) {
        // Find assignments.
        for (size_t point = 0; point < data.size(); ++point) {
            auto best_distance = std::numeric_limits<float>::max();
            size_t best_cluster = 0;
            for (size_t cluster = 0; cluster < k; ++cluster) {
                const float distance =
                    squared_l2_distance(data[point], means[cluster]);
                if (distance < best_distance) {
                    best_distance = distance;
                    best_cluster = cluster;
                }
            }
            assignments[point] = best_cluster;
        }

        // Sum up and count points for each cluster.
        std::vector<cv::Point> new_means(k);
        std::vector<size_t> counts(k, 0);
        for (size_t point = 0; point < data.size(); ++point) {
            const auto cluster = assignments[point];
            new_means[cluster].x += data[point].x;
            new_means[cluster].y += data[point].y;
            counts[cluster] += 1;
        }

        // Divide sums by counts to get new centroids.
        for (size_t cluster = 0; cluster < k; ++cluster) {
            // Turn 0/0 into 0/1 to avoid zero division.
            const auto count = std::max<size_t>(1, counts[cluster]);
            means[cluster].x = static_cast<int>(std::round(new_means[cluster].x / count));
            means[cluster].y = static_cast<int>(std::round(new_means[cluster].y / count));
        }
    }

    // Generate empty vectors for every cluster
    std::map<size_t, std::vector<cv::Point>> clusters;
    for (size_t cluster = 0; cluster < k; ++cluster) {
        std::vector<cv::Point> temp_vector{};
        clusters[cluster] = temp_vector;
    }

    // Fill the vectors with the points
    for (size_t point = 0; point < data.size(); ++point) {
        clusters[assignments[point]].push_back(data[point]);
    }

    return clusters;
}

#endif