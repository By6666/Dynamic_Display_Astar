#ifndef PLANNER_BEZIRE_CURVE_H
#define PLANNER_BEZIRE_CURVE_H

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

#include "state.h"

// 二阶bezier curve
void SecOrderBezier(int pos_num, const cv::Point2d& start,
                    const cv::Point2d& goal, const cv::Point2d& mid,
                    std::vector<cv::Point2d>& path);

// 三阶bezier curve
void ThridOrder(int pos_num, const cv::Point2d& start, const cv::Point2d& mid_1,
                const cv::Point2d& mid_2, const cv::Point2d& goal,
                std::vector<cv::Point2d>& path, std::vector<Yaw_type>& yaw);

#endif