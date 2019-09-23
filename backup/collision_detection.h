#ifndef PLANNER_COLLISION_DETECTIVE_H
#define PLANNER_COLLISION_DETECTIVE_H

#include <opencv2/opencv.hpp>

#include <iostream>
#include <limits>
#include <map>
#include <vector>

#include "obb.h"
#include "opencv_draw.h"
#include "state.h"

std::vector<cv::Point2d> GetBorder(const cv::Point2d& central, Yaw_type yaw,
                                   double frame_length);

bool CollisionDetective(int col, const cv::Point2d central, Yaw_type yaw,
                        double frame_length,
                        const std::map<ID_SIZE, Points> obstacle_list);

inline int OpencvToGrid(double value) {
  return static_cast<int>(value / edge_length);
}

inline ID_SIZE calculate_id(const Points& xoy, int col) {
  return static_cast<ID_SIZE>(xoy.first * col + xoy.second);
}

std::vector<cv::Point2d> GetObstacleAllVertex(const Points& pos);

#endif