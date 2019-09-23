#ifndef PLANNER_OPENCV_DRAW_H
#define PLANNER_OPENCV_DRAW_H
#include <stdint.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <list>
#include <map>
#include <string>
#include <vector>

#include "bezier_curve.h"
#include "grid_input.h"
#include "state.h"

extern const uint16_t edge_length;

void DrawMap(int num, int16_t row, int16_t col, const Points& start,
             const Points& goal, const std::map<ID_SIZE, Points>& obstacle,
             const std::list<DriveStateInfo>& path);

void DrawOneMap(int num);

void DrawCar(const cv::Point2d& central_point, Yaw_type yaw,
             const cv::Mat& img);

//将离散的点转为到opencv坐标大小
inline cv::Point2d GridToOpencv(const Points& pos) {
  return cv::Point2d(
      static_cast<double>(pos.first * edge_length + edge_length / 2),
      static_cast<double>(pos.second * edge_length + edge_length / 2));
}
void RotateCar(std::vector<cv::Point2d>& car, const cv::Point2d& central_point,
               Yaw_type yaw);

#endif