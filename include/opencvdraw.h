#ifndef PLANNER_OPENCVDRAW_H
#define PLANNER_OPENCVDRAW_H

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <iostream>
#include <list>
#include <set>
#include <string>
#include <vector>

#include "carsize.h"
#include "collision_checking.h"
#include "path_process.h"
#include "state.h"
#include "typedef.h"

/* opencv coordinate system
 * o ————x————>
 *   |
 *   y
 *   |
 *   v
 * */

/* gridmap coordinate system
 * o ————y————>
 *   |
 *   x
 *   |
 *   v
 * */

/* Gridmap to opencv draw */
/* int to double */
inline double GridmapToOpencvdraw(int value) {
  return static_cast<double>(value * OpencvdrawSize::edge_length() +
                             OpencvdrawSize::edge_length() / 2);
}
inline Point2d_type GridmapToOpencvdraw(const Point2i_type& pt) {
  return Point2d_type(GridmapToOpencvdraw(pt.x), GridmapToOpencvdraw(pt.y));
}

/* opencv draw to gridmap */
/* double to int */
inline int OpencvdrawToGridmap(double value) {
  return static_cast<int>(value / OpencvdrawSize::edge_length());
}
inline Point2i_type OpencvdrawToGridmap(const Point2d_type& pt) {
  return Point2i_type(OpencvdrawToGridmap(pt.x), OpencvdrawToGridmap(pt.y));
}

/* opencv draw Astar result */
void DrawWholeMap(int num, int row, int col, const Point2i_type& start,
                  const Point2i_type& goal, const std::set<ID_SIZE>& obstacle,
                  const std::list<CellPosState>& path);

/* opencv draw grid map */
void DrawGridMap(cv::Mat& img, int row, int col, const Point2i_type& start,
                 const Point2i_type& goal, const std::set<ID_SIZE>& obstacle);

/* opencv draw car */
void DrawCar(cv::Mat& img, const cv::Point2d& central_point, Yaw_type yaw);

/* draw path */
void DrawPath(cv::Mat& img, const std::vector<DrawPathPos>& final_path);

#endif