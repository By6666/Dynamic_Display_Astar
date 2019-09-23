#ifndef PLANNER_COLLISION_CHECKING_H
#define PLANNER_COLLISION_CHECKING_H

#include <cmath>
#include <iostream>
#include <set>
#include <vector>

#include "carsize.h"
#include "obb.h"
#include "opencvdraw.h"
#include "typedef.h"

/* 获得旋转车辆的四个点 */
std::vector<Point2d_type> GetBorder(const Point2d_type& central, Yaw_type yaw,
                                    double frame_length, double frame_width);

/* 旋转车辆 */
void RotateObj(std::vector<Point2d_type>& poset,
               const Point2d_type& central_point, Yaw_type yaw);

/* collision checking */
bool CollisionChecking(int col, const Point2i_type& first_pos,
                       const Point2i_type& second_pos, Yaw_type yaw,
                       const std::set<ID_SIZE>& obstacle_list);

/* 获得障碍物的四个顶点 */
std::vector<Point2d_type> GetObstacleAllVertex(const Point2i_type& pos);

#endif