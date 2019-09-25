#ifndef PLANNER_PATH_PROCESS_H
#define PLANNER_PATH_PROCESS_H

#include <list>
#include <vector>

#include "bezier_curve.h"
#include "opencvdraw.h"
#include "state.h"
#include "typedef.h"

#define PATHDIVID 7

/* swap x & y for opencv draw*/
inline Point2d_type SwapPointXY(const Point2d_type& pt) {
  return Point2d_type(pt.y, pt.x);
}

/* car path smooth */
bool CarPathSmooth(int col, const std::list<CellPosState>& path,
                   std::vector<DrawPathPos>& final_path);

void PrePartInterpoletion(const DrawPathPos& start, const DrawPathPos& end,
                          std::vector<DrawPathPos>& final_path,
                          int pre_part_num);
/* 直线路段 插值 */
void PathInterpoletion(int start, int end, int pre_part_num,
                       const std::vector<DrawPathPos>& path,
                       std::vector<DrawPathPos>& final_path);

/* path smooth */
bool PathSmooth(const std::vector<DrawPathPos>& prime_path,
                std::vector<DrawPathPos>& final_path);

#endif