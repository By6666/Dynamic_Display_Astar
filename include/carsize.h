#ifndef PLANNER_CARSIZE_H
#define PLANNER_CARSIZE_H

#include "typedef.h"

/* 车辆尺寸 */
namespace CarSize {
inline double kConstraintValue() { return M_PI / 6.0; }  //车辆转向角约束值
inline double CarLength() { return 46.0; }               //车长度，单位cm
inline double CarWidth() { return 18.0; }                //车宽度，单位cm
}  // namespace CarSize

/* opencv draw size */
namespace OpencvdrawSize {
inline int edge_length() { return 30; }
inline int car_line_thinckness() { return 1; }
inline int grid_line_thinckness() { return 1; }
}  // namespace OpencvdrawSize

#endif