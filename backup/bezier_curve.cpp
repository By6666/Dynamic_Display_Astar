#include "bezier_curve.h"

// 二阶bezier curve
void SecOrderBezier(int pos_num, const cv::Point2d& start,
                    const cv::Point2d& mid, const cv::Point2d& goal,
                    std::vector<cv::Point2d>& path) {
  double t = 0.0;
  for (int16_t i = 0; i < pos_num; ++i) {
    t = static_cast<double>(i) / static_cast<double>(pos_num);
    cv::Point2d temp;
    temp.x = pow((1.0 - t), 2.0) * start.x + 2.0 * t * (1.0 - t) * mid.x +
             pow(t, 2.0) * goal.x;
    temp.y = pow((1.0 - t), 2.0) * start.y + 2.0 * t * (1.0 - t) * mid.y +
             pow(t, 2.0) * goal.y;
    path.push_back(temp);
  }
}

// 三阶bezier curve
void ThridOrder(int pos_num, const cv::Point2d& start, const cv::Point2d& mid_1,
                const cv::Point2d& mid_2, const cv::Point2d& goal,
                std::vector<cv::Point2d>& path, std::vector<Yaw_type>& yaw) {
  double t = 0.0;
  cv::Point2d last_temp = path.back();
  for (int16_t i = 0; i < pos_num; ++i) {
    t = static_cast<double>(i) / static_cast<double>(pos_num);
    cv::Point2d temp;
    temp.x = pow((1.0 - t), 3.0) * start.x +
             3.0 * pow((1.0 - t), 2.0) * t * mid_1.x +
             3.0 * pow(t, 2.0) * (1.0 - t) * mid_2.x + pow(t, 3.0) * goal.x;
    temp.y = pow((1.0 - t), 3.0) * start.y +
             3.0 * pow((1.0 - t), 2.0) * t * mid_1.y +
             3.0 * pow(t, 2.0) * (1.0 - t) * mid_2.y + pow(t, 3.0) * goal.y;
    path.push_back(temp);
    yaw.push_back(atan2(temp.y - last_temp.y, temp.x - last_temp.x));
    last_temp = temp;
  }
}