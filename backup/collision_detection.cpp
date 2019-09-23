#include "collision_detection.h"

//获得车体边框四点
// D-----A
// | ——> |  车头方向
// C-----B
// A,B,C,D  ——>  0,1,2,3
// frame_length为当前框架总长度
std::vector<cv::Point2d> GetBorder(const cv::Point2d& central, Yaw_type yaw,
                                   double frame_length) {
  cv::Point2d front_centre, rear_centre;
  std::vector<cv::Point2d> stg(4);

  front_centre.x = central.x + frame_length / 2.0 * cos(yaw);  // x
  front_centre.y = central.y + frame_length / 2.0 * sin(yaw);  // y

  rear_centre.x = central.x - frame_length / 2.0 * cos(yaw);  // x
  rear_centre.y = central.y - frame_length / 2.0 * sin(yaw);  // y

  stg[0].x = front_centre.x - CarWidth / 2.0 * sin(yaw);
  stg[0].y = front_centre.y + CarWidth / 2.0 * cos(yaw);

  stg[1].x = front_centre.x + CarWidth / 2.0 * sin(yaw);
  stg[1].y = front_centre.y - CarWidth / 2.0 * cos(yaw);

  stg[2].x = rear_centre.x + CarWidth / 2.0 * sin(yaw);
  stg[2].y = rear_centre.y - CarWidth / 2.0 * cos(yaw);

  stg[3].x = rear_centre.x - CarWidth / 2.0 * sin(yaw);
  stg[3].y = rear_centre.y + CarWidth / 2.0 * cos(yaw);

  return stg;
}

//碰撞检测
bool CollisionDetective(int col, const cv::Point2d central, Yaw_type yaw,
                        double frame_length,
                        const std::map<ID_SIZE, Points> obstacle_list) {
  std::vector<cv::Point2d> frame = GetBorder(central, yaw, frame_length);

  //****先使用AABB，将障碍物范围缩小****
  //找到x、y最值
  double x_min = std::numeric_limits<double>::max(),
         y_min = std::numeric_limits<double>::max(), x_max = 0, y_max = 0;
  for (auto& elem : frame) {
    if (elem.x < x_min) x_min = elem.x;
    if (elem.x > x_max) x_max = elem.x;

    if (elem.y < y_min) y_min = elem.y;
    if (elem.y > y_max) y_max = elem.y;
  }
  std::cout << "  " << x_min << "  " << x_max << "  " << y_min << "  " << y_max
            << std::endl;
  x_min = OpencvToGrid(x_min);
  x_max = OpencvToGrid(x_max);
  y_min = OpencvToGrid(y_min);
  y_max = OpencvToGrid(y_max);

  std::cout << "x: [" << x_min << "," << x_max << "]  "
            << "y: [" << y_min << "," << y_max << "]" << std::endl;

  //在AABB中是障碍物的列表
  std::vector<Points> range_obstacle;
  for (int16_t i = x_min; i <= x_max; ++i) {
    for (int16_t j = y_min; j <= y_max; ++j) {
      Points temp(i, j);
      if (obstacle_list.find(calculate_id(temp, col)) != obstacle_list.end())
        range_obstacle.push_back(temp);
    }
  }

  //****再使用OOBB进行障碍物碰撞检测****
  for (auto& elem : range_obstacle) {
    OBB obb(frame, GetObstacleAllVertex(elem));
    if (obb.IsCollision()) return true;
  }

  return false;
}

/* 获得障碍物的四个顶点 */
std::vector<cv::Point2d> GetObstacleAllVertex(const Points& pos) {
  std::vector<cv::Point2d> result;
  result.reserve(4);
  result.push_back(
      cv::Point2d(pos.first * edge_length, pos.second * edge_length));
  result.push_back(
      cv::Point2d((pos.first + 1) * edge_length, pos.second * edge_length));
  result.push_back(cv::Point2d((pos.first + 1) * edge_length,
                               (pos.second + 1) * edge_length));
  result.push_back(
      cv::Point2d(pos.first * edge_length, (pos.second + 1) * edge_length));

  return result;
}