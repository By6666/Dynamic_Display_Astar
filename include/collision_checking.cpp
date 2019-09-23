#include "collision_checking.h"

/* 旋转car */
void RotateObj(std::vector<Point2d_type>& car,
               const Point2d_type& central_point, Yaw_type yaw) {
  for (auto& elem : car) {
    cv::Point2d temp = elem;
    elem.x = temp.x * cos(yaw) - temp.y * sin(yaw) + central_point.x;
    elem.y = temp.x * sin(yaw) + temp.y * cos(yaw) + central_point.y;
  }
}

/* *
 * 获得obj体边框四点
 * D-----A
 * | ——> |  车头方向
 * C-----B
 * A,B,C,D  ——>  0,1,2,3
 * */
std::vector<Point2d_type> GetBorder(const Point2d_type& central, Yaw_type yaw,
                                    double frame_length, double frame_width) {
  std::vector<Point2d_type> objposet(4);
  objposet.reserve(4);

  objposet[0].x = frame_length / 2.0;
  objposet[0].y = frame_width / 2.0;

  objposet[1].x = frame_length / 2.0;
  objposet[1].y = -frame_width / 2.0;

  objposet[2].x = -frame_length / 2.0;
  objposet[2].y = -frame_width / 2.0;

  objposet[3].x = -frame_length / 2.0;
  objposet[3].y = frame_width / 2.0;

  RotateObj(objposet, central, yaw);

  return objposet;
}

/* collision checking */
bool CollisionChecking(int col, const Point2i_type& first_pos,
                       const Point2i_type& second_pos, Yaw_type yaw,
                       const std::set<ID_SIZE>& obstacle_list) {
  /* gridmap 坐标向 opencv 转化 */
  Point2d_type front = GridmapToOpencvdraw(first_pos);
  Point2d_type rear = GridmapToOpencvdraw(second_pos);
  Point2d_type central((front.x + rear.x) / 2.0, (front.y + rear.y) / 2.0);

  double frame_length =
      sqrt(pow(front.x - rear.x, 2.0) + pow(front.y - rear.y, 2.0)) +
      CarSize::CarLength();
  double frame_width = CarSize::CarWidth();

  /* 获得frame */
  std::vector<Point2d_type> frame =
      GetBorder(central, yaw, frame_length, frame_width);

  //****先使用AABB，将障碍物范围缩小****
  //找到x、y最值
  double x_min = DBL_MAX, y_min = DBL_MAX, x_max = 0.0, y_max = 0.0;
  for (auto& elem : frame) {
    if (elem.x < x_min) x_min = elem.x;
    if (elem.x > x_max) x_max = elem.x;

    if (elem.y < y_min) y_min = elem.y;
    if (elem.y > y_max) y_max = elem.y;
  }

  x_min = OpencvdrawToGridmap(x_min);
  x_max = OpencvdrawToGridmap(x_max);
  y_min = OpencvdrawToGridmap(y_min);
  y_max = OpencvdrawToGridmap(y_max);

  //   std::cout << "x: [" << x_min << "," << x_max << "]  "
  //             << "y: [" << y_min << "," << y_max << "]" << std::endl;

  //在AABB中是障碍物的列表
  std::vector<Point2i_type> range_obstacle;
  for (int16_t i = x_min; i <= x_max; ++i) {
    for (int16_t j = y_min; j <= y_max; ++j) {
      Point2i_type temp(i, j);
      if (obstacle_list.find(CodeID(temp, col)) != obstacle_list.end())
        range_obstacle.push_back(temp);
    }
  }
  if (range_obstacle.size() == 0) return false;

  //****再使用OOBB进行障碍物碰撞检测****
  for (auto& elem : range_obstacle) {
    OBB obb(frame, GetObstacleAllVertex(elem));
    if (obb.IsCollision()) return true;
  }
  return false;
}

/* 获得障碍物的四个顶点 */
std::vector<Point2d_type> GetObstacleAllVertex(const Point2i_type& pt) {
  std::vector<Point2d_type> result;
  result.reserve(4);
  result.push_back(Point2d_type(pt.x * OpencvdrawSize::edge_length(),
                                pt.y * OpencvdrawSize::edge_length()));
  result.push_back(Point2d_type((pt.x + 1) * OpencvdrawSize::edge_length(),
                                pt.y * OpencvdrawSize::edge_length()));
  result.push_back(Point2d_type((pt.x + 1) * OpencvdrawSize::edge_length(),
                                (pt.y + 1) * OpencvdrawSize::edge_length()));
  result.push_back(Point2d_type(pt.x * OpencvdrawSize::edge_length(),
                                (pt.y + 1) * OpencvdrawSize::edge_length()));

  return result;
}