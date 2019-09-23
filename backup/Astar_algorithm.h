#ifndef PLANNER_ASTAR_ALGORITHM_H
#define PLANNER_ASTAR_ALGORITHM_H

#include <algorithm>
#include <cmath>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <vector>

#include "grid_input.h"
#include "opencv_draw.h"
#include "state.h"
#include "collision_detection.h"
#include "opencv_draw.h"

bool SearchOneMap(int map_num_);
void PrintSumResult();  //打印结果统计

class Astar {
 public:
  Astar(int row_, int columns_, Points statr_, Points goal_,
        std::map<ID_SIZE, Points> obstacle_list_);
  Astar(const Astar& as) = delete;

  // A*算法函数
  void AstarGetPath();

  //类内私有成员赋值函数
  inline int& set_row() { return row_; }
  inline int& set_column() { return column_; }
  inline Points& set_start_pos() { return start_pos_; }
  inline Points& set_goal_pos() { return goal_pos_; }
  inline Points& set_current_start() { return current_start_; }
  inline std::map<ID_SIZE, Points>& set_map_obstacle_list() {
    return map_obstacle_list_;
  }
  inline std::map<ID_SIZE, Points>& set_current_obstacle_list() {
    return current_obstacle_list_;
  }
  inline std::list<DriveStateInfo>& set_current_path() { return current_path_; }

  void PrintCountResult();
  void SearchResultPrint();
  //******************

  //获取类内私有成员函数
  inline int get_row() const { return row_; }
  inline int get_column() const { return column_; }
  inline Points get_start_pos() const { return start_pos_; }
  inline Points get_goal_pos() const { return goal_pos_; }
  inline Points get_current_start() const { return current_start_; }
  inline std::map<ID_SIZE, Points> get_map_obstacle_list() const {
    return map_obstacle_list_;
  }
  inline std::map<ID_SIZE, Points> get_current_obstacle_list() const {
    return current_obstacle_list_;
  }
  inline std::list<DriveStateInfo> get_current_path() const {
    return current_path_;
  }
  inline int get_all_expand_nums() const { return all_expand_points_count_; }
  inline int get_search_nums() const { return search_nums_count_; }
  inline int get_move_step_nums() const { return move_step_nums_; }
  inline std::list<DriveStateInfo> get_final_path() const { return final_path; }
  //******************

  //计算当前点与终点距离的函数
  inline double DistenceToGoal(const Points& current) {
    // //曼哈顿距离
    // return fabs(current.first - goal_pos_.first) +
    //        fabs(current.second - goal_pos_.second);

    // x距离与y距离的最大值
    return std::max(fabs(current.first - goal_pos_.first),
                    fabs(current.second - goal_pos_.second));

    // //欧式距离
    // return sqrt(pow(current.first - goal_pos_.first, 2) +
    //             pow(current.second - goal_pos_.second, 2));
  }

  //计算两点间的欧式距离
  inline double DisBetweenTwoPoint(const Points& pos1, const Points& pos2) {
    return sqrt(pow(pos1.first - pos2.first, 2) +
                pow(pos1.second - pos2.second, 2));
  }

  //获得当前节点的neighbors
  std::vector<CellInfo> GetNeighbors(const DriveStateInfo& current_pos);

  //判断遍历到的点能否push到openlist中
  void YawInkConstraintValue(const DriveStateInfo& current_pos, Points pos,
                             std::vector<CellInfo>& neighbors);

  //判断起点的车身角方向
  Yaw_type JudgeStartYaw(const Points& start, const Points& goal);

  //获得当前节点的neighbors，但仅仅包含障碍信息
  void UpdataMapInfo();

  void AddObstaclePoint(Points point);

  void StartMove() {
    current_start_ = current_path_.back().xoy;
    final_path.push_back(current_path_.back());
    current_path_.pop_back();
    ++move_step_nums_;
  }

  bool ArriveGoal() { return current_start_ == goal_pos_; }

  inline bool NextStepIsInObstacleList() {
    return IsInMap(current_path_.back().xoy, current_obstacle_list_);
  }

 private:
  int row_, column_;
  Points start_pos_, goal_pos_;
  Points current_start_;                         //当前起点
  std::map<ID_SIZE, Points> map_obstacle_list_;  //所有障碍物list
  std::list<DriveStateInfo> current_path_;       //当前路径
  std::list<DriveStateInfo> final_path;          // for opencv
  std::map<ID_SIZE, Points> current_obstacle_list_;

  int current_expand_points_count_,
      all_expand_points_count_,  //整体算法中的expand计数
      search_nums_count_,        //搜索次数计数
      move_step_nums_;           //移动步数

  //判断点是否在list中
  inline bool IsInList(const Points& point,
                       const std::list<DriveStateInfo>& list) {
    for (auto& elem : list) {
      if (point == elem.xoy) return true;
    }
    return false;
    // return std::find(list.begin(), list.end(), point) != list.end();
  }

  //
  inline ID_SIZE calculate_id(const Points& xoy) const {
    return static_cast<ID_SIZE>(xoy.first * column_ + xoy.second);
  }

  inline bool IsInMap(ID_SIZE id, const std::map<ID_SIZE, Points> map) {
    return map.find(id) != map.end();
  }

  inline bool IsInMap(const Points& point,
                      const std::map<ID_SIZE, Points> map) {
    ID_SIZE id = calculate_id(point);
    return map.find(id) != map.end();
  }

  inline bool IsInGrideMap(const Points& pos) {
    return pos.first >= 0 && pos.first < row_ && pos.second >= 0 &&
           pos.second < column_;
  }
};

#endif