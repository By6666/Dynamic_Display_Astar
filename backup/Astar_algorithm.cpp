#include "Astar_algorithm.h"

std::vector<std::string> sum_result;  //总结输出结果

bool SearchOneMap(int map_num_) {
  //获得map信息
  GrideInput map_info(map_num_);
  map_info.GetOneGrid();
  map_info.PrintMap();  //打印原始map

  bool find_goal_flg = false;

  //数据传入，构造类对象
  Astar Astar_algorithm(map_info.get_grid_rows(), map_info.get_grid_columns(),
                        map_info.get_start_pos(), map_info.get_goal_pos(),
                        map_info.get_obstacle_pos());

  while (1) {
    //规划路径
    std::cout << "**********" << std::endl;
    std::cout << "search num : " << Astar_algorithm.get_search_nums() + 1
              << std::endl;
    Astar_algorithm.AstarGetPath();  //以当前起点为起点进行一次路径规划

    if (Astar_algorithm.get_current_path().size() == 0) {
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
      std::cout << "|final result : no path to goal !!|" << std::endl;
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
      Astar_algorithm.PrintCountResult();
      break;
    }

    while (!Astar_algorithm.get_current_path().empty()) {
      //更新当前点各临近点的信息
      Astar_algorithm.UpdataMapInfo();

      if (Astar_algorithm.NextStepIsInObstacleList()) {
        break;  //当前点要移动到的下一个点是obstacle
      } else {
        Astar_algorithm.StartMove();
      }
    }

    if (Astar_algorithm.ArriveGoal()) {  //走到了终点
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
      std::cout << "|final result: get goal successflly!!|" << std::endl;
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
      Astar_algorithm.PrintCountResult();
      find_goal_flg = true;
      break;
    }
  }
  sum_result.push_back(
      std::to_string(map_num_ + 1) + "          " +
      std::to_string(Astar_algorithm.get_search_nums()) + "          " +
      std::to_string(Astar_algorithm.get_all_expand_nums()) + "          " +
      std::to_string(Astar_algorithm.get_move_step_nums()));

  /* opencv draw */
  DrawMap(map_num_, map_info.get_grid_rows(), map_info.get_grid_columns(),
          map_info.get_start_pos(), map_info.get_goal_pos(),
          map_info.get_obstacle_pos(), Astar_algorithm.get_final_path());

  return find_goal_flg;
}

Astar::Astar(int row, int column, Points statr, Points goal,
             std::map<ID_SIZE, Points> obstacle_list_)
    : row_(row),
      column_(column),
      start_pos_(statr),
      goal_pos_(goal),
      map_obstacle_list_(obstacle_list_) {
  current_obstacle_list_ = map_obstacle_list_;
  current_start_ = start_pos_;
  all_expand_points_count_ = 0;
  search_nums_count_ = 0;
  move_step_nums_ = 0;
  final_path.push_back(
      {current_start_, JudgeStartYaw(current_start_, goal_pos_)});
}

//获得当前节点的neighbors,一次A*算法中
std::vector<CellInfo> Astar::GetNeighbors(const DriveStateInfo& current_pos) {
  std::vector<CellInfo> neighbors, final_neighbors;
  int expand_state_flg = 0;
  // Up
  Points temp = ExpandPos::Up(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    expand_state_flg |= ExpandState::Up;
    YawInkConstraintValue(current_pos, temp, neighbors);
  }
  // Down
  temp = ExpandPos::Dw(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    expand_state_flg |= ExpandState::Dw;
    YawInkConstraintValue(current_pos, temp, neighbors);
  }
  // Lf
  temp = ExpandPos::Lf(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    expand_state_flg |= ExpandState::Lf;
    YawInkConstraintValue(current_pos, temp, neighbors);
  }
  // Rg
  temp = ExpandPos::Rg(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    expand_state_flg |= ExpandState::Rg;
    YawInkConstraintValue(current_pos, temp, neighbors);
  }

  // Up_Rg
  temp = ExpandPos::Up_Rg(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    expand_state_flg |= ExpandState::Up_Rg;
  }
  // Dw_Rg
  temp = ExpandPos::Dw_Rg(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    expand_state_flg |= ExpandState::Dw_Rg;
  }
  // Dw_Lf
  temp = ExpandPos::Dw_Lf(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    expand_state_flg |= ExpandState::Dw_Lf;
  }
  // Up_Lf
  temp = ExpandPos::Up_Lf(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    expand_state_flg |= ExpandState::Up_Lf;
  }

  // Rg_Up_26_5
  temp = ExpandPos::Rg_Up_26_5(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    if ((ExpandState::Rg & expand_state_flg) &&
        (ExpandState::Up_Rg & expand_state_flg))
      YawInkConstraintValue(current_pos, temp, neighbors);
  }
  // Rg_Dw_26_5
  temp = ExpandPos::Rg_Dw_26_5(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    if ((ExpandState::Rg & expand_state_flg) &&
        (ExpandState::Dw_Rg & expand_state_flg))
      YawInkConstraintValue(current_pos, temp, neighbors);
  }
  // Dw_Rg_26_5
  temp = ExpandPos::Dw_Rg_26_5(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    if ((ExpandState::Dw & expand_state_flg) &&
        (ExpandState::Dw_Rg & expand_state_flg))
      YawInkConstraintValue(current_pos, temp, neighbors);
  }
  // Dw_Lf_26_5
  temp = ExpandPos::Dw_Lf_26_5(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    if ((ExpandState::Dw & expand_state_flg) &&
        (ExpandState::Dw_Lf & expand_state_flg))
      YawInkConstraintValue(current_pos, temp, neighbors);
  }
  // Lf_Dw_26_5
  temp = ExpandPos::Lf_Dw_26_5(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    if ((ExpandState::Lf & expand_state_flg) &&
        (ExpandState::Dw_Lf & expand_state_flg))
      YawInkConstraintValue(current_pos, temp, neighbors);
  }
  // Lf_Up_26_5
  temp = ExpandPos::Lf_Up_26_5(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    if ((ExpandState::Lf & expand_state_flg) &&
        (ExpandState::Up_Lf & expand_state_flg))
      YawInkConstraintValue(current_pos, temp, neighbors);
  }
  // Up_Lf_26_5
  temp = ExpandPos::Up_Lf_26_5/* 判断起点车身yaw */
Yaw_type Astar::JudgeStartYaw() {
  if (abs(goal_pos_.y - start_pos_.y) > abs(goal_pos_.x - start_pos_.x)) {
    if (goal_pos_.y > start_pos_.y)
      return M_PI_2;
    else
      return -M_PI_2;
  } else {
    if (goal_pos_.x > start_pos_.x)
      return 0.0;
    else
      return M_PI;
  }
}(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    if ((ExpandState::Up & expand_state_flg) &&
        (ExpandState::Up_Lf & expand_state_flg))
      YawInkConstraintValue(current_pos, temp, neighbors);
  }
  // Up_Rg_26_5
  temp = ExpandPos::Up_Rg_26_5(current_pos.xoy);
  if (IsInGrideMap(temp) && !IsInMap(temp, current_obstacle_list_)) {
    if ((ExpandState::Up & expand_state_flg) &&
        (ExpandState::Up_Rg & expand_state_flg))
      YawInkConstraintValue(current_pos, temp, neighbors);
  }

  for (int16_t i = 0; i < neighbors.size(); ++i) {
    cv::Point2d front = GridToOpencv(neighbors[i].state.xoy);
    cv::Point2d rear = GridToOpencv(current_pos.xoy);
    cv::Point2d central((front.x + rear.x) / 2.0, (front.y + rear.y) / 2.0);
    double frame_length =
        sqrt(pow(front.x - rear.x, 2.0) + pow(front.y - rear.y, 2.0)) +
        CarLength;
    if (CollisionDetective(column_, central, neighbors[i].state.yaw,
                           frame_length, current_obstacle_list_))
      continue;
    else
      final_neighbors.push_back(neighbors[i]);
  }
  return final_neighbors;
}

//判断遍历到的点能否push到openlist中
void Astar::YawInkConstraintValue(const DriveStateInfo& current_pos, Points pos,
                                  std::vector<CellInfo>& neighbors) {
  Yaw_type temp_yaw = calculate_arct2(current_pos.xoy, pos);
  Yaw_type yaw = fabs(temp_yaw - current_pos.yaw);
  yaw = (yaw > M_PI) ? (2.0 * M_PI - yaw) : yaw;
  std::cout << std::endl << "yaw : " << yaw;
  if (yaw < kConstraintValue) {
    neighbors.push_back({0, 0, pos, temp_yaw});
    std::cout << " push --> (" << pos.first << "," << pos.second
              << ")  yaw : " << temp_yaw << std::endl;
  }
}

//获得当前节点的neighbors，但仅仅包含障碍信息,整体A*算法中
void Astar::UpdataMapInfo() {
  if ((current_start_.first - 1) >= 0) {
    AddObstaclePoint(Points(current_start_.first - 1, current_start_.second));
  }
  // Down
  if ((current_start_.first + 1) < row_) {
    AddObstaclePoint(Points(current_start_.first + 1, current_start_.second));
  }
  // Left
  if ((current_start_.second - 1) >= 0) {
    AddObstaclePoint(Points(current_start_.first, current_start_.second - 1));
  }
  // Right
  if ((current_start_.second + 1) < column_) {
    AddObstaclePoint(Points(current_start_.first, current_start_.second + 1));
  }
}

//
void Astar::AddObstaclePoint(Points point) {
  ID_SIZE id = calculate_id(point);
  if (IsInMap(id, map_obstacle_list_)) current_obstacle_list_[id] = point;
}

//判断起点的车身角方向
Yaw_type Astar::JudgeStartYaw(const Points& start, const Points& goal) {
  if (abs(goal.second - start.second) > abs(goal.first - start.first)) {
    if (goal.second > start.second)
      return M_PI_2;
    else
      return -M_PI_2;
  } else {
    if (goal.first > start.first)
      return 0.0;
    else
      return M_PI;
  }
}

//一次A*算法，以当前起点为起点
void Astar::AstarGetPath() {
  {
    std::priority_queue<CellInfo> open_list;  //存放将要遍历的点
    std::map<ID_SIZE, Points>
        close_list;  //存放已经遍历的点以及已知的地图障碍点
    std::map<Points, Points> save_path_hash;  //用于路径回溯
    std::map<Points, Yaw_type> save_point_info;
    std::list<Points> path_result_list;  //存放结果

    //初始化起点的信息
    CellInfo start_info = {0, 0, current_start_,
                           JudgeStartYaw(current_start_, goal_pos_)};
    int search_successful_flg = 1;
    current_expand_points_count_ = 0;

    open_list.push(start_info);  //起点入队列

    while (!open_list.empty()) {
      CellInfo current_cell_pos = open_list.top();
      open_list.pop();
      //找到终点，一次算法结束
      if (current_cell_pos.state.xoy == goal_pos_) {
        if (current_cell_pos.state.yaw == start_info.state.yaw) {
          search_successful_flg = 0;  //搜索成功,保证yaw与起始点同向
          save_point_info[goal_pos_] = current_cell_pos.state.yaw;
          break;
        } else {
          continue;
        }

        // search_successful_flg = 0;  //搜索成功
        // break;
      }

      //如果不在closelist中，可以expand
      if (!IsInMap(current_cell_pos.state.xoy, close_list)) {
        close_list[calculate_id(current_cell_pos.state.xoy)] =
            current_cell_pos.state.xoy;
        save_point_info[current_cell_pos.state.xoy] =
            current_cell_pos.state.yaw;
        std::cout << std::endl
                  << "current_cell_pos : (" << current_cell_pos.state.xoy.first
                  << "," << current_cell_pos.state.xoy.second
                  << ")  yaw : " << current_cell_pos.state.yaw;
        //
        std::vector<CellInfo> neighbors = GetNeighbors(current_cell_pos.state);
        std::cout << "neighbors : ";
        for (auto& elem : neighbors) {
          std::cout << "(" << elem.state.xoy.first << ","
                    << elem.state.xoy.second << ")  yaw : " << elem.state.yaw
                    << std::endl;
        }

        int8_t neighbor_expand_cnt = 0;
        for (int i = 0; i < neighbors.size(); ++i) {
          if (!IsInMap(neighbors[i].state.xoy, close_list)) {
            ++neighbor_expand_cnt;

            // g(n)
            neighbors[i].cost_to_start_ =
                current_cell_pos.cost_to_start_ +
                DisBetweenTwoPoint(current_cell_pos.state.xoy,
                                   neighbors[i].state.xoy);
            // f(n)=g(n)+h(n)
            neighbors[i].all_cost_ = neighbors[i].cost_to_start_ +
                                     DistenceToGoal(neighbors[i].state.xoy);
            open_list.push(neighbors[i]);
            save_path_hash[neighbors[i].state.xoy] = current_cell_pos.state.xoy;
            // save_point_info[neighbors[i].state.xoy] = neighbors[i].state;
          }
        }
        if (neighbor_expand_cnt) ++current_expand_points_count_;  //扩展点自曾
      }
    }

    if (search_successful_flg) {
      std::cout << "search fail !!" << std::endl;
    }
    // nope path to goal
    else {
      std::cout << "search successfully !!" << std::endl;
      Points node = goal_pos_;
      //得到最短路径的坐标向量
      while (node != current_start_) {
        path_result_list.push_back(node);
        node = save_path_hash[node];
        std::cout << "(" << node.first << " , " << node.second << ")"
                  << std::endl;
      }
    }

    all_expand_points_count_ += current_expand_points_count_;  // expand计数累加

    current_path_.clear();
    for (auto& elem : path_result_list) {
      current_path_.push_back({elem, save_point_info[elem]});
    }
    // current_path_ = path_result_list;

    SearchResultPrint();

    ++search_nums_count_;  //搜索次数自增1
  }
}

//**打印一次搜索结果**
//输入：无
//输出：无
void Astar::SearchResultPrint() {
  for (int i = 0; i < row_; ++i) {
    for (int j = 0; j < column_; ++j) {
      if (current_start_.first == i && current_start_.second == j)
        std::cout << "s ";

      else if (goal_pos_.first == i && goal_pos_.second == j)
        std::cout << "g ";

      else if (IsInMap(Points(i, j), current_obstacle_list_))
        std::cout << "x ";

      else if (IsInList(Points(i, j), current_path_))
        std::cout << "o ";
      else
        std::cout << "_ ";
    }
    std::cout << std::endl;
  }
  std::cout << "shortest path step nums : " << current_path_.size()
            << "    expand point nums : " << current_expand_points_count_
            << std::endl;

  std::cout << std::endl << std::endl;
}

//**打印计数结果**
//输入：无
//输出：无
void Astar::PrintCountResult() {
  std::cout << std::endl
            << "The nums of search : " << search_nums_count_
            << "  total expanded nums : " << all_expand_points_count_
            << std::endl
            << std::endl;
}

/* 打印统计结果
 * 输入：无
 * 输出：无
 *  */
void PrintSumResult() {
  std::cout << std::endl
            << "-——-——-——-——-——-——-——-——-***-——-——-——-——-——-——-——-——-——-"
            << std::endl
            << "-——                   Sum  Result                    ——-"
            << std::endl
            << "-——-——-——-——-——-——-——-——-***-——-——-——-——-——-——-——-——-——-"
            << std::endl;
  std::cout << "| map num | search nums | expand nums | move_step nums |"
            << std::endl;
  for (int16_t i = 0; i < sum_result.size(); ++i) {
    std::cout << sum_result[i] << std::endl;
  }
}