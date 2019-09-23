#include "Astar.h"

/* 构造函数 */
Astar::Astar(int row, int columns, Point2i_type start, Point2i_type goal,
             const std::set<ID_SIZE>& obstacle_list)
    : row_(row),
      col_(columns),
      start_pos_(start),
      goal_pos_(goal),
      obstacle_list_(obstacle_list) {
  expand_cnt_ = 0;

  /* init */
  Yaw_type start_yaw = JudgeStartYaw();

  all_cell_info_[CellPosState(CodeID(start_pos_, col_), start_yaw)] = {
      CellPosState(CodeID(start_pos_, col_), start_yaw),
      {0.0, GetHeuristicValue(start_pos_)},
      StateValue::NOT,
      CellPosState(-1, -1)};

  all_cell_info_[CellPosState(CodeID(goal_pos_, col_), start_yaw)] = {
      CellPosState(CodeID(goal_pos_, col_), start_yaw),
      {DBL_MAX, 0},
      StateValue::NOT,
      CellPosState(-1, -1)};

  start_ = &all_cell_info_[CellPosState(CodeID(start_pos_, col_), start_yaw)];
  goal_ = &all_cell_info_[CellPosState(CodeID(goal_pos_, col_), start_yaw)];
}

/* 判断起点车身yaw */
Yaw_type Astar::JudgeStartYaw() const {
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
}

/* 计算路径 */
bool Astar::ComputePath() {
  /* init */
  priority_que openlist;
  std::set<CellPosState> closelist;

  openlist.push({start_->posinfo, CalculateKey(start_)});  // push start

  while (!openlist.empty()) {
    CellInfo* cur_cell = NULL;
    cur_cell = &all_cell_info_[openlist.top().posinfo];
    openlist.pop();

    // std::cout << std::endl << "****   ****   current pos : ";
    // PrintPosState(cur_cell->posinfo);

    if (cur_cell->posinfo == goal_->posinfo) {
      return true;
    }
    if (cur_cell->inclose == StateValue::NOT) {
      SetStateValue(cur_cell, StateValue::IN);

      /* get neighbors */
      std::vector<CellPosState> neighbors = GetNeighbor(cur_cell->posinfo);
      // std::cout << "neighbors size : " << neighbors.size() << std::endl;
      int neighbors_cnt = 0;
      for (auto& elem : neighbors) {
        if (PushOpenlist(cur_cell, elem, openlist)) ++neighbors_cnt;
      }
      if (neighbors_cnt) ++expand_cnt_;
    }
  }
  return false;
}

/* push openlist */
bool Astar::PushOpenlist(const CellInfo* const cur_cell,
                         const CellPosState& neighbor, priority_que& openlist) {
  bool result = false;
  auto iter = all_cell_info_.find(neighbor);
  double g =
      cur_cell->value.g + TwoPosDistence(cur_cell->posinfo.id, neighbor.id);
  double h = GetHeuristicValue(DecodeID(neighbor.id, col_));

  if (iter == all_cell_info_.end()) {
    all_cell_info_[neighbor] =
        CellInfo{neighbor, {g, h}, StateValue::NOT, cur_cell->posinfo};

    openlist.push(CmpInfo{neighbor, CalculateKey(g, h)});
    result = true;
    // std::cout << "push openlist";
    // PrintPosState(neighbor);
  } else {
    if (iter->second.inclose == StateValue::NOT) {
      // if (CalculateKey(g, h) < CalculateKey(&(iter->second))) {
        iter->second.value.g = g;
        iter->second.value.h = h;
        iter->second.pre_best = cur_cell->posinfo;
        openlist.push(CmpInfo{neighbor, CalculateKey(g, h)});
        result = true;
        // std::cout << "push openlist";
        // PrintPosState(neighbor);
      // }
    }
  }
  return result;
}

/* 路径回溯 */
void Astar::RecurPath() {
  CellPosState node = goal_->posinfo;
  final_path_.push_front(node);

  while (node != start_->posinfo) {
    node = all_cell_info_[node].pre_best;
    final_path_.push_front(node);
    path_.push_back(node.id);
  }
}

/* 执行函数 */
bool Astar::Execute() {
  if (ComputePath()) {
    RecurPath();
    return true;
  }
  return false;
}

/* print result */
void Astar::PrintResult() {
  for (int i = 0; i < row_; ++i) {
    for (int j = 0; j < col_; ++j) {
      if (start_pos_.x == i && start_pos_.y == j)
        std::cout << "s ";

      else if (goal_pos_.x == i && goal_pos_.y == j)
        std::cout << "g ";

      else if (IsObstacle(i * col_ + j))
        std::cout << "x ";

      else if (IsInPath(i * col_ + j))
        std::cout << "o ";
      else
        std::cout << "_ ";
    }
    std::cout << std::endl;
  }
  std::cout << "shortest path step nums : " << path_.size()
            << "    expand point nums : " << expand_cnt_ << std::endl;

  std::cout << std::endl << std::endl;

  // std::cout << "path size: " << final_path_.size() << std::endl;
  // for (auto& elem : final_path_) {
  //   std::cout << DecodeID(elem.id, col_) << std::endl;
  // }
}