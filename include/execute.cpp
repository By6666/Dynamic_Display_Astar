#include "execute.h"

bool SearchOneMap(int map_num) {
  //获得map信息
  GrideInput map_info(map_num);
  map_info.GetOneGrid();
  map_info.PrintMap();  //打印原始map

  bool find_goal_flg = false;

  //数据传入，构造类对象
  Astar Astar_algorithm(map_info.get_grid_rows(), map_info.get_grid_columns(),
                        map_info.get_start_pos(), map_info.get_goal_pos(),
                        map_info.get_obstacle_pos());

  if (Astar_algorithm.Execute()) {
    std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
    std::cout << "|final result: get goal successflly!!|" << std::endl;
    std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
    find_goal_flg = true;
  } else {
    std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
    std::cout << "|final result : no path to goal !!|" << std::endl;
    std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
  }
  Astar_algorithm.PrintResult();

  /* opencv draw */
  if (find_goal_flg)
    DrawWholeMap(map_num, Astar_algorithm.get_row(), Astar_algorithm.get_col(),
                 Astar_algorithm.get_start(), Astar_algorithm.get_goal(),
                 Astar_algorithm.get_obstacle(),
                 Astar_algorithm.get_final_path());

  return find_goal_flg;
}