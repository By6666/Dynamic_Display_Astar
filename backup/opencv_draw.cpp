#include "opencv_draw.h"
//边长与放大倍数
const uint16_t edge_length = 30;

void DrawMap(int num, int16_t row, int16_t col, const Points& start,
             const Points& goal, const std::map<ID_SIZE, Points>& obstacle,
             const std::list<DriveStateInfo>& path) {
  //网格线宽
  const int line_thickness = 1;
  std::string title = "map - " + std::to_string(num);
  cv::Mat img(row * edge_length + line_thickness,
              col * edge_length + line_thickness, CV_8UC3,
              cv::Scalar::all(255));
  // grid line
  for (int16_t i = 0; i <= col; ++i) {
    cv::line(img, cv::Point(i * edge_length, 0),
             cv::Point(i * edge_length, row * edge_length), cv::Scalar::all(0),
             line_thickness);
  }
  for (int16_t i = 0; i <= row; ++i) {
    cv::line(img, cv::Point(0, i * edge_length),
             cv::Point(col * edge_length, i * edge_length), cv::Scalar::all(0),
             line_thickness);
  }

  // star -> green
  cv::circle(
      img,
      cv::Point(static_cast<int>(start.second * edge_length) + edge_length / 2,
                static_cast<int>(start.first * edge_length) + edge_length / 2),
      5, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);

  DrawCar(cv::Point(start.first * edge_length + edge_length / 2,
                    start.second * edge_length + edge_length / 2),
          path.front().yaw, img);

  // goal -> red
  cv::rectangle(
      img,
      cv::Rect(static_cast<int>(goal.second * edge_length) + line_thickness,
               static_cast<int>(goal.first * edge_length) + line_thickness,
               edge_length - line_thickness, edge_length - line_thickness),
      cv::Scalar(0, 0, 255), cv::FILLED);

  // obstacle -> black
  for (auto& elem : obstacle) {
    cv::rectangle(
        img,
        cv::Rect(
            static_cast<int>(elem.second.second * edge_length) + line_thickness,
            static_cast<int>(elem.second.first * edge_length) + line_thickness,
            edge_length - line_thickness, edge_length - line_thickness),
        cv::Scalar::all(0), cv::FILLED);
  }

  std::vector<cv::Point> change_path;
  change_path.reserve(path.size());

  for (auto& elem : path) {
    change_path.push_back(
        cv::Point(elem.xoy.first * edge_length + edge_length / 2,
                  elem.xoy.second * edge_length + edge_length / 2));
  }

  std::cout << "-------------------123-------------------" << std::endl;
  std::vector<cv::Point2d> final_path;
  std::vector<Yaw_type> path_yaw;
  // bool continue_flg = false;
  for (int16_t i = 1; i < change_path.size() - 1; ++i) {
    if (calculate_arct2(change_path[i - 1], change_path[i]) ==
        calculate_arct2(change_path[i], change_path[i + 1])) {
      final_path.push_back(change_path[i - 1]);
      path_yaw.push_back(calculate_arct2(change_path[i - 1], change_path[i]));
    } else {
      int16_t last_conor = i;
      double last_yaw = calculate_arct2(change_path[i - 1], change_path[i]);
      while (1) {
        ++i;
        if (i == change_path.size()) break;
        if (last_yaw == calculate_arct2(change_path[i - 1], change_path[i]))
          break;
      }
      ThridOrder((i - last_conor) * 15, change_path[last_conor - 1],
                 change_path[last_conor], change_path[i - 1], change_path[i],
                 final_path, path_yaw);
    }
  }
  std::cout << "-------------------456-------------------" << std::endl;
  final_path.push_back(change_path.back());
  path_yaw.push_back(path.back().yaw);

  for (auto& elem : final_path) {
    cv::circle(img, cv::Point2d(elem.y, elem.x), 2, cv::Scalar(255, 200, 0), -1,
               cv::LINE_AA);
  }

  // cv::polylines(img, change_path, false, cv::Scalar(255, 0, 0), 1,
  // cv::LINE_AA);

  for (int16_t i = 0; i < final_path.size(); ++i) {
    DrawCar(final_path[i], path_yaw[i], img);
    std::cout << final_path[i] << "  " << path_yaw[i] << std::endl;
  }

  cv::namedWindow(title, CV_WINDOW_NORMAL);
  cv::imshow(title, img);

  cv::waitKey(0);
  cv::destroyWindow(title);
}

void DrawOneMap(int num) {
  GrideInput grid_info(num);
  grid_info.GetOneGrid();
  grid_info.PrintMap();
  std::list<DriveStateInfo> path;
  DrawMap(num, grid_info.get_grid_rows(), grid_info.get_grid_columns(),
          grid_info.get_start_pos(), grid_info.get_goal_pos(),
          grid_info.get_obstacle_pos(), path);
}

void DrawCar(const cv::Point2d& central_point, Yaw_type yaw,
             const cv::Mat& img) {
  cv::Point2d front_centre, rear_centre;
  cv::Point2d A, B, C, D;
  const int line_thinckness = 1;

  std::vector<cv::Point2d> stg;
  stg.reserve(4);

  A.x = CarLength / 2.0 - line_thinckness;
  A.y = CarWidth / 2.0 - line_thinckness;

  B.x = CarLength / 2.0 - line_thinckness;
  B.y = -CarWidth / 2.0 + line_thinckness;

  C.x = -CarLength / 2.0 + line_thinckness;
  C.y = -CarWidth / 2.0 + line_thinckness;

  D.x = -CarLength / 2.0 + line_thinckness;
  D.y = CarWidth / 2.0 - line_thinckness;

  stg.push_back(A);
  stg.push_back(B);
  stg.push_back(C);
  stg.push_back(D);

  RotateCar(stg, central_point, yaw);

  // front_centre.x = central_point.x +
  //                  (CarLength - line_thinckness * 2) / 2.0 * cos(yaw);  // x
  // front_centre.y = central_point.y +
  //                  (CarLength - line_thinckness * 2) / 2.0 * sin(yaw);  // y

  // rear_centre.x = central_point.x -
  //                 (CarLength - line_thinckness * 2) / 2.0 * cos(yaw);  // x
  // rear_centre.y = central_point.y -
  //                 (CarLength - line_thinckness * 2) / 2.0 * sin(yaw);  // y

  // A.x = front_centre.x - (CarWidth - line_thinckness * 2) / 2.0 * sin(yaw);
  // A.y = front_centre.y + (CarWidth - line_thinckness * 2) / 2.0 * cos(yaw);

  // B.x = front_centre.x + (CarWidth - line_thinckness * 2) / 2.0 * sin(yaw);
  // B.y = front_centre.y - (CarWidth - line_thinckness * 2) / 2.0 * cos(yaw);

  // C.x = rear_centre.x + (CarWidth - line_thinckness * 2) / 2.0 * sin(yaw);
  // C.y = rear_centre.y - (CarWidth - line_thinckness * 2) / 2.0 * cos(yaw);

  // D.x = rear_centre.x - (CarWidth - line_thinckness * 2) / 2.0 * sin(yaw);
  // D.y = rear_centre.y + (CarWidth - line_thinckness * 2) / 2.0 * cos(yaw);

  // std::vector<cv::Point2d> stg;
  // stg.push_back(A);
  // stg.push_back(B);
  // stg.push_back(C);
  // stg.push_back(D);

  for (int i = 0; i < 4; i++) {
    cv::line(img, cv::Point2f(stg[i].y, stg[i].x),
             cv::Point2f(stg[(i + 1) % 4].y, stg[(i + 1) % 4].x),
             cv::Scalar(255, 0, 0), line_thinckness, cv::LINE_AA);
    // std::cout << stg[i] << std::endl;
  }
  // for (auto& elem : stg) {
  //   std::swap(elem.x, elem.y);
  // }
  // cv::polylines(img, stg, true, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
  cv::circle(img, cv::Point2f(stg[0].y, stg[0].x), 2, cv::Scalar(255, 0, 255),
             -1, cv::LINE_AA);

  // cv::Point2f vertices[4];
  // cv::Point2f pos;
  // cv::RotatedRect rRect(central_point, cv::Size2f(CarWidth, CarLength),
  //                       yaw * 57.3);
  // rRect.points(vertices);

  // for (int i = 0; i < 4; i++) {
  //   cv::line(img, cv::Point2f(vertices[i].y, vertices[i].x),
  //            cv::Point2f(vertices[(i + 1) % 4].y, vertices[(i + 1) % 4].x),
  //            cv::Scalar(0, 255, 0), 1);
  //   std::cout << vertices[i] << std::endl;
  // }
  // cv::circle(img, cv::Point2f(vertices[0].y, vertices[0].x), 2,
  //            cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
}

void RotateCar(std::vector<cv::Point2d>& car, const cv::Point2d& central_point,
               Yaw_type yaw) {
  for (auto& elem : car) {
    cv::Point2d temp = elem;
    // std::cout << " primary rotate elem : " << elem << std::endl;
    elem.x = temp.x * cos(yaw) - temp.y * sin(yaw) + central_point.x;
    elem.y = temp.x * sin(yaw) + temp.y * cos(yaw) + central_point.y;
    // std::cout << " rotate elem : " << elem << std::endl;
  }
}