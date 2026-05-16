#include "rclcpp/rclcpp.hpp"
#include "local_planner/hybrid_astar.hpp"
#include <sstream>
#include <iomanip>
#include <iostream>

HybridAStar::HybridAStar(
  int width, int height,
  float resolution,
  float origin_x, float origin_y,
  VehicleParams params)
: width_(width), height_(height),
  resolution_(resolution),
  origin_x_(origin_x), origin_y_(origin_y),
  params_(params) {}

double HybridAStar::heuristic(double x1, double y1, double x2, double y2)
{
  return std::sqrt(
    std::pow(x2 - x1, 2) +
    std::pow(y2 - y1, 2)
  );
}

int HybridAStar::toIndex(double x, double y)
{
  int gx = static_cast<int>((x - origin_x_) / resolution_);
  int gy = static_cast<int>((y - origin_y_) / resolution_);
  return gy * width_ + gx;
}

std::string HybridAStar::toKey(double x, double y, double theta)
{
  std::ostringstream oss;
  oss << static_cast<int>(x / 0.1) << ","
      << static_cast<int>(y / 0.1) << ","
      << static_cast<int>(theta / (10.0 * M_PI / 180.0));
  return oss.str();
}

bool HybridAStar::isValid(double x, double y, const std::vector<int8_t> & grid)
{
  int inflation = static_cast<int>(0.3 / resolution_);
  if (inflation < 1) inflation = 1;

  int cx = static_cast<int>((x - origin_x_) / resolution_);
  int cy = static_cast<int>((y - origin_y_) / resolution_);

  for (int dx = -inflation; dx <= inflation; dx++) {
    for (int dy = -inflation; dy <= inflation; dy++) {
      int gx = cx + dx;
      int gy = cy + dy;

      if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) return false;

      int idx = gy * width_ + gx;
      if (idx < 0 || idx >= (int)grid.size()) return false;

      int8_t cell = grid[idx];
      if (cell < 0 || cell >= 50) return false;
    }
  }
  return true;
}

HNode HybridAStar::motion(const HNode & current, double steer, double step)
{
  HNode next;
  next.x     = current.x + step * std::cos(current.theta);
  next.y     = current.y + step * std::sin(current.theta);
  next.theta = current.theta +
    (step / params_.wheelbase) * std::tan(steer);

  while (next.theta >  M_PI) next.theta -= 2.0 * M_PI;
  while (next.theta < -M_PI) next.theta += 2.0 * M_PI;

  next.parent_x     = current.x;
  next.parent_y     = current.y;
  next.parent_theta = current.theta;
  next.valid_parent = true;

  return next;
}

std::vector<std::tuple<double,double,double>> HybridAStar::plan(
  std::tuple<double,double,double> start,
  std::tuple<double,double,double> goal,
  const std::vector<int8_t> & grid)
{
  double sx = std::get<0>(start);
  double sy = std::get<1>(start);
  double st = std::get<2>(start);

  double gx = std::get<0>(goal);
  double gy = std::get<1>(goal);

  RCLCPP_INFO(rclcpp::get_logger("hybrid_astar"),
    "Plan basliyor: start=(%.2f,%.2f,%.2f) goal=(%.2f,%.2f)",
    sx, sy, st, gx, gy);

  if (!isValid(sx, sy, grid)) {
    RCLCPP_WARN(rclcpp::get_logger("hybrid_astar"), "Baslangic noktasi gecersiz!");
    return {};
  }
  if (!isValid(gx, gy, grid)) {
    RCLCPP_WARN(rclcpp::get_logger("hybrid_astar"), "Hedef noktasi gecersiz!");
    return {};
  }

  std::priority_queue<HNode, std::vector<HNode>, std::greater<HNode>> open_list;
  std::unordered_map<std::string, HNode> all_nodes;
  std::unordered_map<std::string, bool>  closed_list;

  HNode start_node;
  start_node.x     = sx;
  start_node.y     = sy;
  start_node.theta = st;
  start_node.g     = 0.0;
  start_node.h     = heuristic(sx, sy, gx, gy);
  start_node.f     = start_node.g + start_node.h;
  start_node.valid_parent = false;

  std::string start_key = toKey(sx, sy, st);
  open_list.push(start_node);
  all_nodes[start_key] = start_node;

  std::vector<double> steers;
  int steps = params_.steer_steps;
  for (int i = -steps; i <= steps; i++) {
    steers.push_back(
      params_.max_steer * static_cast<double>(i) / static_cast<double>(steps));
  }

  double goal_thresh = goal_thresh_;
  int iteration = 0;

  while (!open_list.empty()) {
    iteration++;

    HNode current = open_list.top();
    open_list.pop();

    std::string curr_key = toKey(current.x, current.y, current.theta);

    if (closed_list.count(curr_key) && closed_list[curr_key]) continue;
    closed_list[curr_key] = true;

    double dist = heuristic(current.x, current.y, gx, gy);
    if (dist < goal_thresh) {
      RCLCPP_INFO(rclcpp::get_logger("hybrid_astar"),
        "Hedefe ulasildi! Iterasyon: %d", iteration);

      std::vector<std::tuple<double,double,double>> path;
      HNode n = current;
      while (n.valid_parent) {
        path.push_back({n.x, n.y, n.theta});
        std::string parent_key = toKey(n.parent_x, n.parent_y, n.parent_theta);
        if (all_nodes.find(parent_key) == all_nodes.end()) break;
        n = all_nodes[parent_key];
      }
      path.push_back({sx, sy, st});
      std::reverse(path.begin(), path.end());
      return smoothPath(path, grid);
    }

    for (double steer : steers) {
      // Engele yakınlığa göre dinamik adım boyutu
         double dynamic_step = params_.step_size;
         int cx = static_cast<int>((current.x - origin_x_) / resolution_);
         int cy = static_cast<int>((current.y - origin_y_) / resolution_);
         int check_dist = 3; // 3 hücre yarıçapında engel var mı
         bool near_obstacle = false;
       for (int ddx = -check_dist; ddx <= check_dist && !near_obstacle; ddx++) {
        for (int ddy = -check_dist; ddy <= check_dist && !near_obstacle; ddy++) {
          int ngx = cx + ddx;
          int ngy = cy + ddy;
          if (ngx < 0 || ngx >= width_ || ngy < 0 || ngy >= height_) continue;
          int idx = ngy * width_ + ngx;
          if (grid[idx] >= 50 || grid[idx] < 0) near_obstacle = true;
        }
       }
      if (near_obstacle) dynamic_step = params_.step_size * 0.5;
      HNode next = motion(current, steer, dynamic_step);
      if (!isValid(next.x, next.y, grid)) continue;
      std::string next_key = toKey(next.x, next.y, next.theta);
      if (closed_list.count(next_key) && closed_list[next_key]) continue;
      double new_g = current.g + params_.step_size;
      next.g = new_g;
      next.h = 1.3 * heuristic(next.x, next.y, gx, gy);
      next.f = next.g + next.h;
      if (all_nodes.find(next_key) == all_nodes.end() ||
          new_g < all_nodes[next_key].g)
      {
        all_nodes[next_key] = next;
        open_list.push(next);
      }
    }
  }

  RCLCPP_WARN(rclcpp::get_logger("hybrid_astar"),
    "Rota bulunamadi. Toplam iterasyon: %d", iteration);
  return {};
}

std::vector<std::tuple<double,double,double>> HybridAStar::smoothPath(
  const std::vector<std::tuple<double,double,double>> & path,
  const std::vector<int8_t> & grid)
{
  if (path.size() < 3) return path;

  std::vector<std::tuple<double,double,double>> smoothed;
  smoothed.push_back(path[0]);

  size_t current = 0;

  while (current < path.size() - 1) {
    size_t farthest = current + 1;

    for (size_t next = current + 2; next < path.size() && next <= current + 3; next++) {
      double x0 = std::get<0>(path[current]);
      double y0 = std::get<1>(path[current]);
      double x1 = std::get<0>(path[next]);
      double y1 = std::get<1>(path[next]);

      // İki nokta arasında engel var mı kontrol et
      bool clear = true;
      int steps = static_cast<int>(std::sqrt(std::pow(x1-x0,2) + std::pow(y1-y0,2)) / resolution_) + 1;

      for (int s = 1; s < steps; s++) {
        double t = static_cast<double>(s) / steps;
        double cx = x0 + t * (x1 - x0);
        double cy = y0 + t * (y1 - y0);
        if (!isValid(cx, cy, grid)) {
          clear = false;
          break;
        }
      }

      if (clear) farthest = next;
    }

    smoothed.push_back(path[farthest]);
    current = farthest;
  }

  return smoothed;
}
