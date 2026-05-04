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

  std::cout << "Plan basliyor:" << std::endl;
  std::cout << "  Start: x=" << sx << " y=" << sy << " theta=" << st << std::endl;
  std::cout << "  Goal:  x=" << gx << " y=" << gy << std::endl;
  std::cout << "  Grid boyutu: " << grid.size() << std::endl;
  std::cout << "  Width=" << width_ << " Height=" << height_ << std::endl;
  std::cout << "  Resolution=" << resolution_ << std::endl;

  // Başlangıç ve hedef geçerli mi?
  std::cout << "Start gecerli mi: " << isValid(sx, sy, grid) << std::endl;
  std::cout << "Goal gecerli mi: "  << isValid(gx, gy, grid) << std::endl;

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

    // İlk 5 iterasyonu debug et
    if (iteration <= 5) {
      std::cout << "Iterasyon " << iteration
                << " open_list boyutu: " << open_list.size() << std::endl;
    }

    HNode current = open_list.top();
    open_list.pop();

    std::string curr_key = toKey(current.x, current.y, current.theta);

    if (closed_list.count(curr_key) && closed_list[curr_key]) continue;
    closed_list[curr_key] = true;

    double dist = heuristic(current.x, current.y, gx, gy);
    if (dist < goal_thresh) {
      std::cout << "Hedefe ulasildi! Iterasyon: " << iteration << std::endl;
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
      return path;
    }

    for (double steer : steers) {
      HNode next = motion(current, steer, params_.step_size);
      if (!isValid(next.x, next.y, grid)) continue;
      std::string next_key = toKey(next.x, next.y, next.theta);
      if (closed_list.count(next_key) && closed_list[next_key]) continue;
      double new_g = current.g + params_.step_size;
      next.g = new_g;
      next.h = heuristic(next.x, next.y, gx, gy);
      next.f = next.g + next.h;
      if (all_nodes.find(next_key) == all_nodes.end() ||
          new_g < all_nodes[next_key].g)
      {
        all_nodes[next_key] = next;
        open_list.push(next);
      }
    }
  }

  std::cout << "Rota bulunamadi. Toplam iterasyon: " << iteration << std::endl;
  return {};
}
