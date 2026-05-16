#include "replanner/dstar_lite.hpp"

DStarLite::DStarLite(int width, int height, float resolution,
                     float origin_x, float origin_y)
: width_(width), height_(height), resolution_(resolution),
  origin_x_(origin_x), origin_y_(origin_y), k_m_(0.0)
{
  g_.resize(width_ * height_, INF);
  rhs_.resize(width_ * height_, INF);
}

int DStarLite::toIndex(int x, int y)
{
  return y * width_ + x;
}

double DStarLite::heuristic(int x1, int y1, int x2, int y2)
{
  return std::sqrt(std::pow(x2-x1,2) + std::pow(y2-y1,2));
}

bool DStarLite::isValid(int x, int y)
{
  return x >= 0 && x < width_ && y >= 0 && y < height_;
}

bool DStarLite::isObstacle(int x, int y)
{
  int inflation = static_cast<int>(0.3 / resolution_);
  if (inflation < 1) inflation = 1;

  for (int dx = -inflation; dx <= inflation; dx++) {
    for (int dy = -inflation; dy <= inflation; dy++) {
      int nx = x + dx;
      int ny = y + dy;
      if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) return true;
      int idx = toIndex(nx, ny);
      if (idx < 0 || idx >= (int)grid_.size()) return true;
      if (grid_[idx] >= 50 || grid_[idx] < 0) return true;
    }
  }
  return false;
}

std::pair<int,int> DStarLite::worldToGrid(double wx, double wy)
{
  return {
    static_cast<int>((wx - origin_x_) / resolution_),
    static_cast<int>((wy - origin_y_) / resolution_)
  };
}

std::pair<double,double> DStarLite::gridToWorld(int gx, int gy)
{
  return {
    gx * resolution_ + origin_x_,
    gy * resolution_ + origin_y_
  };
}

std::vector<std::pair<int,int>> DStarLite::getNeighbors(int x, int y)
{
  std::vector<std::pair<int,int>> neighbors;
  int dx[] = {0, 0, 1, -1, 1, 1, -1, -1};
  int dy[] = {1, -1, 0, 0, 1, -1, 1, -1};
  for (int i = 0; i < 8; i++) {
    int nx = x + dx[i];
    int ny = y + dy[i];
    if (isValid(nx, ny)) {
      neighbors.push_back({nx, ny});
    }
  }
  return neighbors;
}

double DStarLite::moveCost(int x1, int y1, int x2, int y2)
{
  if (isObstacle(x2, y2)) return INF;
  int dx = std::abs(x2 - x1);
  int dy = std::abs(y2 - y1);
  return (dx + dy == 2) ? 1.414 : 1.0;
}

std::pair<double,double> DStarLite::calculateKey(int x, int y)
{
  int idx = toIndex(x, y);
  double min_g_rhs = std::min(g_[idx], rhs_[idx]);
  return {
    min_g_rhs + heuristic(start_.first, start_.second, x, y) + k_m_,
    min_g_rhs
  };
}

void DStarLite::updateVertex(int x, int y)
{
  int idx = toIndex(x, y);

  if (x != goal_.first || y != goal_.second) {
    double min_rhs = INF;
    for (auto & n : getNeighbors(x, y)) {
      double cost = moveCost(n.first, n.second, x, y) +
                    g_[toIndex(n.first, n.second)];
      min_rhs = std::min(min_rhs, cost);
    }
    rhs_[idx] = min_rhs;
  }

  if (std::abs(g_[idx] - rhs_[idx]) > 1e-9) {
    auto key = calculateKey(x, y);
    DNode node;
    node.x = x; node.y = y;
    node.k1 = key.first; node.k2 = key.second;
    open_list_.push(node);
  }
}

void DStarLite::computeShortestPath()
{
  while (!open_list_.empty()) {
    DNode u = open_list_.top();
    open_list_.pop();

    int u_idx = toIndex(u.x, u.y);

    if (g_[u_idx] == rhs_[u_idx]) continue;

    if (g_[u_idx] > rhs_[u_idx]) {
      g_[u_idx] = rhs_[u_idx];
      for (auto & n : getNeighbors(u.x, u.y)) {
        updateVertex(n.first, n.second);
      }
    } else {
      g_[u_idx] = INF;
      updateVertex(u.x, u.y);
      for (auto & n : getNeighbors(u.x, u.y)) {
        updateVertex(n.first, n.second);
      }
    }

    int s_idx = toIndex(start_.first, start_.second);
    if (g_[s_idx] == rhs_[s_idx] && g_[s_idx] < INF) break;
  }
}

void DStarLite::initialize(
  std::pair<int,int> start,
  std::pair<int,int> goal,
  const std::vector<int8_t> & grid)
{
  start_ = start;
  goal_  = goal;
  grid_  = grid;
  k_m_   = 0.0;

  std::fill(g_.begin(),   g_.end(),   INF);
  std::fill(rhs_.begin(), rhs_.end(), INF);

  while (!open_list_.empty()) open_list_.pop();

  int goal_idx = toIndex(goal_.first, goal_.second);
  rhs_[goal_idx] = 0.0;

  auto key = calculateKey(goal_.first, goal_.second);
  DNode goal_node;
  goal_node.x  = goal_.first;
  goal_node.y  = goal_.second;
  goal_node.k1 = key.first;
  goal_node.k2 = key.second;
  open_list_.push(goal_node);

  computeShortestPath();
}

void DStarLite::updateGrid(const std::vector<int8_t> & new_grid)
{
  std::vector<std::pair<int,int>> changed;

  for (int y = 0; y < height_; y++) {
    for (int x = 0; x < width_; x++) {
      int idx = toIndex(x, y);
      if (grid_[idx] != new_grid[idx]) {
        grid_[idx] = new_grid[idx];
        changed.push_back({x, y});
      }
    }
  }

  for (auto & c : changed) {
    int idx = toIndex(c.first, c.second);
    g_[idx]   = INF;
    rhs_[idx] = INF;
    updateVertex(c.first, c.second);
    for (auto & n : getNeighbors(c.first, c.second)) {
      updateVertex(n.first, n.second);
    }
  }

  computeShortestPath();
}
std::vector<std::pair<int,int>> DStarLite::getPath(std::pair<int,int> current)
{
  std::vector<std::pair<int,int>> path;
  path.push_back(current);

  int max_steps = width_ * height_;
  int steps = 0;

  while ((current.first != goal_.first || current.second != goal_.second)
         && steps < max_steps)
  {
    steps++;
    double min_cost = INF;
    std::pair<int,int> best = current;

    for (auto & n : getNeighbors(current.first, current.second)) {
      if (isObstacle(n.first, n.second)) continue;
      double cost = moveCost(current.first, current.second, n.first, n.second)
                  + g_[toIndex(n.first, n.second)];
      if (cost < min_cost) {
        min_cost = cost;
        best = n;
      }
    }

    if (best == current || min_cost >= INF) break;
    current = best;
    path.push_back(current);
  }

  return smoothPath(path);
}

std::vector<std::pair<int,int>> DStarLite::smoothPath(
  const std::vector<std::pair<int,int>> & path)
{
  if (path.size() < 3) return path;

  std::vector<std::pair<int,int>> smoothed;
  smoothed.push_back(path[0]);

  size_t current = 0;

  while (current < path.size() - 1) {
    size_t farthest = current + 1;

    for (size_t next = current + 2; next < path.size() && next <= current + 5; next++) {
      int x0 = path[current].first;
      int y0 = path[current].second;
      int x1 = path[next].first;
      int y1 = path[next].second;

      bool clear = true;
      int ddx = std::abs(x1 - x0);
      int ddy = std::abs(y1 - y0);
      int sx = (x0 < x1) ? 1 : -1;
      int sy = (y0 < y1) ? 1 : -1;
      int err = ddx - ddy;

      int cx = x0, cy = y0;
      while (cx != x1 || cy != y1) {
        if (isObstacle(cx, cy)) { clear = false; break; }
        int e2 = 2 * err;
        if (e2 > -ddy) { err -= ddy; cx += sx; }
        if (e2 <  ddx) { err += ddx; cy += sy; }
      }

      if (clear) farthest = next;
    }

    smoothed.push_back(path[farthest]);
    current = farthest;
  }

  return smoothed;
}
