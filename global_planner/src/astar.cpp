#include "global_planner/astar.hpp"

AStar::AStar(int width, int height, float resolution)
: width_(width), height_(height), resolution_(resolution) {}

int AStar::toIndex(int x, int y)
{
  return y * width_ + x;
}

float AStar::heuristic(int x1, int y1, int x2, int y2)
{
  return std::sqrt(
    std::pow(static_cast<float>(x2 - x1), 2) +
    std::pow(static_cast<float>(y2 - y1), 2)
  );
}

bool AStar::isValid(int x, int y, const std::vector<int8_t> & grid)
{
  int inflation = static_cast<int>(0.3 / resolution_);
  if (inflation < 1) inflation = 1;

  for (int dx = -inflation; dx <= inflation; dx++) {
    for (int dy = -inflation; dy <= inflation; dy++) {
      int nx = x + dx;
      int ny = y + dy;
      if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) return false;
      int8_t cell = grid[toIndex(nx, ny)];
      if (cell < 0 || cell >= 50) return false;
    }
  }
  return true;
}

std::vector<std::pair<int,int>> AStar::smoothPath(
  const std::vector<std::pair<int,int>> & path,
  const std::vector<int8_t> & grid)
{
  if (path.size() < 3) return path;

  std::vector<std::pair<int,int>> smoothed;
  smoothed.push_back(path[0]);

  size_t current = 0;

  while (current < path.size() - 1) {
    size_t farthest = current + 1;

    for (size_t next = current + 2; next < path.size(); next++) {
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
        if (!isValid(cx, cy, grid)) { clear = false; break; }
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

std::vector<std::pair<int,int>> AStar::plan(
  std::pair<int,int> start,
  std::pair<int,int> goal,
  const std::vector<int8_t> & grid)
{
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
  std::unordered_map<int, Node> all_nodes;
  std::unordered_map<int, bool> closed_list;

  Node start_node;
  start_node.x = start.first;
  start_node.y = start.second;
  start_node.g = 0.0f;
  start_node.h = heuristic(start.first, start.second, goal.first, goal.second);
  start_node.f = start_node.g + start_node.h;
  start_node.parent_x = -1;
  start_node.parent_y = -1;

  open_list.push(start_node);
  all_nodes[toIndex(start.first, start.second)] = start_node;

  int dx[] = { 0,  0,  1, -1,  1,  1, -1, -1};
  int dy[] = { 1, -1,  0,  0,  1, -1,  1, -1};
  float move_cost[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.414f, 1.414f, 1.414f, 1.414f};

  while (!open_list.empty()) {
    Node current = open_list.top();
    open_list.pop();

    int curr_idx = toIndex(current.x, current.y);
    if (closed_list.count(curr_idx) && closed_list[curr_idx]) continue;
    closed_list[curr_idx] = true;

    if (current.x == goal.first && current.y == goal.second) {
      std::vector<std::pair<int,int>> path;
      Node n = current;

      while (n.parent_x != -1 && n.parent_y != -1) {
        path.push_back({n.x, n.y});
        int parent_idx = toIndex(n.parent_x, n.parent_y);
        n = all_nodes[parent_idx];
      }

      path.push_back({start.first, start.second});
      std::reverse(path.begin(), path.end());
      return smoothPath(path, grid);
    }

    for (int i = 0; i < 8; i++) {
      int nx = current.x + dx[i];
      int ny = current.y + dy[i];

      if (!isValid(nx, ny, grid)) continue;

      int n_idx = toIndex(nx, ny);
      if (closed_list.count(n_idx) && closed_list[n_idx]) continue;

      float new_g = current.g + move_cost[i];

      if (all_nodes.find(n_idx) == all_nodes.end() ||
          new_g < all_nodes[n_idx].g)
      {
        Node neighbor;
        neighbor.x = nx;
        neighbor.y = ny;
        neighbor.g = new_g;
        neighbor.h = heuristic(nx, ny, goal.first, goal.second);
        neighbor.f = neighbor.g + neighbor.h;
        neighbor.parent_x = current.x;
        neighbor.parent_y = current.y;

        all_nodes[n_idx] = neighbor;
        open_list.push(neighbor);
      }
    }
  }

  return {};
}
