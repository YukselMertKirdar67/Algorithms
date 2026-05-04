#ifndef GLOBAL_PLANNER__ASTAR_HPP_
#define GLOBAL_PLANNER__ASTAR_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

struct Node {
  int x, y;
  float g, h, f;
  int parent_x, parent_y;

  bool operator>(const Node & other) const {
    return f > other.f;
  }
};

class AStar {
public:
  AStar(int width, int height, float resolution);

  // Ana planlama fonksiyonu
  // start: başlangıç grid koordinatı
  // goal: hedef grid koordinatı
  // grid: OccupancyGrid verisi
  // return: waypoint listesi (grid koordinatları)
  std::vector<std::pair<int,int>> plan(
    std::pair<int,int> start,
    std::pair<int,int> goal,
    const std::vector<int8_t> & grid
  );

private:
  int width_;
  int height_;
  float resolution_;

  // Öklid heuristik fonksiyonu
  float heuristic(int x1, int y1, int x2, int y2);

  // Grid sınır ve engel kontrolü
  bool isValid(int x, int y, const std::vector<int8_t> & grid);

  // 2D koordinatı 1D index'e çevir
  int toIndex(int x, int y);
};

#endif
