#ifndef REPLANNER__DSTAR_LITE_HPP_
#define REPLANNER__DSTAR_LITE_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <limits>
#include <unordered_set>

struct DNode {
  int x, y;
  double k1, k2; // Öncelik anahtarları

  bool operator>(const DNode & other) const {
    if (k1 != other.k1) return k1 > other.k1;
    return k2 > other.k2;
  }
};

class DStarLite {
public:
  DStarLite(int width, int height, float resolution,
            float origin_x, float origin_y);

  // İlk planlama — hedeften başlangıca arar
  void initialize(
    std::pair<int,int> start,
    std::pair<int,int> goal,
    const std::vector<int8_t> & grid);

  // Engel değiştiğinde güncelle
  void updateGrid(const std::vector<int8_t> & new_grid);

  // Mevcut konumdan hedefe rota döndür
  std::vector<std::pair<int,int>> getPath(std::pair<int,int> current);

  // Dünya koordinatı → grid koordinatı
  std::pair<int,int> worldToGrid(double wx, double wy);

  // Grid koordinatı → dünya koordinatı
  std::pair<double,double> gridToWorld(int gx, int gy);
  
  std::vector<std::pair<int,int>> smoothPath(
  const std::vector<std::pair<int,int>> & path);

private:
  int width_;
  int height_;
  float resolution_;
  float origin_x_;
  float origin_y_;

  std::pair<int,int> start_;
  std::pair<int,int> goal_;
  std::vector<int8_t> grid_;

  const double INF = std::numeric_limits<double>::infinity();

  // g: gerçek maliyet, rhs: tek adım lookahead maliyet
  std::vector<double> g_;
  std::vector<double> rhs_;

  std::priority_queue<DNode, std::vector<DNode>, std::greater<DNode>> open_list_;
  double k_m_; // Heuristic offset

  int toIndex(int x, int y);
  double heuristic(int x1, int y1, int x2, int y2);
  bool isValid(int x, int y);
  bool isObstacle(int x, int y);

  std::pair<double,double> calculateKey(int x, int y);
  void updateVertex(int x, int y);
  void computeShortestPath();

  // 8 yönlü komşular
  std::vector<std::pair<int,int>> getNeighbors(int x, int y);
  double moveCost(int x1, int y1, int x2, int y2);
};

#endif
