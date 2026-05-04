#ifndef LOCAL_PLANNER__HYBRID_ASTAR_HPP_
#define LOCAL_PLANNER__HYBRID_ASTAR_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

// Hybrid A* düğümü
// Standart A*'dan farkı: yönelim açısı (theta) da var
struct HNode {
  double x, y, theta;  // Araç konumu ve yönü
  double g, h, f;
  double parent_x, parent_y, parent_theta;
  bool valid_parent;

  bool operator>(const HNode & other) const {
    return f > other.f;
  }
};

// Araç kinematik parametreleri
struct VehicleParams {
  double wheelbase;       // Dingil mesafesi (metre)
  double max_steer;       // Maksimum direksiyon açısı (radyan)
  double step_size;       // Adım boyutu (metre)
  int steer_steps;        // Direksiyon açısı adım sayısı
};

class HybridAStar {
public:
  HybridAStar(
    int width, int height,
    float resolution,
    float origin_x, float origin_y,
    VehicleParams params);

  // Ana planlama fonksiyonu
  // start: başlangıç (x, y, theta) — dünya koordinatı
  // goal:  hedef  (x, y, theta) — dünya koordinatı
  // grid:  OccupancyGrid verisi
  std::vector<std::tuple<double,double,double>> plan(
    std::tuple<double,double,double> start,
    std::tuple<double,double,double> goal,
    const std::vector<int8_t> & grid
  );
  
  void setGoalThreshold(double threshold) { goal_thresh_ = threshold; }

private:
  int width_;
  int height_;
  float resolution_;
  VehicleParams params_;
  float origin_x_;
  float origin_y_;

  // Öklid heuristik
  double heuristic(double x1, double y1, double x2, double y2);

  // Grid sınır ve engel kontrolü
  bool isValid(double x, double y, const std::vector<int8_t> & grid);

  // Dünya koordinatı → grid index
  int toIndex(double x, double y);

  // Durum uzayı anahtarı (x, y, theta) → string key
  std::string toKey(double x, double y, double theta);

  // Bisiklet kinematik modeli ile sonraki durumu hesapla
  HNode motion(const HNode & current, double steer, double step);
  
  double goal_thresh_ = 1.0;
};

#endif
