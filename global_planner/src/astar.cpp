#include "global_planner/astar.hpp"

AStar::AStar(int width, int height, float resolution)
: width_(width), height_(height), resolution_(resolution) {}

int AStar::toIndex(int x, int y)
{
  return y * width_ + x;
}

float AStar::heuristic(int x1, int y1, int x2, int y2)
{
  // Öklid mesafesi
  return std::sqrt(
    std::pow(static_cast<float>(x2 - x1), 2) +
    std::pow(static_cast<float>(y2 - y1), 2)
  );
}

bool AStar::isValid(int x, int y, const std::vector<int8_t> & grid)
{
  // Araç boyutuna göre inflation radius
  // Araç genişliği: 1.06m, çözünürlük: 0.5m → 2 hücre güvenlik mesafesi
  int inflation = static_cast<int>(0.3 / resolution_);
  if (inflation < 1) inflation = 1;

  for (int dx = -inflation; dx <= inflation; dx++) {
    for (int dy = -inflation; dy <= inflation; dy++) {
      int nx = x + dx;
      int ny = y + dy;

      if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
        return false;
      }

      int8_t cell = grid[toIndex(nx, ny)];
      if (cell < 0 || cell >= 50) {
        return false;
      }
    }
  }

  return true;
}

std::vector<std::pair<int,int>> AStar::plan(
  std::pair<int,int> start,
  std::pair<int,int> goal,
  const std::vector<int8_t> & grid)
{
  // Open list: en düşük f değeri önce çıkar
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;

  // Tüm ziyaret edilen düğümler
  std::unordered_map<int, Node> all_nodes;

  // Kapalı liste
  std::unordered_map<int, bool> closed_list;

  // Başlangıç düğümü oluştur
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

  // 8 yönlü hareket
  // Düz: maliyet 1.0, çapraz: maliyet 1.414
  int dx[] = { 0,  0,  1, -1,  1,  1, -1, -1};
  int dy[] = { 1, -1,  0,  0,  1, -1,  1, -1};
  float move_cost[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.414f, 1.414f, 1.414f, 1.414f};

  while (!open_list.empty()) {

    // En düşük f değerli düğümü al
    Node current = open_list.top();
    open_list.pop();

    int curr_idx = toIndex(current.x, current.y);

    // Zaten işlendiyse atla
    if (closed_list.count(curr_idx) && closed_list[curr_idx]) {
      continue;
    }
    closed_list[curr_idx] = true;

    // Hedefe ulaşıldı mı?
    if (current.x == goal.first && current.y == goal.second) {

      // Geriye doğru iz sürerek rotayı oluştur
      std::vector<std::pair<int,int>> path;
      Node n = current;

      while (n.parent_x != -1 && n.parent_y != -1) {
        path.push_back({n.x, n.y});
        int parent_idx = toIndex(n.parent_x, n.parent_y);
        n = all_nodes[parent_idx];
      }

      // Başlangıç noktasını da ekle
      path.push_back({start.first, start.second});

      // Rotayı baştan sona sırala
      std::reverse(path.begin(), path.end());
      return path;
    }

    // 8 komşuyu genişlet
    for (int i = 0; i < 8; i++) {
      int nx = current.x + dx[i];
      int ny = current.y + dy[i];

      // Geçerli değilse atla
      if (!isValid(nx, ny, grid)) {
        continue;
      }

      int n_idx = toIndex(nx, ny);

      // Zaten kapalı listedeyse atla
      if (closed_list.count(n_idx) && closed_list[n_idx]) {
        continue;
      }

      float new_g = current.g + move_cost[i];

      // Bu düğüm hiç görülmedi veya daha iyi bir yol bulundu
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

  // Rota bulunamadı — boş liste döndür
  return {};
}
