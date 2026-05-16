#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "askar_interfaces/msg/vehicle_location.hpp"
#include "global_planner/astar.hpp"
#include <cmath>

class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode() : Node("global_planner_node")
  {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&GlobalPlannerNode::mapCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&GlobalPlannerNode::goalCallback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<askar_interfaces::msg::VehicleLocation>(
      "/vehicle_location", 10,
      std::bind(&GlobalPlannerNode::poseCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);

    RCLCPP_INFO(this->get_logger(), "Global Planner Node baslatildi.");
  }

private:
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_;
  askar_interfaces::msg::VehicleLocation::SharedPtr current_location_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<askar_interfaces::msg::VehicleLocation>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::pair<double,double> gpsToMeters(
    double lat, double lon,
    double ref_lat, double ref_lon)
  {
    const double R = 6371000.0;
    double x = R * (lon - ref_lon) * M_PI / 180.0 *
               std::cos(ref_lat * M_PI / 180.0);
    double y = R * (lat - ref_lat) * M_PI / 180.0;
    return {x, y};
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (map_) return;
    map_ = msg;
    RCLCPP_INFO(this->get_logger(), "Harita alindi: %d x %d",
      msg->info.width, msg->info.height);
    planPath();
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_ = msg;
    RCLCPP_INFO(this->get_logger(), "Hedef alindi: x=%.2f y=%.2f",
      msg->pose.position.x, msg->pose.position.y);
    planPath();
  }

  void poseCallback(const askar_interfaces::msg::VehicleLocation::SharedPtr msg)
  {
    // Sadece konumu sakla, her konum geldiğinde planlamayı tetikleme
    // Planlama sadece hedef veya harita değiştiğinde yapılır
    current_location_ = msg;
  }

  void planPath()
  {
    if (!map_ || !goal_ || !current_location_) {
      RCLCPP_WARN(this->get_logger(), "Harita, hedef veya konum bekleniyor...");
      return;
    }

    float res  = map_->info.resolution;
    int width  = map_->info.width;
    int height = map_->info.height;
    float ox   = map_->info.origin.position.x;
    float oy   = map_->info.origin.position.y;

    double ref_lat = 40.790343;
    double ref_lon = 29.509014;

    double sx, sy, gx, gy;

    double cx = current_location_->x;
    double cy = current_location_->y;
    bool is_gps = current_location_->is_gps_reliable && (cy > 35.0 && cy < 43.0 && cx > 25.0 && cx < 45.0);
    if (is_gps) {
      auto [mx, my] = gpsToMeters(cy, cx, ref_lat, ref_lon);
      sx = mx; sy = my;
    } else {
      sx = cx; sy = cy;
    }

    double tx = goal_->pose.position.x;
    double ty = goal_->pose.position.y;
    bool is_gps_goal = (ty > 35.0 && ty < 43.0 && tx > 25.0 && tx < 45.0);
    if (is_gps_goal) {
      auto [mx, my] = gpsToMeters(ty, tx, ref_lat, ref_lon);
      gx = mx; gy = my;
    } else {
      gx = tx; gy = ty;
    }

    auto toGrid = [&](double wx, double wy) -> std::pair<int,int> {
      return {
        static_cast<int>((wx - ox) / res),
        static_cast<int>((wy - oy) / res)
      };
    };

    auto start = toGrid(sx, sy);
    auto goal  = toGrid(gx, gy);

    RCLCPP_INFO(this->get_logger(),
      "Planlama basliyor: start=(%.2f,%.2f) goal=(%.2f,%.2f)",
      sx, sy, gx, gy);

    AStar astar(width, height, res);
    auto path_cells = astar.plan(start, goal, map_->data);

    if (path_cells.empty()) {
      RCLCPP_WARN(this->get_logger(), "Rota bulunamadi!");
      return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = this->now();
    path_msg.header.frame_id = "map";

    for (auto & cell : path_cells) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header           = path_msg.header;
      pose.pose.position.x  = cell.first  * res + ox;
      pose.pose.position.y  = cell.second * res + oy;
      pose.pose.position.z  = 0.0;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
    RCLCPP_INFO(this->get_logger(),
      "Rota yayinlandi: %zu waypoint", path_cells.size());
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
