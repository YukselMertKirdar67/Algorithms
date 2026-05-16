#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "askar_interfaces/msg/vehicle_location.hpp"
#include "replanner/dstar_lite.hpp"

class ReplannerNode : public rclcpp::Node
{
public:
  ReplannerNode() : Node("replanner_node"), initialized_(false)
  {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&ReplannerNode::mapCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&ReplannerNode::goalCallback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<askar_interfaces::msg::VehicleLocation>(
      "/vehicle_location", 10,
      std::bind(&ReplannerNode::poseCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/replanned_path", 10);

    RCLCPP_INFO(this->get_logger(), "Replanner Node baslatildi.");
  }

private:
  bool initialized_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_;
  askar_interfaces::msg::VehicleLocation::SharedPtr current_location_;
  std::shared_ptr<DStarLite> dstar_;

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
    bool first_map = (map_ == nullptr);
    map_ = msg;

    if (first_map) {
      RCLCPP_INFO(this->get_logger(), "Harita alindi: %d x %d",
        msg->info.width, msg->info.height);
      tryInitialize();
    } else {
      RCLCPP_INFO(this->get_logger(), "Harita guncellendi — D* Lite replanning!");
      if (initialized_) {
        dstar_->updateGrid(msg->data);
        publishPath();
      }
    }
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_ = msg;
    RCLCPP_INFO(this->get_logger(), "Hedef alindi: x=%.2f y=%.2f",
      msg->pose.position.x, msg->pose.position.y);
    tryInitialize();
  }

  void poseCallback(const askar_interfaces::msg::VehicleLocation::SharedPtr msg)
  {
    current_location_ = msg;
    if (!initialized_) {
      tryInitialize();
    }
  }

  void tryInitialize()
  {
    if (!map_ || !goal_ || !current_location_) return;
    if (initialized_) return;

    float res    = map_->info.resolution;
    int width    = map_->info.width;
    int height   = map_->info.height;
    float ox     = map_->info.origin.position.x;
    float oy     = map_->info.origin.position.y;

    double ref_lat = 40.790343;
    double ref_lon = 29.509014;

    double cx = current_location_->x;
    double cy = current_location_->y;
    double sx, sy;
    bool is_gps = (cy > 35.0 && cy < 43.0 && cx > 25.0 && cx < 45.0);
    if (is_gps) {
      auto [mx, my] = gpsToMeters(cy, cx, ref_lat, ref_lon);
      sx = mx; sy = my;
    } else {
      sx = cx; sy = cy;
    }

    double tx = goal_->pose.position.x;
    double ty = goal_->pose.position.y;
    double gx, gy;
    bool is_gps_goal = (ty > 35.0 && ty < 43.0 && tx > 25.0 && tx < 45.0);
    if (is_gps_goal) {
      auto [mx, my] = gpsToMeters(ty, tx, ref_lat, ref_lon);
      gx = mx; gy = my;
    } else {
      gx = tx; gy = ty;
    }

    dstar_ = std::make_shared<DStarLite>(width, height, res, ox, oy);

    auto start_grid = dstar_->worldToGrid(sx, sy);
    auto goal_grid  = dstar_->worldToGrid(gx, gy);

    RCLCPP_INFO(this->get_logger(),
      "D* Lite baslatiliyor: start=(%d,%d) goal=(%d,%d)",
      start_grid.first, start_grid.second,
      goal_grid.first, goal_grid.second);

    dstar_->initialize(start_grid, goal_grid, map_->data);
    initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "D* Lite hazir!");
    publishPath();
  }

  void publishPath()
  {
    if (!initialized_ || !current_location_) return;

    double ref_lat = 40.790343;
    double ref_lon = 29.509014;

    double cx = current_location_->x;
    double cy = current_location_->y;
    double sx, sy;
    bool is_gps = (cy > 35.0 && cy < 43.0 && cx > 25.0 && cx < 45.0);
    if (is_gps) {
      auto [mx, my] = gpsToMeters(cy, cx, ref_lat, ref_lon);
      sx = mx; sy = my;
    } else {
      sx = cx; sy = cy;
    }

    auto current_grid = dstar_->worldToGrid(sx, sy);
    auto path_cells   = dstar_->getPath(current_grid);

    if (path_cells.empty()) {
      RCLCPP_WARN(this->get_logger(), "D* Lite rota bulunamadi!");
      return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = this->now();
    path_msg.header.frame_id = "map";

    for (auto & cell : path_cells) {
      auto [wx, wy] = dstar_->gridToWorld(cell.first, cell.second);
      geometry_msgs::msg::PoseStamped pose;
      pose.header           = path_msg.header;
      pose.pose.position.x  = wx;
      pose.pose.position.y  = wy;
      pose.pose.position.z  = 0.0;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
    RCLCPP_INFO(this->get_logger(),
      "D* Lite rota yayinlandi: %zu waypoint", path_cells.size());
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReplannerNode>());
  rclcpp::shutdown();
  return 0;
}
