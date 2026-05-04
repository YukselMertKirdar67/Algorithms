#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "askar_interfaces/msg/vehicle_location.hpp"
#include "askar_interfaces/msg/vehicle_command.hpp"
#include "local_planner/hybrid_astar.hpp"
#include <cmath>

class LocalPlannerNode : public rclcpp::Node
{
public:
  LocalPlannerNode() : Node("local_planner_node")
  {
    // Config parametrelerini oku
    this->declare_parameter("max_steering_angle", 32.5);
    this->declare_parameter("safe_stopping_distance", 2.0);
    this->declare_parameter("lookahead_distance", 3.5);
    this->declare_parameter("desired_speed", 5.0);
    this->declare_parameter("wheelbase", 1.86);
    this->declare_parameter("step_size", 1.0);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&LocalPlannerNode::mapCallback, this, std::placeholders::_1));

    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/global_path", 10,
      std::bind(&LocalPlannerNode::globalPathCallback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<askar_interfaces::msg::VehicleLocation>(
      "/vehicle_location", 10,
      std::bind(&LocalPlannerNode::poseCallback, this, std::placeholders::_1));

    local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
    cmd_pub_ = this->create_publisher<askar_interfaces::msg::VehicleCommand>("/vehicle_command", 10);

    RCLCPP_INFO(this->get_logger(), "Local Planner Node baslatildi.");
    RCLCPP_INFO(this->get_logger(), "max_steering_angle=%.1f lookahead=%.1f speed=%.1f wheelbase=%.2f step_size=%.2f",
      this->get_parameter("max_steering_angle").as_double(),
      this->get_parameter("lookahead_distance").as_double(),
      this->get_parameter("desired_speed").as_double(),
      this->get_parameter("wheelbase").as_double(),
      this->get_parameter("step_size").as_double());
  }

private:
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  nav_msgs::msg::Path::SharedPtr global_path_;
  askar_interfaces::msg::VehicleLocation::SharedPtr current_location_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<askar_interfaces::msg::VehicleLocation>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<askar_interfaces::msg::VehicleCommand>::SharedPtr cmd_pub_;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (map_) return;
    if (map_) return;
    map_ = msg;
    RCLCPP_INFO(this->get_logger(), "Harita alindi: %d x %d",
      msg->info.width, msg->info.height);
    planPath();
  }

  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    global_path_ = msg;
    RCLCPP_INFO(this->get_logger(), "Global rota alindi: %zu waypoint",
      msg->poses.size());
    planPath();
  }

  void poseCallback(const askar_interfaces::msg::VehicleLocation::SharedPtr msg)
  {
    current_location_ = msg;
  }

  void planPath()
  {
    if (!map_ || !global_path_ || !current_location_) {
      RCLCPP_WARN(this->get_logger(), "Harita, global rota veya konum bekleniyor...");
      return;
    }

    if (global_path_->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Global rota bos!");
      return;
    }

    // Config'den parametreleri al
    double max_steer_deg = this->get_parameter("max_steering_angle").as_double();
    double lookahead     = this->get_parameter("lookahead_distance").as_double();
    double desired_speed = this->get_parameter("desired_speed").as_double();
    double wheelbase     = this->get_parameter("wheelbase").as_double();

    VehicleParams params;
    params.wheelbase   = wheelbase;
    params.max_steer   = max_steer_deg * M_PI / 180.0;  // derece → radyan
    params.step_size = this->get_parameter("step_size").as_double();
    params.steer_steps = 5;

    float res    = map_->info.resolution;
    int width    = map_->info.width;
    int height   = map_->info.height;
    float ox     = map_->info.origin.position.x;
    float oy     = map_->info.origin.position.y;

    auto & last = global_path_->poses.back();
    double gx = last.pose.position.x;
    double gy = last.pose.position.y;
    double gt = 0.0;

    double ref_lat = 40.790343;
    double ref_lon = 29.509014;
    double cx = current_location_->x;
    double cy = current_location_->y;

    double sx, sy;
    bool is_gps = current_location_->is_gps_reliable &&
                  (cy > 35.0 && cy < 43.0 && cx > 25.0 && cx < 45.0);
    if (is_gps) {
      const double R = 6371000.0;
      sx = R * (cx - ref_lon) * M_PI / 180.0 * std::cos(ref_lat * M_PI / 180.0);
      sy = R * (cy - ref_lat) * M_PI / 180.0;
    } else {
      sx = cx;
      sy = cy;
    }

    double st = current_location_->yaw;

    RCLCPP_INFO(this->get_logger(),
      "Hybrid A* basliyor: start=(%.2f,%.2f) goal=(%.2f,%.2f)",
      sx, sy, gx, gy);

    HybridAStar hybrid(width, height, res, ox, oy, params);
    hybrid.setGoalThreshold(lookahead);  // lookahead_distance → goal_thresh

    auto path_states = hybrid.plan(
      std::make_tuple(sx, sy, st),
      std::make_tuple(gx, gy, gt),
      map_->data);

    if (path_states.empty()) {
      RCLCPP_WARN(this->get_logger(), "Lokal yörünge bulunamadi!");
      askar_interfaces::msg::VehicleCommand cmd;
      cmd.target_speed    = 0.0;
      cmd.target_steering = 0.0;
      cmd.current_state   = "EMERGENCY";
      cmd.emergency_stop  = true;
      cmd_pub_->publish(cmd);
      return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = this->now();
    path_msg.header.frame_id = "map";

    for (auto & state : path_states) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header           = path_msg.header;
      pose.pose.position.x  = std::get<0>(state);
      pose.pose.position.y  = std::get<1>(state);
      pose.pose.position.z  = 0.0;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }

    local_path_pub_->publish(path_msg);

    if (path_states.size() > 1) {
      double next_x = std::get<0>(path_states[1]);
      double next_y = std::get<1>(path_states[1]);
      double angle_to_next = std::atan2(next_y - sy, next_x - sx);
      double steering = angle_to_next - st;
      while (steering >  M_PI) steering -= 2.0 * M_PI;
      while (steering < -M_PI) steering += 2.0 * M_PI;

      askar_interfaces::msg::VehicleCommand cmd;
      cmd.target_speed    = desired_speed;
      cmd.target_steering = steering;
      cmd.current_state   = "DRIVING";
      cmd.emergency_stop  = false;
      cmd_pub_->publish(cmd);
    }

    RCLCPP_INFO(this->get_logger(),
      "Lokal yörünge yayinlandi: %zu nokta", path_states.size());
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
