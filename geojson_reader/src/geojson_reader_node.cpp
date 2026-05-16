#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "askar_interfaces/msg/vehicle_location.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

struct GeoPoint {
  std::string name;
  std::string type;
  double lat;
  double lon;
};

class GeoJsonReaderNode : public rclcpp::Node
{
public:
  GeoJsonReaderNode() : Node("geojson_reader_node"), current_index_(0)
  {
    this->declare_parameter("geojson_file", "");
    this->declare_parameter("ref_lat", 40.790343);
    this->declare_parameter("ref_lon", 29.509014);

    std::string file = this->get_parameter("geojson_file").as_string();
    ref_lat_ = this->get_parameter("ref_lat").as_double();
    ref_lon_ = this->get_parameter("ref_lon").as_double();

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    passenger_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/passenger_point", 10);
    pickup_pub_  = this->create_publisher<geometry_msgs::msg::PoseStamped>("/passenger_pickup", 10);
    dropoff_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/passenger_dropoff", 10);


    location_sub_ = this->create_subscription<askar_interfaces::msg::VehicleLocation>(
      "/vehicle_location", 10,
      std::bind(&GeoJsonReaderNode::locationCallback, this, std::placeholders::_1));

    if (!file.empty()) {
      loadGeoJson(file);
    } else {
      RCLCPP_WARN(this->get_logger(), "GEOJSON dosyasi belirtilmedi! Ornek koordinatlar yukleniyor.");
      loadSamplePoints();
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&GeoJsonReaderNode::update, this));

    RCLCPP_INFO(this->get_logger(), "GeoJSON Reader baslatildi. %zu nokta yuklendi.", points_.size());
    for (auto & p : points_) {
      RCLCPP_INFO(this->get_logger(), "  Nokta: %s lat=%.6f lon=%.6f",
        p.name.c_str(), p.lat, p.lon);
    }
  }

private:
  std::vector<GeoPoint> points_;
  std::vector<GeoPoint> passenger_points_;
  size_t current_index_;
  double ref_lat_;
  double ref_lon_;
  bool goal_published_ = false;
  askar_interfaces::msg::VehicleLocation::SharedPtr current_location_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr passenger_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pickup_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dropoff_pub_;
  rclcpp::Subscription<askar_interfaces::msg::VehicleLocation>::SharedPtr location_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void loadSamplePoints()
  {
    GeoPoint gorev1;
    gorev1.name = "gorev_1";
    gorev1.type = "pickup";
    gorev1.lat  = 40.790298;
    gorev1.lon  = 29.509014;

    GeoPoint gorev2;
    gorev2.name = "gorev_2";
    gorev2.type = "dropoff";
    gorev2.lat  = 40.790388;
    gorev2.lon  = 29.509014;

    GeoPoint park;
    park.name = "park_giris";
    park.type = "park";
    park.lat  = 40.790505;
    park.lon  = 29.509014;

    points_.push_back(gorev1);
    points_.push_back(gorev2);
    points_.push_back(park);

    passenger_points_.push_back(gorev1);
    passenger_points_.push_back(gorev2);
  }

  void loadGeoJson(const std::string & file_path)
  {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "GEOJSON dosyasi acilamadi: %s", file_path.c_str());
      loadSamplePoints();
      return;
    }

    std::string content((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());

    // Basit JSON parser — coordinates ve name alanlarını bul
    size_t pos = 0;
    while ((pos = content.find("\"name\"", pos)) != std::string::npos) {
      // name değerini bul
      size_t name_start = content.find("\"", pos + 7) + 1;
      size_t name_end   = content.find("\"", name_start);
      std::string name  = content.substr(name_start, name_end - name_start);

      // coordinates değerini bul
      size_t coord_pos = content.find("\"coordinates\"", pos);
      if (coord_pos == std::string::npos) { pos++; continue; }

      size_t bracket = content.find("[", coord_pos) + 1;
      size_t comma   = content.find(",", bracket);
      size_t end     = content.find("]", comma);

      double lon = std::stod(content.substr(bracket, comma - bracket));
      double lat = std::stod(content.substr(comma + 1, end - comma - 1));

      GeoPoint p;
      p.name = name;
      p.lat  = lat;
      p.lon  = lon;

      if (name == "start") {
        ref_lat_ = lat;
        ref_lon_ = lon;
        RCLCPP_INFO(this->get_logger(), "Ref nokta guncellendi: lat=%.6f lon=%.6f", lat, lon);
      } else {
        points_.push_back(p);
        if (name.find("gorev") != std::string::npos) {
          passenger_points_.push_back(p);
        }
      }
      pos = end;
    }

    if (points_.empty()) {
      RCLCPP_WARN(this->get_logger(), "GEOJSON parse edilemedi, ornek noktalar yukleniyor.");
      loadSamplePoints();
    }
  }

  std::pair<double, double> gpsToMeters(double lat, double lon)
  {
    const double R = 6371000.0;
    double x = R * (lon - ref_lon_) * M_PI / 180.0 * std::cos(ref_lat_ * M_PI / 180.0);
    double y = R * (lat - ref_lat_) * M_PI / 180.0;
    return {x, y};
  }

  double distanceTo(double target_lat, double target_lon)
  {
    if (!current_location_) return 999.0;
    auto [tx, ty] = gpsToMeters(target_lat, target_lon);
    double dx = current_location_->x - tx;
    double dy = current_location_->y - ty;
    return std::sqrt(dx * dx + dy * dy);
  }

  void locationCallback(const askar_interfaces::msg::VehicleLocation::SharedPtr msg)
  {
    current_location_ = msg;
  }

  void update()
  {
    if (points_.empty() || current_index_ >= points_.size()) return;

    auto & current_point = points_[current_index_];
    auto [tx, ty] = gpsToMeters(current_point.lat, current_point.lon);

    // Hedefi yayınla — sadece bir kez
    if (!goal_published_) {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.stamp    = this->now();
      goal.header.frame_id = "map";
      goal.pose.position.x = tx;
      goal.pose.position.y = ty;
      goal_pub_->publish(goal);
      goal_published_ = true;
    }

    // Hedefe yaklaştı mı?
    if (current_location_) {
      double dist = distanceTo(current_point.lat, current_point.lon);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "Hedef: %s (%.1f m uzakta)", current_point.name.c_str(), dist);

      if (dist < 1.0) {
        RCLCPP_INFO(this->get_logger(), "%s noktasina ulasildi! (tip: %s)",
        current_point.name.c_str(), current_point.type.c_str());

        geometry_msgs::msg::PoseStamped action_pose;
        action_pose.header.stamp    = this->now();
        action_pose.header.frame_id = "map";
        action_pose.pose.position.x = tx;
        action_pose.pose.position.y = ty;

        if (current_point.type == "pickup") {
           pickup_pub_->publish(action_pose);
        } else if (current_point.type == "dropoff") {
               dropoff_pub_->publish(action_pose);
        }

        current_index_++;
        if (current_index_ < points_.size()) {
           RCLCPP_INFO(this->get_logger(), "Sonraki hedef: %s",
           points_[current_index_].name.c_str());
        } else {
           RCLCPP_INFO(this->get_logger(), "Tum noktalara ulasildi!");
        }
      }
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeoJsonReaderNode>());
  rclcpp::shutdown();
  return 0;
}
