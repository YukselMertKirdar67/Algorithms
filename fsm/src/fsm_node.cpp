#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "askar_interfaces/msg/vehicle_location.hpp"
#include "askar_interfaces/msg/vehicle_command.hpp"
#include "askar_interfaces/msg/traffic_light.hpp"
#include "askar_interfaces/msg/traffic_sign.hpp"
#include "fsm/fsm.hpp"
#include <cmath>

class FSMNode : public rclcpp::Node
{
public:
  FSMNode() : Node("fsm_node"), state_(FSMState::IDLE)
  {
    location_sub_ = this->create_subscription<askar_interfaces::msg::VehicleLocation>(
      "/vehicle_location", 10,
      std::bind(&FSMNode::locationCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&FSMNode::goalCallback, this, std::placeholders::_1));

    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/global_path", 10,
      std::bind(&FSMNode::globalPathCallback, this, std::placeholders::_1));

    replanned_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/replanned_path", 10,
      std::bind(&FSMNode::replannedPathCallback, this, std::placeholders::_1));

    traffic_light_sub_ = this->create_subscription<askar_interfaces::msg::TrafficLight>(
      "/traffic_light", 10,
      std::bind(&FSMNode::trafficLightCallback, this, std::placeholders::_1));

    traffic_sign_sub_ = this->create_subscription<askar_interfaces::msg::TrafficSign>(
      "/traffic_sign", 10,
      std::bind(&FSMNode::trafficSignCallback, this, std::placeholders::_1));

    pickup_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/passenger_pickup", 10,
      std::bind(&FSMNode::pickupCallback, this, std::placeholders::_1));

    dropoff_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/passenger_dropoff", 10,
      std::bind(&FSMNode::dropoffCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<askar_interfaces::msg::VehicleCommand>("/vehicle_command", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&FSMNode::update, this));

    RCLCPP_INFO(this->get_logger(), "FSM Node baslatildi. Durum: IDLE");
  }

private:
  FSMState state_;

  askar_interfaces::msg::VehicleLocation::SharedPtr current_location_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_goal_;
  nav_msgs::msg::Path::SharedPtr global_path_;
  nav_msgs::msg::Path::SharedPtr replanned_path_;
  askar_interfaces::msg::TrafficLight::SharedPtr traffic_light_;
  askar_interfaces::msg::TrafficSign::SharedPtr traffic_sign_;

  rclcpp::Subscription<askar_interfaces::msg::VehicleLocation>::SharedPtr location_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr replanned_path_sub_;
  rclcpp::Subscription<askar_interfaces::msg::TrafficLight>::SharedPtr traffic_light_sub_;
  rclcpp::Subscription<askar_interfaces::msg::TrafficSign>::SharedPtr traffic_sign_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pickup_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dropoff_sub_;
  rclcpp::Publisher<askar_interfaces::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time stop_start_time_;
  rclcpp::Time passenger_start_time_;
  rclcpp::Time wait_start_time_;
  rclcpp::Time last_traffic_sign_time_;
  
  bool traffic_sign_received_   = false;
  bool passenger_timer_started_ = false;
  bool is_pickup_point_         = false;
  bool is_dropoff_              = false;
  bool stop_timer_started_      = false;
  bool wait_timer_started_      = false;
  double speed_limit_ = 5.0;  // varsayılan hız Bee1 için

  void locationCallback(const askar_interfaces::msg::VehicleLocation::SharedPtr msg)
  { current_location_ = msg; }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_goal_ = msg;
    if (state_ == FSMState::IDLE) setState(FSMState::PLANNING);
  }

  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    global_path_ = msg;
    if (state_ == FSMState::PLANNING && !msg->poses.empty())
      setState(FSMState::DRIVING);
    if (msg->poses.empty() && state_ == FSMState::PLANNING)
      setState(FSMState::EMERGENCY);
  }

  void replannedPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (replanned_path_ && state_ == FSMState::DRIVING) {
      int old_size = replanned_path_->poses.size();
      int new_size = msg->poses.size();
      int diff = std::abs(new_size - old_size);
      if (diff > 10) setState(FSMState::REPLANNING);
    }
    replanned_path_ = msg;
  }

  void trafficLightCallback(const askar_interfaces::msg::TrafficLight::SharedPtr msg)
  { traffic_light_ = msg; }

  void trafficSignCallback(const askar_interfaces::msg::TrafficSign::SharedPtr msg)
  {
    traffic_sign_ = msg;
    traffic_sign_received_ = true;
    last_traffic_sign_time_ = this->now();
  }
  void pickupCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {  
    if (state_ == FSMState::DRIVING) {
      is_dropoff_ = false;
      current_goal_ = msg;
      setState(FSMState::APPROACHING);
    }
    (void)msg;
  }

  void dropoffCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (state_ == FSMState::DRIVING) {
      is_dropoff_ = true;
      current_goal_ = msg;
      setState(FSMState::APPROACHING);
    }
    (void)msg;
  }

  void setState(FSMState new_state)
  {
    if (state_ != new_state) {
      RCLCPP_INFO(this->get_logger(), "Durum degisti: %s -> %s",
        stateToString(state_).c_str(), stateToString(new_state).c_str());
      state_ = new_state;
    }
  }

  double distanceToGoal()
  {
    if (!current_location_ || !current_goal_) return 999.0;
    double dx = current_location_->x - current_goal_->pose.position.x;
    double dy = current_location_->y - current_goal_->pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  void update()
  {
    askar_interfaces::msg::VehicleCommand cmd;
    cmd.emergency_stop = false;

    switch (state_) {

      case FSMState::IDLE:
        cmd.target_speed    = 0.0;
        cmd.target_steering = 0.0;
        cmd.current_state   = "IDLE";
        break;

      case FSMState::PLANNING:
        cmd.target_speed    = 0.0;
        cmd.target_steering = 0.0;
        cmd.current_state   = "PLANNING";
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Rota bekleniyor...");
        break;

      case FSMState::DRIVING:
        if (distanceToGoal() < 3.0) { setState(FSMState::PARKING); break; }

        // Kırmızı ışık
        if (traffic_light_ && traffic_light_->color == "red"
            && traffic_light_->distance < 5.0) {
          setState(FSMState::STOP); break;
        }

        // Sarı ışık
        if (traffic_light_ && traffic_light_->color == "yellow") {
          setState(FSMState::SLOW_DOWN); break;
        }
        // Hız sınırı tabelası
        if (traffic_sign_ && traffic_sign_->sign_type == "speed_limit") {
         double new_limit = traffic_sign_->confidence * 10.0;
         if (std::abs(new_limit - speed_limit_) > 0.1) {
           speed_limit_ = new_limit;
           RCLCPP_INFO(this->get_logger(), "Hiz siniri guncellendi: %.1f m/s", speed_limit_);
         }
        }

        // Dur işareti
        if (traffic_sign_ && traffic_sign_->sign_type == "stop"
            && traffic_sign_->distance < 3.0) {
          setState(FSMState::STOP); break;
        }

        // Yaya geçidi
        if (traffic_sign_ && traffic_sign_->sign_type == "pedestrian"
            && traffic_sign_->distance < 5.0) {
          setState(FSMState::PEDESTRIAN_WAIT); break;
        }
        // Yol ver tabelası
        if (traffic_sign_ && traffic_sign_->sign_type == "yield"
             && traffic_sign_->distance < 5.0) {
           setState(FSMState::YIELD); break;
        }

        // Tünel tabelası
        if (traffic_sign_ && traffic_sign_->sign_type == "tunnel"
             && traffic_sign_->distance < 10.0) {
           setState(FSMState::TUNNEL); break;
        }

        // Yolcu noktasına yaklaşma
        if (traffic_sign_ && traffic_sign_->sign_type == "approaching_passenger"
             && traffic_sign_->distance < 10.0) {
           setState(FSMState::APPROACHING); break;
        }

        // GPS güvenilir değilse hızı düşür
        if (current_location_ && !current_location_->is_gps_reliable) {
         cmd.target_speed = std::min(speed_limit_, 2.0);
         cmd.current_state = "DRIVING_NO_GPS";
        } else {
             cmd.target_speed = speed_limit_;
             cmd.current_state = "DRIVING";
          }
        cmd.target_steering = 0.0;
        break;

      case FSMState::SLOW_DOWN:
        cmd.target_speed    = 2.0;
        cmd.target_steering = 0.0;
        cmd.current_state   = "SLOW_DOWN";

        // Sarı ışık geçti mi?
        if (!traffic_light_ || traffic_light_->color == "green") {
          setState(FSMState::DRIVING);
        }
        // Kırmızıya döndü mü?
        if (traffic_light_ && traffic_light_->color == "red"
            && traffic_light_->distance < 5.0) {
          setState(FSMState::STOP);
        }
        break;

      case FSMState::STOP:
        cmd.target_speed    = 0.0;
        cmd.target_steering = 0.0;
        cmd.current_state   = "STOP";

        // Yeşil ışık
        if (traffic_light_ && traffic_light_->color == "green") {
          stop_timer_started_ = false;
          traffic_sign_ = nullptr;
          setState(FSMState::DRIVING);
          break;
        }

        // Dur işareti — 3 saniye bekle
        if (traffic_sign_ && traffic_sign_->sign_type == "stop") {
          if (!stop_timer_started_) {
            stop_start_time_ = this->now();
            stop_timer_started_ = true;
          }
          double elapsed = (this->now() - stop_start_time_).seconds();
          if (elapsed > 3.0) {
            stop_timer_started_ = false;
            traffic_sign_ = nullptr;
            setState(FSMState::DRIVING);
          }
        }
        break;

      case FSMState::WAIT:
        cmd.target_speed    = 0.0;
        cmd.target_steering = 0.0;
        cmd.current_state   = "WAIT";

        // Engel geçti mi — normal rota geri geldi
        if (replanned_path_ && !replanned_path_->poses.empty()) {
          int size = replanned_path_->poses.size();
          if (size < 10000) {
            setState(FSMState::DRIVING);
          }
        }

        // Max 30 saniye bekle
        if (!wait_timer_started_) {
          wait_start_time_ = this->now();
          wait_timer_started_ = true;
        }
        if ((this->now() - wait_start_time_).seconds() > 30.0) {
          wait_timer_started_ = false;
          setState(FSMState::REPLANNING);
        }
        break;

      case FSMState::PEDESTRIAN_WAIT:
        cmd.target_speed    = 0.0;
        cmd.target_steering = 0.0;
        cmd.current_state   = "PEDESTRIAN_WAIT";

        if (!wait_timer_started_) {
          wait_start_time_ = this->now();
          wait_timer_started_ = true;
          RCLCPP_INFO(this->get_logger(), "Yaya gecidi bekleniyor — 3 saniye dur.");
        }

        {
         double elapsed = (this->now() - wait_start_time_).seconds();
         RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
           "Yaya gecidi: %.1f saniye", elapsed);
         if (elapsed > 3.0) {
           wait_timer_started_ = false;
           traffic_sign_ = nullptr;
           setState(FSMState::DRIVING);
         }
        }
       break;
       
      case FSMState::YIELD:
        cmd.target_speed    = 1.0;
        cmd.target_steering = 0.0;
        cmd.current_state   = "YIELD";

        // 2 saniyedir mesaj gelmiyorsa DRIVING'e dön
        if (traffic_sign_received_) {
          double elapsed = (this->now() - last_traffic_sign_time_).seconds();
         if (elapsed > 2.0) {
           traffic_sign_ = nullptr;
           traffic_sign_received_ = false;
           setState(FSMState::DRIVING);
         }
        }
        // Mesafe uzaklaştıysa da dön
        if (traffic_sign_ && traffic_sign_->distance > 10.0) {
         traffic_sign_ = nullptr;
         setState(FSMState::DRIVING);
        }
       break;

      case FSMState::APPROACHING:
        cmd.target_speed    = 1.0;
        cmd.target_steering = 0.0;
        cmd.current_state   = "APPROACHING";

      // Yolcu noktasına ulaştı mı?
       if (distanceToGoal() < 1.0) {
        if (is_dropoff_) {
            setState(FSMState::PASSENGER_DROPOFF);
        } else {
           setState(FSMState::PASSENGER_PICKUP);
          }
       }
      break;

      case FSMState::TUNNEL:
        cmd.target_speed    = std::min(speed_limit_, 2.0);
        cmd.target_steering = 0.0;
        cmd.current_state   = "TUNNEL";

        // 2 saniyedir mesaj gelmiyorsa DRIVING'e dön
        if (traffic_sign_received_) {
          double elapsed = (this->now() - last_traffic_sign_time_).seconds();
          if (elapsed > 2.0) {
            traffic_sign_ = nullptr;
            traffic_sign_received_ = false;
            setState(FSMState::DRIVING);
          }
        }
        // Mesafe uzaklaştıysa da dön
        if (traffic_sign_ && traffic_sign_->distance > 20.0) {
          traffic_sign_ = nullptr;
          setState(FSMState::DRIVING);
        }
      break;

      case FSMState::LANE_CHANGE:
        cmd.target_speed    = 2.0;
        cmd.target_steering = 0.0;
        cmd.current_state   = "LANE_CHANGE";

        // Replanning tamamlandı mı?
        if (replanned_path_ && !replanned_path_->poses.empty()) {
          setState(FSMState::DRIVING);
        }
        break;

      case FSMState::REPLANNING:
        cmd.target_speed    = 1.0;
        cmd.target_steering = 0.0;
        cmd.current_state   = "REPLANNING";

        if (replanned_path_ && !replanned_path_->poses.empty()) {
          setState(FSMState::DRIVING);
        }
        break;

      case FSMState::PASSENGER_PICKUP:
        cmd.target_speed    = 0.0;
        cmd.target_steering = 0.0;
        cmd.current_state   = "PASSENGER_PICKUP";

        if (!passenger_timer_started_) {
          passenger_start_time_ = this->now();
          passenger_timer_started_ = true;
          RCLCPP_INFO(this->get_logger(), "Yolcu bindirme baslatildi — 15-20 saniye bekleniyor.");
        }
        {
          double elapsed = (this->now() - passenger_start_time_).seconds();
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "Yolcu bindirme: %.1f saniye", elapsed);
          if (elapsed >= 15.0 && elapsed <= 20.0) {
            passenger_timer_started_ = false;
            RCLCPP_INFO(this->get_logger(), "Yolcu bindirme tamamlandi!");
            setState(FSMState::DRIVING);
          } else if (elapsed > 20.0) {
            passenger_timer_started_ = false;
            RCLCPP_WARN(this->get_logger(), "Yolcu bindirme 20 saniye asild!");
            setState(FSMState::DRIVING);
          }
        }
        break;

      case FSMState::PASSENGER_DROPOFF:
        cmd.target_speed    = 0.0;
        cmd.target_steering = 0.0;
        cmd.current_state   = "PASSENGER_DROPOFF";

        if (!passenger_timer_started_) {
          passenger_start_time_ = this->now();
          passenger_timer_started_ = true;
          RCLCPP_INFO(this->get_logger(), "Yolcu indirme baslatildi — 15-20 saniye bekleniyor.");
        }
        {
          double elapsed = (this->now() - passenger_start_time_).seconds();
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "Yolcu indirme: %.1f saniye", elapsed);
          if (elapsed >= 15.0 && elapsed <= 20.0) {
            passenger_timer_started_ = false;
            RCLCPP_INFO(this->get_logger(), "Yolcu indirme tamamlandi!");
            setState(FSMState::DRIVING);
          } else if (elapsed > 20.0) {
            passenger_timer_started_ = false;
            RCLCPP_WARN(this->get_logger(), "Yolcu indirme 20 saniye asild!");
            setState(FSMState::DRIVING);
          }
        }
        break;

      case FSMState::PARKING:
        cmd.target_speed    = 0.5;
        cmd.target_steering = 0.0;
        cmd.current_state   = "PARKING";

        if (distanceToGoal() < 0.5) {
          cmd.target_speed = 0.0;
          setState(FSMState::ARRIVED);
        }
        break;

      case FSMState::ARRIVED:
        cmd.target_speed    = 0.0;
        cmd.target_steering = 0.0;
        cmd.current_state   = "ARRIVED";
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Hedefe ulasildi!");
        current_goal_ = nullptr;
        global_path_  = nullptr;
        setState(FSMState::IDLE);
        break;

      case FSMState::EMERGENCY:
        cmd.target_speed    = 0.0;
        cmd.target_steering = 0.0;
        cmd.current_state   = "EMERGENCY";
        cmd.emergency_stop  = true;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "EMERGENCY: Araç durduruldu!");
        break;

      default:
        break;
    }

    cmd_pub_->publish(cmd);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FSMNode>());
  rclcpp::shutdown();
  return 0;
}
