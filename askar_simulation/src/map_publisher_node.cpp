#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class MapPublisherNode : public rclcpp::Node
{
public:
  MapPublisherNode() : Node("map_publisher_node")
  {
    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    buildMap();
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&MapPublisherNode::publishMap, this));
    RCLCPP_INFO(this->get_logger(), "Map Publisher baslatildi.");
  }

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid map_msg_;

  void buildMap()
  {
    float res = 0.2f;
    int width  = 200;  // 40 metre
    int height = 200;  // 40 metre

    map_msg_.header.frame_id = "map";
    map_msg_.info.resolution = res;
    map_msg_.info.width  = width;
    map_msg_.info.height = height;
    map_msg_.info.origin.position.x = -20.0;
    map_msg_.info.origin.position.y = -20.0;
    map_msg_.info.origin.orientation.w = 1.0;
    map_msg_.data.resize(width * height, 0);

    auto setRect = [&](float x1, float y1, float x2, float y2) {
      int gx1 = (int)((x1 + 20.0) / res);
      int gy1 = (int)((y1 + 20.0) / res);
      int gx2 = (int)((x2 + 20.0) / res);
      int gy2 = (int)((y2 + 20.0) / res);
      for (int y = gy1; y < gy2; y++)
        for (int x = gx1; x < gx2; x++)
          if (x >= 0 && x < width && y >= 0 && y < height)
            map_msg_.data[y * width + x] = 100;
    };

    // Dış duvarlar
    setRect(-20, -20, 20, -19.7);
    setRect(-20,  19.7, 20, 20);
    setRect(-20, -20, -19.7, 20);
    setRect( 19.7, -20, 20, 20);

    // İç duvarlar
    setRect(-10, 4.85, 0, 5.15);
    setRect(4.85, -10, 5.15, 0);

      // Çapraz dizilim — araç zig-zag yapmak zorunda
    // Statik engeller — çapraz dizilim
   setRect(-8, -14, 4, -10);    // engel 1 — sol alt
   setRect(-4, -4, 8, 0);       // engel 2 — orta
   setRect(4, 8, 8, 12);        // engel 3 — sağ üst
    // Kenar engeller
    setRect(-16, 9, -14, 11);    // engel 4
    setRect(14, -11, 16, -9);    // engel 5
    setRect(-11, -16, -9, -14);  // engel 6
  }

  void publishMap()
  {
    map_msg_.header.stamp = this->now();
    pub_->publish(map_msg_);
    RCLCPP_INFO(this->get_logger(), "Harita yayinlandi.");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
