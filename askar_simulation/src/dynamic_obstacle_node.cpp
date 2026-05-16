#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class DynamicObstacleNode : public rclcpp::Node
{
public:
  DynamicObstacleNode() : Node("dynamic_obstacle_node"), t_(0.0)
  {
    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&DynamicObstacleNode::update, this));
    RCLCPP_INFO(this->get_logger(), "Dinamik Engel Node baslatildi.");
  }

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double t_;

  void setRect(nav_msgs::msg::OccupancyGrid & msg, float x1, float y1, float x2, float y2, int8_t val)
  {
    float res = msg.info.resolution;
    float ox  = msg.info.origin.position.x;
    float oy  = msg.info.origin.position.y;
    int width  = msg.info.width;
    int height = msg.info.height;

    int gx1 = (int)((x1 - ox) / res);
    int gy1 = (int)((y1 - oy) / res);
    int gx2 = (int)((x2 - ox) / res);
    int gy2 = (int)((y2 - oy) / res);

    for (int y = gy1; y < gy2; y++)
      for (int x = gx1; x < gx2; x++)
        if (x >= 0 && x < width && y >= 0 && y < height)
          msg.data[y * width + x] = val;
  }

  void update()
  {
    t_ += 0.1;

    double dyn_x = 5.0 + 5.0 * std::sin(t_);
    
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

    float res = 0.2f;
    int width  = 200;
    int height = 200;

    msg.info.resolution = res;
    msg.info.width  = width;
    msg.info.height = height;
    msg.info.origin.position.x = -20.0;
    msg.info.origin.position.y = -20.0;
    msg.info.origin.orientation.w = 1.0;
    msg.data.resize(width * height, 0);

    // Dış duvarlar
    setRect(msg, -20, -20, 20, -19.7, 100);
    setRect(msg, -20,  19.7, 20, 20, 100);
    setRect(msg, -20, -20, -19.7, 20, 100);
    setRect(msg,  19.7, -20, 20, 20, 100);

    // İç duvarlar
    setRect(msg, -10, 4.85, 0, 5.15, 100);
    setRect(msg, 4.85, -10, 5.15, 0, 100);

    // Statik engeller — çapraz
    setRect(msg, -8, -14, 4, -10, 100);
    setRect(msg, -4, -4, 8, 0, 100);
    setRect(msg, -8, 8, 4, 12, 100);

    // Kenar engeller
    setRect(msg, -16, 9, -14, 11, 100);
    setRect(msg, 14, -11, 16, -9, 100);
    setRect(msg, -11, -16, -9, -14, 100);

    
    float dx = (float)dyn_x;
    setRect(msg, dx - 0.8, 2.0, dx + 0.8, 3.5, 100);

    pub_->publish(msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicObstacleNode>());
  rclcpp::shutdown();
  return 0;
}
