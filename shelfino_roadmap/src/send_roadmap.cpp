#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "geometry_msgs/msg/polygon.hpp"

using std::placeholders::_1;

class RoadmapPublisher : public rclcpp::Node
{
  public:
    RoadmapPublisher()
    : Node("send_roadmap")
    {
      subscription_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "/obstacles", 10, std::bind(&RoadmapPublisher::obstacles_callback, this, _1));
      subscription_borders_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "/map_borders", 10, std::bind(&RoadmapPublisher::borders_callback, this, _1));
    }

  private:
    std::vector<std::pair<float, float>> obstacle_vertices;

    void obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        for (const auto &obs : msg->obstacles) {
            for (const auto point : obs.polygon.points) {
                RCLCPP_INFO(this->get_logger(), "I heard an obstacle vertex: '%f', '%f", point.x, point.y);
                obstacle_vertices.push_back(std::make_pair(point.x, point.y));
            }
            
        }
      
    }

    void borders_callback(const geometry_msgs::msg::Polygon msg)
    {

        for (const auto point : msg.points) {
            RCLCPP_INFO(this->get_logger(), "I heard a border vertex: '%f', '%f", point.x, point.y);
            obstacle_vertices.push_back(std::make_pair(point.x, point.y));
        }
            
      
    }
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_obstacles_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_borders_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoadmapPublisher>());
  rclcpp::shutdown();
  return 0;
}