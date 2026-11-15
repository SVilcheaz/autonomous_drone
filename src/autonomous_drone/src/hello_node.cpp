#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("hello_node");

  RCLCPP_INFO(node->get_logger(), "Hello from my first ROS 2 node!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
