#include "trtinfer.hpp"

#include <memory>


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClusterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}