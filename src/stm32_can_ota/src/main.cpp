#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "stm32_can_ota/stm32_ota_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stm32_can_ota::Stm32OtaNode>());
  rclcpp::shutdown();
  return 0;
}
