// "Copyright [year] <Copyright Owner>"

#include "pixhawk_platform.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PixhawkPlatform>();
  rclcpp::Rate r(200);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    r.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
