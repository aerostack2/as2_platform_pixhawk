// "Copyright [year] <Copyright Owner>"

#include "pixhawk_platform.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PixhawkPlatform>();
  node->spinLoop(300);
  rclcpp::shutdown();
  return 0;
}
