// "Copyright [year] <Copyright Owner>"

#include "pixhawk_platform.hpp"
#include "as2_core/core_functions.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "ROS INIT" << std::endl;
  auto node = std::make_shared<PixhawkPlatform>();
  node->preset_loop_frequency(300);
  as2::spinLoop(node);  
  

  rclcpp::shutdown();
  return 0;
}
