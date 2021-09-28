// "Copyright [year] <Copyright Owner>"

#include "pixhawk_platform.hpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  std::cout << "Starting PixhawkPlatform " << std::endl;
  rclcpp::spin(std::make_shared<PixhawkPlatform>());
  rclcpp::shutdown();
  return 0;
}
