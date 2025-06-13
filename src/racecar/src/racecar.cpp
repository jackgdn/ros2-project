#include "rclcpp/rclcpp.hpp"
#include "/root/dev_ws/src/racecar/include/racecar/Racecar_context.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  std::shared_ptr <Racecar> racecar = std::make_shared<Racecar>();
  racecar->run();
  rclcpp::shutdown();
  return 0;
}
