#include <memory>
#include <random>
#include <vector>

#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class ParameterTestNode : public rclcpp::Node
{
public:
  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Time t_start;

  int int_param = 0;
  double double_param = 0.0;
  bool bool_param = false;

  ParameterTestNode();
  void timer_cb();
};

ParameterTestNode::ParameterTestNode()
: Node("parameter_test")
{
  declare_parameter<bool>("bool_param", false);
  get_parameter("bool_param", bool_param);

  declare_parameter<int>("int_param", 0);
  get_parameter("int_param", int_param);

  declare_parameter<double>("double_param", 0.0);
  get_parameter("double_param", double_param);

  t_start = get_clock()->now();

  timer = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ParameterTestNode::timer_cb, this));
}

void ParameterTestNode::timer_cb()
{
  const rclcpp::Time t(get_clock()->now());

  const double dt = (t - t_start).seconds();
  RCLCPP_INFO(
    get_logger(),
    "%.2f bool_param: %s  int_param: %d  double_param: %.3f",
    dt,
    bool_param ? "true" : "false",
    int_param,
    double_param);

  // bail after 1 second
  if ((t - t_start).seconds() > 1.0)
  {
    RCLCPP_INFO(get_logger(), "we're done.");
    rclcpp::shutdown();
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParameterTestNode>());
  rclcpp::shutdown();
  printf("exiting main()\n");
  return 0;
}
