#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_contestant/msg/stamped_blob.hpp"
using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode();
  void callback(const ros2_contestant::msg::StampedBlob::SharedPtr msg);

  rclcpp::Subscription<ros2_contestant::msg::StampedBlob>::SharedPtr sub;
};

SubscriberNode::SubscriberNode()
: Node("subscriber_node")
{
  sub = create_subscription<ros2_contestant::msg::StampedBlob>(
    "blob", 10, std::bind(&SubscriberNode::callback, this, _1));
}

void SubscriberNode::callback(
  const ros2_contestant::msg::StampedBlob::SharedPtr msg)
{
  double dt = (get_clock()->now() - msg->stamp).seconds();
  RCLCPP_INFO(get_logger(), "latency: %.6f", dt);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
