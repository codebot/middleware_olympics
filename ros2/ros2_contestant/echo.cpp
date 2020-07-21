#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_contestant/msg/stamped_blob.hpp"
using std::placeholders::_1;

class EchoNode : public rclcpp::Node
{
public:
  rclcpp::Subscription<ros2_contestant::msg::StampedBlob>::SharedPtr sub;
  rclcpp::Publisher<ros2_contestant::msg::StampedBlob>::SharedPtr pub;

  EchoNode();
  void callback(const ros2_contestant::msg::StampedBlob::SharedPtr msg);
};

EchoNode::EchoNode()
: Node("echo_node")
{
  sub = create_subscription<ros2_contestant::msg::StampedBlob>(
    "echo_in", 10, std::bind(&EchoNode::callback, this, _1));

  pub = create_publisher<ros2_contestant::msg::StampedBlob>("echo_out", 10);
}

void EchoNode::callback(
  const ros2_contestant::msg::StampedBlob::SharedPtr msg)
{
  pub->publish(*msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EchoNode>());
  rclcpp::shutdown();
  return 0;
}
