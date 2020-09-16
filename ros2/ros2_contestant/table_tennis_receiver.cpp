#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_contestant/msg/stamped_blob.hpp"
using std::placeholders::_1;

class ReceiverNode : public rclcpp::Node
{
public:
  rclcpp::Subscription<ros2_contestant::msg::StampedBlob>::SharedPtr sub;
  rclcpp::Publisher<ros2_contestant::msg::StampedBlob>::SharedPtr pub;

  ReceiverNode()
  : Node("table_tennis_receiver")
  {
    sub = create_subscription<ros2_contestant::msg::StampedBlob>(
      "ball_serve",
      10,
      std::bind(&ReceiverNode::callback, this, _1));

    pub = create_publisher<ros2_contestant::msg::StampedBlob>(
      "ball_return",
      10);
  }

  void callback(const ros2_contestant::msg::StampedBlob::SharedPtr msg)
  {
    pub->publish(*msg);
    printf("callback\n");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReceiverNode>());
  rclcpp::shutdown();
  return 0;
}
