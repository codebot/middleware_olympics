#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "ros2_contestant/msg/stamped_blob.hpp"
using namespace std::chrono_literals;


class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode();
  void timer_callback();

  ros2_contestant::msg::StampedBlob msg;
  rclcpp::Publisher<ros2_contestant::msg::StampedBlob>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;
};

PublisherNode::PublisherNode()
: Node("publisher_node")
{
  declare_parameter<int>("blob_size", 0);
  int blob_size = 0;
  get_parameter("blob_size", blob_size);
  srandom(0);
  for (int i = 0; i < blob_size; i++)
    msg.blob.push_back(random() % 256);
  RCLCPP_INFO(get_logger(), "blob size: %zu", msg.blob.size());

  double publish_rate = 1.0;
  declare_parameter<double>("publish_rate", 1.0);
  get_parameter("publish_rate", publish_rate);
  const double interval_seconds = 1.0 / publish_rate;
  RCLCPP_INFO(
    get_logger(),
    "rate: %.6f  interval: %.6f",
    publish_rate,
    interval_seconds);

  pub = create_publisher<ros2_contestant::msg::StampedBlob>("blob", 10);
  timer = create_wall_timer(
    std::chrono::microseconds(
      static_cast<int>(interval_seconds * 1000000)),
    std::bind(&PublisherNode::timer_callback, this));
}

void PublisherNode::timer_callback()
{
  msg.stamp = get_clock()->now();
  msg.counter++;
  pub->publish(msg);
  RCLCPP_INFO(get_logger(), "publishing");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherNode>());
  rclcpp::shutdown();
  return 0;
}