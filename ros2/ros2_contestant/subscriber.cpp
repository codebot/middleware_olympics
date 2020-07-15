#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_contestant/msg/stamped_blob.hpp"
using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node
{
public:
  rclcpp::Subscription<ros2_contestant::msg::StampedBlob>::SharedPtr sub;
  uint64_t expected_counter = 1;
  int max_message_count = 0;
  int skips = 0;
  std::vector<double> latency_measurements;

  SubscriberNode();
  void callback(const ros2_contestant::msg::StampedBlob::SharedPtr msg);
};

SubscriberNode::SubscriberNode()
: Node("subscriber_node")
{
  declare_parameter<int>("max_message_count", 0);
  get_parameter("max_message_count", max_message_count);
  latency_measurements.reserve(max_message_count);

  sub = create_subscription<ros2_contestant::msg::StampedBlob>(
    "blob", 10, std::bind(&SubscriberNode::callback, this, _1));
}

void SubscriberNode::callback(
  const ros2_contestant::msg::StampedBlob::SharedPtr msg)
{
  const double latency = (get_clock()->now() - msg->stamp).seconds();
  latency_measurements.push_back(latency);

  if (expected_counter != msg->counter)
  {
    skips++;
  }
  expected_counter = msg->counter + 1;

  if (max_message_count && msg->counter > (uint64_t)max_message_count)
  {
    std::sort(latency_measurements.begin(), latency_measurements.end());
    double median = latency_measurements[latency_measurements.size()/2];

    printf(
      "%.6f %.6f %.6f %d\n",
      median,
      latency_measurements.front(),
      latency_measurements.back(),
      skips);

    rclcpp::shutdown();
  }

  // RCLCPP_INFO(get_logger(), "latency: %.6f", latency);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
