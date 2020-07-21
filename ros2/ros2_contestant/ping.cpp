#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_contestant/msg/stamped_blob.hpp"
using std::placeholders::_1;

class PingNode : public rclcpp::Node
{
public:
  ros2_contestant::msg::StampedBlob tx_msg;

  rclcpp::Publisher<ros2_contestant::msg::StampedBlob>::SharedPtr pub;
  rclcpp::Subscription<ros2_contestant::msg::StampedBlob>::SharedPtr sub;

  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Time t_start, t_last_publish;

  uint64_t last_rx_counter = 0, last_tx_counter = 0;
  int max_message_count = 0;
  int num_skips = 0;
  double startup_delay = 0;
  std::vector<double> latency_measurements;
  bool verbose = false;

  PingNode();

  void msg_cb(const ros2_contestant::msg::StampedBlob::SharedPtr msg);
  void timer_cb();
  void terminate();
};

PingNode::PingNode()
: Node("ping_node")
{
  declare_parameter<bool>("verbose", false);
  get_parameter("verbose", verbose);

  declare_parameter<int>("max_message_count", 0);
  get_parameter("max_message_count", max_message_count);
  latency_measurements.reserve(max_message_count);

  declare_parameter<int>("blob_size", 0);
  int blob_size = 0;
  get_parameter("blob_size", blob_size);
  srandom(0);
  for (int i = 0; i < blob_size; i++)
    tx_msg.blob.push_back(random() % 256);

  if (verbose)
    RCLCPP_INFO(get_logger(), "blob size: %zu", tx_msg.blob.size());

  sub = create_subscription<ros2_contestant::msg::StampedBlob>(
    "echo_out", 10, std::bind(&PingNode::msg_cb, this, _1));

  pub = create_publisher<ros2_contestant::msg::StampedBlob>("echo_in", 10);

  double publish_rate = 1.0;
  declare_parameter<double>("publish_rate", 1.0);
  get_parameter("publish_rate", publish_rate);
  const double interval_seconds = 1.0 / publish_rate;

  declare_parameter<double>("startup_delay", 1.0);
  get_parameter("startup_delay", startup_delay);

  t_start = get_clock()->now();

  timer = create_wall_timer(
    std::chrono::microseconds(
      static_cast<int>(interval_seconds * 1000000)),
    std::bind(&PingNode::timer_cb, this));
}

void PingNode::msg_cb(
  const ros2_contestant::msg::StampedBlob::SharedPtr rx_msg)
{
  const double latency = (get_clock()->now() - rx_msg->stamp).seconds();

  if (rx_msg->counter == last_tx_counter)
  {
    latency_measurements.push_back(latency);
  }
  last_rx_counter = rx_msg->counter;

  if (verbose)
    RCLCPP_INFO(get_logger(), "latency: %.6f", latency);
}

void PingNode::terminate()
{
  std::sort(latency_measurements.begin(), latency_measurements.end());
  double median = latency_measurements[latency_measurements.size()/2];

  printf(
    "%.6f %.6f %.6f %d %d\n",
    median,
    latency_measurements.front(),
    latency_measurements.back(),
    num_skips,
    static_cast<int>(tx_msg.counter));

  rclcpp::shutdown();
}

void PingNode::timer_cb()
{
  if ((get_clock()->now() - t_start).seconds() < startup_delay)
    return;

  if (last_rx_counter != tx_msg.counter)
  {
    num_skips++;
  }

  tx_msg.counter++;
  tx_msg.stamp = get_clock()->now();
  pub->publish(tx_msg);

  last_tx_counter = tx_msg.counter;
  t_last_publish = get_clock()->now();

  if (max_message_count && tx_msg.counter > (uint64_t)max_message_count)
    terminate();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PingNode>());
  rclcpp::shutdown();
  return 0;
}
