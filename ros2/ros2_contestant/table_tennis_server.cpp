#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_contestant/msg/stamped_blob.hpp"
using std::placeholders::_1;

class ServerNode : public rclcpp::Node
{
public:
  ros2_contestant::msg::StampedBlob tx_msg;

  rclcpp::Publisher<ros2_contestant::msg::StampedBlob>::SharedPtr pub;
  rclcpp::Subscription<ros2_contestant::msg::StampedBlob>::SharedPtr sub;

  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Time t_start, t_last_serve;

  uint64_t last_rx_counter = 0, last_tx_counter = 0;
  int max_message_count = 0;
  double startup_delay = 0;
  bool verbose = false;
  FILE *log = nullptr;
  const int MAX_BLOB_SIZE = 10000000;
  bool ball_returned = false;

  ServerNode();
  void msg_cb(const ros2_contestant::msg::StampedBlob::SharedPtr msg);
  void timer_cb();
  void serve();
};

ServerNode::ServerNode()
: Node("table_tennis_server")
{
  declare_parameter<bool>("verbose", false);
  get_parameter("verbose", verbose);

  declare_parameter<int>("max_message_count", 0);
  get_parameter("max_message_count", max_message_count);

  srandom(0);
  for (int i = 0; i < MAX_BLOB_SIZE; i++)
    tx_msg.blob.push_back(random() % 256);

  sub = create_subscription<ros2_contestant::msg::StampedBlob>(
    "ball_return",
    10,
    std::bind(&ServerNode::msg_cb, this, _1));

  pub = create_publisher<ros2_contestant::msg::StampedBlob>(
    "ball_serve",
    10);

  declare_parameter<double>("startup_delay", 1.0);
  get_parameter("startup_delay", startup_delay);

  t_start = get_clock()->now();
  t_last_serve = get_clock()->now();

  timer = create_wall_timer(
    std::chrono::milliseconds(1),
    std::bind(&ServerNode::timer_cb, this));
}

void ServerNode::msg_cb(
  const ros2_contestant::msg::StampedBlob::SharedPtr rx_msg)
{
  const double latency = (get_clock()->now() - rx_msg->stamp).seconds();

  if (rx_msg->counter == last_tx_counter)
  {
    fprintf(log, "%d,%.6f\n", (int)rx_msg->blob.size(), latency);
    ball_returned = true;
  }
  last_rx_counter = rx_msg->counter;

  if (verbose)
    RCLCPP_INFO(get_logger(), "latency: %.6f", latency);
}

void ServerNode::serve()
{
  tx_msg.counter++;
  tx_msg.stamp = get_clock()->now();
  pub->publish(tx_msg);

  last_tx_counter = tx_msg.counter;
  t_last_serve = get_clock()->now();
}

void ServerNode::timer_cb()
{
  const rclcpp::Time t(get_clock()->now());
  if ((t - t_start).seconds() < startup_delay)
    return;

  // if we haven't received the ball back, assume it got lost
  if ((t - t_last_serve).seconds() > 1.0)
  {
    fprintf(log, "%d,10.0\n", (int)tx_msg.blob.size());

    if (verbose)
    {
      RCLCPP_ERROR(
        get_logger(),
        "didn't get a response to %d-byte blob",
        (int)tx_msg.blob.size());
    }

    serve();
  }
  else if (ball_returned)
  {
    serve();
  }

  if (max_message_count && tx_msg.counter > (uint64_t)max_message_count)
  {
    RCLCPP_INFO(get_logger(), "shutting down...");
    rclcpp::shutdown();
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}
