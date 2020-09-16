#include <memory>
#include <random>
#include <vector>

#include <unistd.h>

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

  rclcpp::Time t_start, t_last_serve, t_last_return;

  uint64_t last_rx_counter = 0, last_tx_counter = 0;
  int max_message_count = 0;
  double startup_delay = 0;
  bool verbose = false;
  FILE *log_file = nullptr;
  const int MAX_BLOB_SIZE = 10000000;
  bool ball_returned = false;
  std::mt19937 rand_gen;
  std::uniform_real_distribution<double> blob_magnitude_dist;

  ServerNode();
  void msg_cb(const ros2_contestant::msg::StampedBlob::SharedPtr msg);
  void timer_cb();
  void serve();
};

ServerNode::ServerNode()
: Node("table_tennis_server"),
  rand_gen(1234)
{
  blob_magnitude_dist =
    std::uniform_real_distribution<double>(
      1.0,
      log(MAX_BLOB_SIZE) / log(10));

  declare_parameter<bool>("verbose", false);
  get_parameter("verbose", verbose);

  declare_parameter<int>("max_message_count", 0);
  get_parameter("max_message_count", max_message_count);

  tx_msg.blob.reserve(MAX_BLOB_SIZE);
#if 0
  srandom(0);
  for (int i = 0; i < MAX_BLOB_SIZE; i++)
    tx_msg.blob.push_back(random() % 256);
#endif

  log_file = fopen("log.txt", "w");

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
    std::chrono::milliseconds(100),
    std::bind(&ServerNode::timer_cb, this));
}

void ServerNode::msg_cb(
  const ros2_contestant::msg::StampedBlob::SharedPtr rx_msg)
{
  const double latency = (get_clock()->now() - rx_msg->stamp).seconds();

  if (rx_msg->counter == last_tx_counter)
  {
    fprintf(log_file, "%d,%.6f\n", (int)rx_msg->blob.size(), latency);
    ball_returned = true;
  }
  last_rx_counter = rx_msg->counter;

  if (verbose)
    RCLCPP_INFO(get_logger(), "latency: %.6f", latency);

  t_last_return = get_clock()->now();
}

void ServerNode::serve()
{
  const int blob_size =
    static_cast<int>(pow(10.0, blob_magnitude_dist(rand_gen)));
  tx_msg.blob.resize(blob_size);

  if (verbose)
    RCLCPP_INFO(get_logger(), "serve() size: %d", blob_size);

  tx_msg.counter++;
  tx_msg.stamp = get_clock()->now();
  pub->publish(tx_msg);

  last_tx_counter = tx_msg.counter;
  t_last_serve = tx_msg.stamp;
  ball_returned = false;
}

void ServerNode::timer_cb()
{
  const rclcpp::Time t(get_clock()->now());

  static bool startup_delay_complete = false;
  if ((t - t_start).seconds() < startup_delay)
    return;
  if (!startup_delay_complete)
  {
    startup_delay_complete = true;
    RCLCPP_INFO(get_logger(), "startup delay complete");
  }

  // if we haven't received the ball back, assume it got lost
  if ((t - t_last_serve).seconds() > 1.0)
  {
    if (verbose && tx_msg.counter > 0)
    {
      fprintf(log_file, "%d,10.0\n", (int)tx_msg.blob.size());

      RCLCPP_ERROR(
        get_logger(),
        "didn't get a response to %d-byte blob",
        (int)tx_msg.blob.size());
    }

    serve();
  }
  else if (ball_returned && (t - t_last_return).seconds() > 0.01)
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
  printf("exiting main()\n");
  return 0;
}
