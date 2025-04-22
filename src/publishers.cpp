// Copyright 2025 Sony Group Corporation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdlib>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "common.hpp"

using namespace std::chrono_literals;

class MultiNodesPublisher : public rclcpp::Node
{
public:
MultiNodesPublisher(size_t index)
  : Node("node_pub_" + std::to_string(index),
  rclcpp::NodeOptions().enable_rosout(false).start_parameter_services(false))
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "topic_" + std::to_string(index), 10);
    auto timer_callback =
      [this]() -> void {
        // Wait for at least one receiver.
        if (this->publisher_->get_subscription_count() == 0) {
          return;
        } else {
          std::call_once(
            this->flag_,
            [this]() {
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s: subscriber is "
              "connected and start to publisher message.", this->publisher_->get_topic_name());
            });
        }

        // Publish a message.
        auto message = std_msgs::msg::Float64();
        message.data = static_cast<double>(++this->count_);
        this->publisher_->publish(message);

        // Stop sending after TOTAL_SENT_MESSAGE messages have been sent.
        if (this->count_ == TOTAL_SENT_MESSAGE) {
          this->timer_->cancel();
          RCLCPP_INFO(this->get_logger(), "Node %s finished publishing.", this->get_name());
        }
      };
    timer_ = this->create_wall_timer(std::chrono::milliseconds(SEND_INTERVAL_MS), timer_callback);
  }
private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 0;
  std::once_flag flag_;
};

void print_usage()
{
  std::cout << "ros2 run test_multi_nodes publisher [-n NUM_NODES]" << std::endl;
  std::cout << "Create NUM_NODES nodes and each node includes a publisher." << std::endl;
  std::cout << "  -n NUM_NODES: Number of nodes to create (default: 100)" << std::endl;
}

int main(int argc, char * argv[])
{
  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);

  if (non_ros_args.size() != 1 && non_ros_args.size() != 3) {
    print_usage();
    return EXIT_FAILURE;
  }

  // Get the number of nodes
  size_t num_nodes = DEFAULT_NUM_NODES;
  if (non_ros_args.size() == 3) {
    if (std::string(non_ros_args[1]) != "-n") {
      std::cout << "Invalid argument: " << non_ros_args[1] << std::endl;
      print_usage();
      return EXIT_FAILURE;
    }
    num_nodes = std::stoul(non_ros_args[2]);
    if (num_nodes == 0) {
      std::cout << "Invalid number of nodes: " << non_ros_args[2] << std::endl;
      print_usage();
      return EXIT_FAILURE;
    }
  }

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<std::shared_ptr<MultiNodesPublisher>> nodes;

  for (size_t i = 1; i <= num_nodes; ++i) {
    auto node = std::make_shared<MultiNodesPublisher>(i);
    nodes.push_back(node);
    executor.add_node(node);
  }

  while (rclcpp::ok()) {
    executor.spin_some();
    std::this_thread::sleep_for(100ms);
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
