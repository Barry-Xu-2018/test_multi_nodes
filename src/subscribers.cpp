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
#include <string>
#include <thread>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "common.hpp"

using namespace std::chrono_literals;

class MultiNodesSubscriber : public rclcpp::Node
{
public:
MultiNodesSubscriber(size_t index)
  : Node("node_sub_" + std::to_string(index))
  {
    subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "topic_" + std::to_string(index),
      10,
      [this](std_msgs::msg::Float64::UniquePtr msg) {
        (void) msg;
        ++count_;
        if (count_ == TOTAL_SENT_MESSAGE) {
          RCLCPP_INFO(this->get_logger(), "Node %s finished receiving.", this->get_name());
        }
      });
  }
bool finish_receiving()
{
  return this->count_ == TOTAL_SENT_MESSAGE;
}
private:
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
  std::atomic_uint32_t count_ = 0;
};

void print_usage()
{
  std::cout << "ros2 run test_multi_nodes subscriber [-n NUM_NODES]" << std::endl;
  std::cout << "Create NUM_NODES nodes and each node includes a subscriber." << std::endl;
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

  rclcpp::executors::MultiThreadedExecutor executor;
  std::unordered_set<std::shared_ptr<MultiNodesSubscriber>> nodes;

  for (size_t i = 1; i <= num_nodes; ++i) {
    auto node = std::make_shared<MultiNodesSubscriber>(i);
    nodes.emplace(node);
    executor.add_node(node);
  }

  while (rclcpp::ok()) {
    executor.spin_some();
    std::unordered_set<std::shared_ptr<MultiNodesSubscriber>> finished_nodes;
    for (auto & node : nodes) {
      if (node->finish_receiving()) {
        finished_nodes.insert(node);
        executor.remove_node(node);
      }
    }
    for (auto & node : finished_nodes) {
      nodes.erase(node);
    }
    if (nodes.empty()) {
      std::cout << "All nodes finished receiving." << std::endl;
      break;
    }
    std::this_thread::sleep_for(200ms);
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
