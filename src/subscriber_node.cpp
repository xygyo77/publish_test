#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <random>
#include <iostream>
#include <std_msgs/msg/string.hpp>

#include "publish_test/subscriber_node.hpp"

SubscriberNode::SubscriberNode(const rclcpp::NodeOptions& options)
: Node(
    "subscription_node", rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)),
    base_msg_count_(0),
    var_msg_count_(0)
{
    // komanndline options
    this->declare_parameter("base_topic_count", 10);
    this->declare_parameter("var_topic_count", 10);
    this->declare_parameter("output_suppressed", false);

    std::size_t base_topic_count = this->get_parameter("base_topic_count").as_int();
    std::size_t var_topic_count = this->get_parameter("var_topic_count").as_int();
    auto output_suppressed = this->get_parameter("output_suppressed").as_bool();

    // create base subscribers
    for (auto idx = 0; idx < base_topic_count; ++idx) {
        auto topic_name = "base_topic_" + std::to_string(idx);
        base_subscriptions_.push_back(
            this->create_subscription<std_msgs::msg::String>(
                topic_name, 10,
                [this, idx, output_suppressed](const std_msgs::msg::String::SharedPtr msg) -> void 
                {
                    if (!output_suppressed) {
                        RCLCPP_INFO(this->get_logger(), "SUB: %s (%u : %u)", msg->data.c_str(), idx, base_msg_count_++);
                    }
                }
            )
        );
    }
}