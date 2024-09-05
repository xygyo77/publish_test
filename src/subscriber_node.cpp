#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <random>
#include <iostream>
#include <std_msgs/msg/string.hpp>

#include "publish_test/subscriber_node.hpp"

extern bool DEBUG;
#define DB(X) {if(DEBUG) {std::cout << __func__ << ": " << __LINE__ << " " << X << std::endl;}}

//SubscriberNode::SubscriberNode(const std::string& node_name, const rclcpp::NodeOptions& options)
SubscriberNode::SubscriberNode(const std::string& node_name)
: Node(
    node_name, rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)),
    base_topic_count_(0),
    base_frequency_(0.0),
    base_msg_size_(0),
    base_qos_(0),
    var_topic_count_(0),
    var_qos_(0),
    output_suppressed_(false)
{
    // komanndline options
    DB(node_name)
    //this->declare_parameter("topic_count", 5);
    DB(1)
    //this->declare_parameter("var_topic_count", 5);
    //this->declare_parameter("output_suppressed", false);
    DB(2)

    this->base_topic_count_ = this->get_parameter("topic_count").as_int();
    this->base_qos_ = this->get_parameter("qos").as_int();
    //this->var_topic_count = this->get_parameter("var_topic_count").as_int();
    //this->var_qos_ = this->get_parameter("var_qos_").as_int();
    this->output_suppressed_ = this->get_parameter("output_suppressed").as_bool();
    DB(3)

    // create base subscribers
    for (auto idx = 0; idx < this->base_topic_count_; ++idx) {
        auto topic_name = "base_topic_" + std::to_string(idx);
        this->base_subscriptions_.push_back(
            this->create_subscription<std_msgs::msg::String>(
                topic_name, 10,
                [this, idx](const std_msgs::msg::String::SharedPtr msg) -> void 
                {
                    if (!this->output_suppressed_) {
                        RCLCPP_INFO(this->get_logger(), "SUB: %s (%u : %u)", msg->data.c_str(), idx, this->base_topic_count_++);
                    }
                }
            )
        );
    }
}