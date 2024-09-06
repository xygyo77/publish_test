#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <random>
#include <iostream>
#include <std_msgs/msg/string.hpp>

#include "publish_test/subscriber_node.hpp"

extern bool DEBUG;
#define DB(X) {if(DEBUG) {std::cout << __func__ << ": " << __LINE__ << " " << X << std::endl;}}

SubscriberNode::SubscriberNode(const std::string& node_name, const std::string& ns)
: Node(
    node_name, ns, rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)),
    topic_count_(0),
    qos_depth_(0),
    output_suppressed_(false)
{
    DB("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    DB(node_name)
    // comanndline options
    //if (node_name.find("base") != std::string::npos) {
    if (ns == "base") {
        // base node parameters
        this->topic_count_ = this->get_parameter("topic_count").as_int();
        this->qos_depth_ = this->get_parameter("qos_depth").as_int();
        this->prefix_ = "base";
    } else {
        // variable node parameters
        this->topic_count_ = this->get_parameter("var_topic_count").as_int();
        this->qos_depth_ = this->get_parameter("var_qos_depth").as_int();
        this->prefix_ = "var";
    }
    this->output_suppressed_ = this->get_parameter("output_suppressed").as_bool();

    // create base subscribers
    for (auto idx = 0; idx < this->topic_count_; ++idx) {
        auto topic_name = this->prefix_ + "_topic_" + std::to_string(idx);
        this->base_subscriptions_.push_back(
            this->create_subscription<std_msgs::msg::String>(
                topic_name, this->qos_depth_,
                [this, idx](const std_msgs::msg::String::SharedPtr msg) -> void 
                {
                    if (!this->output_suppressed_) {
                        RCLCPP_INFO(this->get_logger(), "SUB: %s (%u : %u)", msg->data.c_str(), idx, this->topic_count_++);
                    }
                }
            )
        );
    }
}