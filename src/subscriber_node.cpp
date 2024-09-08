#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <random>
#include <iostream>
#include <mutex>
#include <std_msgs/msg/string.hpp>

#include "publish_test/subscriber_node.hpp"

extern bool DEBUG;
#define DB(X) {if(DEBUG) {std::cout << __func__ << ": " << __LINE__ << " " << #X << ": " << X << std::endl;}}

SubscriberNode::SubscriberNode(const std::string& node_name, const std::string& ns)
: Node(
    node_name, ns, rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)),
    topic_count_(0),
    qos_depth_(0),
    msg_counter_(0),
    prefix_("base"),
    output_suppressed_(false)
{
    DB("++++++++++++++++");
    DB(node_name)
    // comanndline options
    if (ns == "base") {
        // base node parameters
        this->topic_count_ = this->get_parameter("topic_count").as_int();
        this->qos_depth_ = this->get_parameter("qos_depth").as_int();
    } else {
        // variable node parameters
        this->topic_count_ = this->get_parameter("var_topic_count").as_int();
        this->qos_depth_ = this->get_parameter("var_qos_depth").as_int();
        this->prefix_ = "var";
    }
    this->output_suppressed_ = this->get_parameter("output_suppressed").as_bool();
    DEBUG = !this->output_suppressed_;

    RCLCPP_INFO(this->get_logger(), "\n=== SUB: %s ===\n topic_count=%d qos=%d suppress=%d", this->prefix_.c_str(), this->topic_count_, this->qos_depth_, this->output_suppressed_);

    // create base subscribers
    for (auto topic_index = 0; topic_index < this->topic_count_; ++topic_index) {
        auto topic_name = this->prefix_ + "_topic_" + std::to_string(topic_index);
        this->subscriptions_.push_back(
            this->create_subscription<std_msgs::msg::String>(
                topic_name, this->qos_depth_,
                [this, topic_index](const std_msgs::msg::String::SharedPtr msg) -> void 
                {
                    std::lock_guard<std::mutex> lock(this->mtx_);

                    if (!this->output_suppressed_) {
                        RCLCPP_INFO(this->get_logger(), "SUB: %s |%s| (%u : %u)", this->prefix_.c_str(), msg->data.c_str(), topic_index, this->msg_counter_);
                    }
                    this->msg_counter_++;
                }
            )
        );
    }
}