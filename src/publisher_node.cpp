#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <random>
#include <iostream>
#include <std_msgs/msg/string.hpp>

#include "publish_test/publisher_node.hpp"

extern bool DEBUG;
#define DB(X) {if(DEBUG) {std::cout << __func__ << ": " << __LINE__ << " " << X << std::endl;}}

PublisherNode::PublisherNode(const rclcpp::NodeOptions& options)
: Node(
    "publisher_node", rclcpp::NodeOptions()
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
    DB("START PUB")
    // command line options
    this->base_topic_count_ = this->get_parameter("topic_count").as_int();
    this->base_frequency_ = this->get_parameter("frequency").as_double();
    this->base_msg_size_ = this->get_parameter("msg_size").as_int();
    this->base_qos_ = this->get_parameter("qos").as_int();
    //this->var_topic_count = this->get_parameter("var_topic_count").as_int();
    //this->var_qos = this->get_parameter("var_qos").as_int();
    this->output_suppressed_ = this->get_parameter("output_suppressed").as_bool();

    RCLCPP_INFO(this->get_logger(), "base_topic_count=%d base_frequency=%f", this->base_topic_count_, this->base_frequency_);

    // create base publishers
    for ( auto idx = 0; idx < this->base_topic_count_; ++idx ) {
        std::string topic_name = "base_topic_" + std::to_string(idx);
        auto publisher = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
        this->base_publishers_.push_back(publisher);
    }

    double interval_us = 1000.0 / this->base_frequency_ * 1000;
    RCLCPP_INFO(this->get_logger(), "interval_us=%f", interval_us);
    auto timer = this->create_wall_timer(
        std::chrono::microseconds(static_cast<int>(interval_us)),
        [this]() {
            DB("TO")
            auto message = std_msgs::msg::String();
            message.data = "Hello, world! " + std::to_string(this->base_topic_count_);
            RCLCPP_INFO(this->get_logger(), "PUB: %s (%zu)", message.data.c_str(), this->base_topic_count_++);
            this->base_publishers_[0]->publish(message);
/***
        [this]() {
            DB("TIMER")
            auto idx = 0;
            for ( const auto& publisher: base_publishers_ ) {
                auto message = std_msgs::msg::String();
                message.data = "[BASE] Hello, world! " + std::to_string(idx++) + "(" + std::to_string(base_msg_count_++) + ")";
                if (!this->output_suppressed_) {
                    RCLCPP_INFO(this->get_logger(), "PUB: %s (%zu)", message.data.c_str(), base_msg_count_);
                }
                publisher->publish(message);
            }
***/
        }
    );
}