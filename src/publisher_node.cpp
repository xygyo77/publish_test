#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <random>
#include <iostream>
#include <std_msgs/msg/string.hpp>

#include "publish_test/publisher_node.hpp"

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode(const rclcpp::NodeOptions& options)
: Node(
    "publisher_node", rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)),
    base_msg_count_(0),
    var_msg_count_(0)
{
    // command line options
    this->declare_parameter("base_topic_count", 10);
    this->declare_parameter("base_frequency", 10.0);
    this->declare_parameter("var_topic_count", 10);
    this->declare_parameter("var_frequency", 10.0);
    this->declare_parameter("var_msg_size", 100);
    this->declare_parameter("output_suppressed", false);

    auto base_topic_count = this->get_parameter("base_topic_count").as_int();
    auto base_frequency = this->get_parameter("base_frequency").as_int();
    auto var_topic_count = this->get_parameter("var_topic_count").as_int();
    auto var_frequency = this->get_parameter("var_frequency").as_int();
    auto output_suppressed = this->get_parameter("output_suppressed").as_bool();

    // create base publishers
    for ( auto idx = 0; idx < base_topic_count; ++idx ) {
        std::string topic_name = "base_topic_" + std::to_string(idx);
        auto publisher = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
        base_publishers_.push_back(publisher);
    }

    double interval_us = 1000.0 / base_frequency * 1000;
    this->create_wall_timer(
        std::chrono::microseconds(static_cast<int>(interval_us)),
        [this, output_suppressed]() {
            auto idx = 0;
            for ( const auto& publisher: base_publishers_ ) {
                auto message = std_msgs::msg::String();
                message.data = "[BASE] Hello, world! " + std::to_string(idx++) + "(" + std::to_string(base_msg_count_++) + ")";
                if (!output_suppressed) {
                    RCLCPP_INFO(this->get_logger(), "PUB: %s (%zu)", message.data.c_str(), base_msg_count_);
                }
                publisher->publish(message);
            }
        }
    );
}

private:
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> base_publishers_;
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> var_publishers_;
    int base_msg_count_;
    int var_msg_count_;
};