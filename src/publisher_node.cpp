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
    base_msg_count_(0),
    var_msg_count_(0)
{
    DB("START PUB")
    // command line options
    auto base_topic_count = this->get_parameter("topic_count").as_int();
    auto base_frequency = this->get_parameter("frequency").as_double();
    //auto base_msg_size = this->get_parameter("msg_size").as_int();
    //auto var_topic_count = this->get_parameter("var_topic_count").as_int();
    //auto var_frequency = this->get_parameter("var_frequency").as_int();
    auto output_suppressed = this->get_parameter("output_suppressed").as_bool();

    RCLCPP_INFO(this->get_logger(), "base_topic_count=%d base_frequency=%f", base_topic_count, base_frequency);

    // create base publishers
    for ( auto idx = 0; idx < base_topic_count; ++idx ) {
        std::string topic_name = "base_topic_" + std::to_string(idx);
        auto publisher = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
        base_publishers_.push_back(publisher);
    }

    double interval_us = 1000.0 / base_frequency * 1000;
    RCLCPP_INFO(this->get_logger(), "interval_us=%f", interval_us);
    this->create_wall_timer(
        std::chrono::microseconds(static_cast<int>(interval_us)),
        [this, output_suppressed]() {
            DB("TIMER")
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