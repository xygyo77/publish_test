#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <random>
#include <iostream>
#include <mutex>
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
    msg_counter_(0),
    prefix_("base"),
    max_rx_serial_num_(0),
    rx_ok_(0),
    rx_loss_(0),
    rx_error_(0),
    output_suppressed_(false)
{
    DB("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
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

                    #if 0
                    std::string rx_serial_num_str =msg->data.substr(0, 8);
                    int rx_serial_num = std::stoi(rx_serial_num_str);
                    if (rx_serial_num == this->msg_counter_) {
                        this->rx_ok_++;
                        this->msg_counter_ = rx_serial_num + 1;
                        if (this->max_rx_serial_num_ > rx_serial_num) {   
                            this->rx_loss_--;
                        } else {
                            this->max_rx_serial_num_ = rx_serial_num + 1;
                        }
                    } else if (rx_serial_num > this->msg_counter_) {
                        RCLCPP_INFO(this->get_logger(), "SUB: %s |%s|", this->prefix_.c_str(), msg->data.c_str());
                        this->max_rx_serial_num_ = rx_serial_num + 1;
                        this->rx_loss_ += (rx_serial_num - this->msg_counter_);
                    } else {
                        // error
                        this->rx_error_++;
                    }
                    RCLCPP_INFO(this->get_logger(), "SUB: %s rx=%d mag_count=%d rx_max=%d OK=%d LOSS=%d ERR=%d", 
                        this->prefix_.c_str(), rx_serial_num, this->msg_counter_, this->max_rx_serial_num_, this->rx_ok_, this->rx_loss_, this->rx_error_);
                    if (!this->output_suppressed_) {
                        RCLCPP_INFO(this->get_logger(), "SUB: %s |%s| (%u : %u)", this->prefix_.c_str(), msg->data.c_str(), topic_index, this->msg_counter_);
                    }
                    #else
                    if (!this->output_suppressed_) {
                        RCLCPP_INFO(this->get_logger(), "SUB: %s |%s| (%u : %u)", this->prefix_.c_str(), msg->data.c_str(), topic_index, this->msg_counter_);
                    }
                    this->msg_counter_++;
                    #endif
                }
            )
        );
    }
}