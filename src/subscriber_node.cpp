#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <chrono>
#include <random>
#include <iostream>
#include <std_msgs/msg/string.hpp>

#include "publish_test/subscriber_node.hpp"

extern std::shared_ptr<SubscriberNode> subscriber_base_node_ptr;
extern std::shared_ptr<SubscriberNode> subscriber_var_node_ptr;
extern bool DEBUG;
#define DB(X) {if(DEBUG) {std::cout << __func__ << ": " << __LINE__ << " " << X << std::endl;}}

void signal_sub_handler(int signal) {
    (void)signal;

    RCLCPP_INFO(rclcpp::get_logger("LOG"), "\n######################: %u", gettid());
    std::string ns = subscriber_base_node_ptr->get_prefix();
    int rx_ok = subscriber_base_node_ptr->get_rx_ok();
    int rx_loss = subscriber_base_node_ptr->get_rx_loss();
    int rx_error = subscriber_base_node_ptr->get_rx_error();
    int msg_counter = subscriber_base_node_ptr->get_msg_counter()- 1;
    RCLCPP_INFO(rclcpp::get_logger("LOG"), "=== SUB: %s ===\n  RX_OK=%d LOSS=%d ERROR=%d msg_count=%d", ns.c_str(), rx_ok, rx_loss, rx_error, msg_counter);
    ns = subscriber_var_node_ptr->get_prefix();
    rx_ok = subscriber_var_node_ptr->get_rx_ok();
    rx_loss = subscriber_var_node_ptr->get_rx_loss();
    rx_error = subscriber_var_node_ptr->get_rx_error();
    msg_counter = subscriber_var_node_ptr->get_msg_counter()- 1;
    RCLCPP_INFO(rclcpp::get_logger("LOG"), "=== SUB: %s ===\n  RX_OK=%d LOSS=%d ERROR=%d msg_count=%d", ns.c_str(), rx_ok, rx_loss, rx_error, msg_counter);
    sleep(5);
    rclcpp::shutdown();
}

SubscriberNode::SubscriberNode(const std::string& node_name, const std::string& ns)
: Node(
    node_name, ns, rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)),
    topic_count_(0),
    qos_depth_(0),
    msg_counter_(0),
    prefix_("base"),
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

    RCLCPP_INFO(this->get_logger(), "\n=== SUB: %s ===\n topic_count=%d qos=%d suppress=%d", this->prefix_.c_str(), this->topic_count_, this->qos_depth_, this->output_suppressed_);

    signal(SIGINT, signal_sub_handler);

    // create base subscribers
    for (auto idx = 0; idx < this->topic_count_; ++idx) {
        auto topic_name = this->prefix_ + "_topic_" + std::to_string(idx);
        this->base_subscriptions_.push_back(
            this->create_subscription<std_msgs::msg::String>(
                topic_name, this->qos_depth_,
                [this, idx](const std_msgs::msg::String::SharedPtr msg) -> void 
                {
                    std::string rx_serial_num_str =msg->data.substr(0, 8);
                    int rx_serial_num = std::stoi(rx_serial_num_str);
                    if (rx_serial_num == this->msg_counter_) {
                        this->rx_ok_ ++;
                    } else if (rx_serial_num > this->msg_counter_) {
                        this->rx_loss_ += (rx_serial_num - this->msg_counter_);
                        this->msg_counter_ = rx_serial_num;
                    } else {
                        this->rx_error_++;
                        this->msg_counter_ = rx_serial_num;
                    }
                    if (!this->output_suppressed_) {
                        RCLCPP_INFO(this->get_logger(), "SUB: %s |%s| (%u : %u)", this->prefix_.c_str(), msg->data.c_str(), idx, this->msg_counter_);
                    }
                    this->msg_counter_++;
                }
            )
        );
    }
}