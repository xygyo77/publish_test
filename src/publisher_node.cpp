#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <random>
#include <iostream>
#include <mutex>
#include <std_msgs/msg/string.hpp>

#include "publish_test/publisher_node.hpp"

extern bool DEBUG;
#define DB(X) {if(DEBUG) {std::cout << __func__ << ": " << __LINE__ << " " << #X << ": " << X << std::endl;}}

PublisherNode::PublisherNode(const std::string& node_name, const std::string& ns)
: Node(
    node_name, ns, rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)),
    topic_count_(0),
    frequency_(0.0),
    msg_size_(0),
    qos_depth_(0),
    msg_counter_(0),
    prefix_("base"),
    output_suppressed_(false)
{
    DB("==================================================================================")
    DB(node_name)
    // command line options
    if (ns == "base") {
        // base node parameters
        this->topic_count_ = this->get_parameter("topic_count").as_int();
        int unit = this->get_parameter("unit").as_int();
        this->unit_ = (unit > this->topic_count_)?  this->topic_count_: unit;
        this->frequency_ = this->get_parameter("frequency").as_double();
        this->msg_size_ = this->get_parameter("msg_size").as_int();
        this->qos_depth_ = this->get_parameter("qos_depth").as_int();
    } else {
        // variable node parameters
        this->topic_count_ = this->get_parameter("var_topic_count").as_int();
        this->unit_ = this->get_parameter("var_unit").as_int();
        this->unit_ = (this->unit_ > this->topic_count_)?  this->topic_count_: this->unit_;
        this->frequency_ = this->get_parameter("var_frequency").as_double();
        this->msg_size_ = this->get_parameter("var_msg_size").as_int();
        this->qos_depth_ = this->get_parameter("var_qos_depth").as_int();
        this->prefix_ = "var";
    }
    this->output_suppressed_ = this->get_parameter("output_suppressed").as_bool();
    DEBUG = !this->output_suppressed_;

    RCLCPP_INFO(this->get_logger(), "\n=== PUB: %s ===\n topic_count=%d unit=%d freq=%f size=%d qos=%d suppress=%d", this->prefix_.c_str(), this->topic_count_, this->unit_, this->frequency_, this->msg_size_, this->qos_depth_, this->output_suppressed_);

    // create base publishers
    for ( auto topic_index = 0; topic_index < this->topic_count_; ++topic_index ) {
        std::string topic_name = this->prefix_ + "_topic_" + std::to_string(topic_index);
        auto publisher = this->create_publisher<std_msgs::msg::String>(topic_name, this->qos_depth_);
        this->publishers_.push_back(publisher);
    }

    double interval_us = 1000.0 / this->frequency_ * 1000;
    RCLCPP_INFO(this->get_logger(), "interval_us=%f", interval_us);
    int timer_count = this->topic_count_/this->unit_;
    timer_count += (this->topic_count_%this->unit_)? 1: 0;
    DB(timer_count)
    for (auto timer_id = 0; timer_id < timer_count; ++timer_id) {
        DB((this->topic_count_ /this->unit_))
        DB(timer_id)
        auto timer = this->create_wall_timer(
            std::chrono::microseconds(static_cast<int>(interval_us)),
            [this, timer_id]() {
                for (auto topic_index = 0; topic_index < this->unit_ ; ++topic_index) {
                    std::lock_guard<std::mutex> lock(this->mtx_);

                    DB(timer_id)
                    DB(topic_index)
                    auto publisher_index = timer_id*this->unit_ + topic_index;
                    DB(publisher_index)
                    if (publisher_index >= this->publishers_.size()) {
                        DB("!!! loop exit !!!")
                        break;
                    }
                    auto publisher = this->publishers_[publisher_index];
                    std::stringstream ss;
                    ss << std::setw(8) << std::setfill('0') << this->msg_counter_ << "[" << this->prefix_ << "] Hello, world! " << std::to_string(publisher_index);
                    std::string header = ss.str();
                    int dummy_size = (this->msg_size_ <= header.size())? 1: this->msg_size_ - header.size();
                    std::vector<char> dummy_data(dummy_size, '-');
                    std::string data_str(dummy_data.begin(), dummy_data.end());
                    data_str.insert(0, header);
                    auto message = std_msgs::msg::String();
                    message.data = data_str;
                    if (!this->output_suppressed_) {
                        RCLCPP_INFO(this->get_logger(), "PUB: %s |%s| (%d : %zu)", this->prefix_.c_str(), message.data.c_str(), publisher_index, this->msg_counter_);
                    }
                    publisher->publish(message);
                    this->msg_counter_++;
                }
            }
        );
        this->timers_.push_back(timer);
        rclcpp::sleep_for(std::chrono::microseconds(static_cast<int>(interval_us)) / 2);
    }
}