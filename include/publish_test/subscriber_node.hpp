#ifndef SUBSCRIBER_NODE_HPP_
#define SUBSCRIBER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SubscriberNode : public rclcpp::Node
{
public:
    //explicit SubscriberNode(const std::string& node_name, const rclcpp::NodeOptions& options);
    explicit SubscriberNode(const std::string& node_name);

private:
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> base_subscriptions_;
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> var_subscriptions_;
    int base_topic_count_;
    double base_frequency_;
    int base_msg_size_;
    int base_qos_;
    int var_topic_count_;
    double var_frequency_;
    int var_msg_size_;
    int var_qos_;
    bool output_suppressed_;
};

#endif  // SUBSCRIBER_NODE_HPP_