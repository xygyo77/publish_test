#ifndef SUBSCRIBER_NODE_HPP_
#define SUBSCRIBER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SubscriberNode : public rclcpp::Node
{
public:
    explicit SubscriberNode(const rclcpp::NodeOptions& options);

private:
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> base_subscriptions_;
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> var_subscriptions_;
    int base_msg_count_;
    int var_msg_count_;
};

#endif  // SUBSCRIBER_NODE_HPP_