#ifndef SUBSCRIBER_NODE_HPP_
#define SUBSCRIBER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SubscriberNode : public rclcpp::Node
{
public:
    explicit SubscriberNode(const std::string& node_name, const std::string& ns);
    
    std::string get_prefix() {return prefix_;}
    int get_msg_counter() {return msg_counter_;}
    int get_rx_ok() {return rx_ok_;}
    int get_rx_loss() {return rx_loss_;}
    int get_rx_error() {return rx_error_;}

private:
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;

    int topic_count_;
    int qos_depth_;
    std::string prefix_;
    int msg_counter_;
    int rx_ok_;
    int rx_loss_;
    int rx_error_;
    bool output_suppressed_;
};

#endif  // SUBSCRIBER_NODE_HPP_