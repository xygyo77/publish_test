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

private:
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;

    std::mutex mtx_; 
    int topic_count_;
    int qos_depth_;
    std::string prefix_;
    int msg_counter_;
    bool output_suppressed_;
};

#endif  // SUBSCRIBER_NODE_HPP_