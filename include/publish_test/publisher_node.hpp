#ifndef PUBLISHER_NODE_HPP_
#define PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class PublisherNode : public rclcpp::Node
{
public:
    explicit PublisherNode(const rclcpp::NodeOptions& options);

private:
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> base_publishers_;
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> var_publishers_;
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

#endif  // PUBLISHER_NODE_HPP_#ifndef PUBLISHER_NODE_HPP_
#define PUBLISHER_NODE_HPP_