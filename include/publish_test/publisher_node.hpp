#ifndef PUBLISHER_NODE_HPP_
#define PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class PublisherNode : public rclcpp::Node
{
public:
    explicit PublisherNode(const std::string& node_name, const std::string& ns);

    std::string get_prefix() {return prefix_;}
    int get_msg_counter() {return msg_counter_;}

private:
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> base_publishers_;
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> var_publishers_;
    rclcpp::TimerBase::SharedPtr timer_;

    int topic_count_;
    double frequency_;
    int msg_size_;
    int qos_depth_;
    int msg_counter_;
    std::string prefix_;
    bool output_suppressed_;
};

#endif  // PUBLISHER_NODE_HPP_#ifndef PUBLISHER_NODE_HPP_
#define PUBLISHER_NODE_HPP_