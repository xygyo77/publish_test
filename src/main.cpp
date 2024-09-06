#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#include "publish_test/publisher_node.hpp"
#include "publish_test/subscriber_node.hpp"

std::shared_ptr<PublisherNode> publisher_base_node_ptr;
std::shared_ptr<PublisherNode> publisher_var_node_ptr;
std::shared_ptr<SubscriberNode> subscriber_base_node_ptr;
std::shared_ptr<SubscriberNode> subscriber_var_node_ptr;
bool DEBUG = true;

int main(int argc, char* argv[]) {
  RCLCPP_INFO(rclcpp::get_logger("log"), "argc: %d", argc);
  bool is_pub = false;
  for (int i = 0; i < argc; ++i) {
      //RCLCPP_INFO(rclcpp::get_logger("log"), "argv[%d]: %s",  i, argv[i]);
      if (strstr(argv[i], "__node:=publisher_node")) {
          is_pub = true; 
          break;
      }
  }

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  if (is_pub) {
    auto publisher_base_node = std::make_shared<PublisherNode>("publisher_node", "base");
    auto publisher_var_node = std::make_shared<PublisherNode>("publisher_node", "var");
    publisher_base_node_ptr = publisher_base_node;
    publisher_var_node_ptr = publisher_var_node;
    executor->add_node(publisher_base_node);
    executor->add_node(publisher_var_node);
    executor->spin();
  } else {
    auto subscriber_base_node = std::make_shared<SubscriberNode>("subscriber_node", "base");
    auto subscriber_var_node = std::make_shared<SubscriberNode>("subscriber_node", "var");
    subscriber_base_node_ptr = subscriber_base_node;
    subscriber_var_node_ptr = subscriber_var_node;
    executor->add_node(subscriber_base_node);
    executor->add_node(subscriber_var_node);
    executor->spin();
  }

  rclcpp::shutdown();
  return 0;
}