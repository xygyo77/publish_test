#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#include "publish_test/publisher_node.hpp"
#include "publish_test/subscriber_node.hpp"

bool DEBUG = true;

int main(int argc, char* argv[]) {
  RCLCPP_INFO(rclcpp::get_logger("log"), "argc: %d", argc);
  bool is_pub = false;
  for (int i = 0; i < argc; ++i) {
      RCLCPP_INFO(rclcpp::get_logger("log"), "argv[%d]: %s",  i, argv[i]);
      if (strstr(argv[i], "__node:=publisher_node")) {
          is_pub = true; 
          break;
      }
  }

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  if (is_pub) {
    //executor->add_node(std::make_shared<PublisherNode>("publisher_base_node", options));
    executor->add_node(std::make_shared<PublisherNode>("publisher_base_node"));
    //executor->add_node(std::make_shared<PublisherNode>("publisher_var_node", options));
    executor->add_node(std::make_shared<PublisherNode>("publisher_var_node"));
    executor->spin();
  } else {
    //executor->add_node(std::make_shared<SubscriberNode>("subscriber_base_node", options));
    executor->add_node(std::make_shared<SubscriberNode>("subscriber_base_node"));
    //executor->add_node(std::make_shared<SubscriberNode>("subscriber_var_node", options));
    executor->add_node(std::make_shared<SubscriberNode>("subscriber_var_node"));
    executor->spin();
  }

  rclcpp::shutdown();
  return 0;
}