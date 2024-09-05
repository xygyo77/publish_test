#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#include "publish_test/publisher_node.hpp"
#include "publish_test/subscriber_node.hpp"

bool DEBUG = true;

int main(int argc, char* argv[]) {
  int topic_count = 0;
  int frequency = 10;
  int msg_size = 100;
  int qos = 10;
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
  if (is_pub) {
    rclcpp::spin(std::make_shared<PublisherNode>(options));
  } else {
    rclcpp::spin(std::make_shared<SubscriberNode>(options));
  }

  rclcpp::shutdown();
  return 0;
}