#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#include "publish_test/publisher_node.hpp"
#include "publish_test/subscriber_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::ParameterValue pub_sub_type_value;
  rclcpp::Parameter("/pub_sub_type", pub_sub_type_value);

  std::string mode = pub_sub_type_value.get<std::string>();

  rclcpp::NodeOptions options;
  if (mode == "pub") {
    rclcpp::spin(std::make_shared<PublisherNode>(options));
  } else if (mode == "sub") {
    rclcpp::spin(std::make_shared<SubscriberNode>(options));
  } else {
    std::cerr << "Invalid mode: " << mode << std::endl;
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}