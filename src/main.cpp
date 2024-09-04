#include <rclcpp/rclcpp.hpp>
#include "publish_test/publisher_node.hpp"
#include "publish_test/subscriber_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " [pub|sub]" << std::endl;
    return 1;
  }

  std::string mode = argv[1];

  if (mode == "pub") {
    rclcpp::spin(std::make_shared<PublisherNode>());
  } else if (mode == "sub") {
    rclcpp::spin(std::make_shared<SubscriberNode>());
  } else {
    std::cerr << "Invalid mode: " << mode << std::endl;
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}