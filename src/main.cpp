#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#include "publish_test/publisher_node.hpp"
#include "publish_test/subscriber_node.hpp"

std::shared_ptr<PublisherNode> publisher_base_node_ptr;
std::shared_ptr<PublisherNode> publisher_var_node_ptr;
std::shared_ptr<SubscriberNode> subscriber_base_node_ptr;
std::shared_ptr<SubscriberNode> subscriber_var_node_ptr;
bool DEBUG = true;


void signal_pub_handler(int signal) {
    (void)signal;

    RCLCPP_INFO(rclcpp::get_logger("LOG"), "\n================ %u", gettid());
    std::string ns = publisher_base_node_ptr->get_prefix();
    int msg_counter = publisher_base_node_ptr->get_msg_counter()- 1;
    RCLCPP_INFO(rclcpp::get_logger("LOG"), "\n=== PUB: %s ===\n  TX=%d", ns.c_str(), msg_counter);
    ns = publisher_var_node_ptr->get_prefix();
    msg_counter = publisher_var_node_ptr->get_msg_counter() - 1;
    RCLCPP_INFO(rclcpp::get_logger("LOG"), "\n=== PUB: %s ===\n  TX=%d", ns.c_str(), msg_counter);
    sleep(1);
    rclcpp::shutdown();
}

void signal_sub_handler(int signal) {
    (void)signal;

    RCLCPP_INFO(rclcpp::get_logger("LOG"), "\n++++++++++++++++ %u", gettid());
    std::string ns = subscriber_base_node_ptr->get_prefix();
    int msg_counter = subscriber_base_node_ptr->get_msg_counter()- 1;
    RCLCPP_INFO(rclcpp::get_logger("LOG"), "\n=== SUB: %s ===\n  RX=%d", ns.c_str(), msg_counter);
    ns = subscriber_var_node_ptr->get_prefix();
    msg_counter = subscriber_var_node_ptr->get_msg_counter()- 1;
    RCLCPP_INFO(rclcpp::get_logger("LOG"), "\n=== SUB: %s ===\n  RX=%d", ns.c_str(), msg_counter);
    sleep(1);
    rclcpp::shutdown();
}

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
    signal(SIGINT, signal_pub_handler);
    auto publisher_base_node = std::make_shared<PublisherNode>("publisher_node", "base");
    auto publisher_var_node = std::make_shared<PublisherNode>("publisher_node", "var");
    publisher_base_node_ptr = publisher_base_node;
    publisher_var_node_ptr = publisher_var_node;
    executor->add_node(publisher_base_node);
    executor->add_node(publisher_var_node);
    executor->spin();
  } else {
    signal(SIGINT, signal_sub_handler);
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