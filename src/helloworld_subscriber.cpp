#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class HelloworldSubscriber : public rclcpp::Node
{
public:
  HelloworldSubscriber()
  : Node("Helloworld_subscriber")
  {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    helloworld_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "helloworld",
        qos_profile,
        std::bind(&HelloworldSubscriber::subscribe_topic_message, this, _1));
  }

private:
  

}
