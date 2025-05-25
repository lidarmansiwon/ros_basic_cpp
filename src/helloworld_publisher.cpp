#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// namespace로 지정함으로, 시간을 가식성이 높은 문자로 표현하기 위해서. ex) 1s, 500ms
using namespace std::chrono_literals;

// main class를 정의, rclcpp의 Node 클래스를 상속
class HelloworldPublihser : public rclcpp::Node
{
public:
  // 클래스 생성자 정의로 Node, count_(0)과 같이 부모 클래스 Node의 생성자를 호출하고 노드 이름을 지정.
  HelloworldPublihser()
  : Node("helloworld_publisher"), count_(0)
  {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    helloworld_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "helloworld", qos_profile);
    timer_ = this->create_wall_timer(
        1s, std::bind(&HelloworldPublihser::publish_helloworld_msg, this));
  }

private:
  void publish_helloworld_msg()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello World: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Published message: '%s'", msg.data.c_str());
    helloworld_publisher_->publish(msg);

  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr helloworld_publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HelloworldPublihser>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}