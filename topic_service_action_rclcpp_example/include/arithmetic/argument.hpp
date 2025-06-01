#ifndef ARITHMETIC__ARGUMENT_HPP_
#define ARITHMETIC__ARGUMENT_HPP_

#include <chrono>   // 시간
#include <memory>   // 동적 메모리
#include <string>   // 문자열 
#include <utility>  // 여러 utility 기능

#include "rclcpp/rclcpp.hpp"
#include "msg_srv_action_interface_example/msg/arithmetic_argument.hpp"

class Argument : public rclcpp::Node
{
public:
  using ArithmeticArgument = msg_srv_action_interface_example::msg::ArithmeticArgument;

  // 생성자에서 rclcpp::NodeOptions 객체를 인자로 받음. 
  // 이 객체에서는 context, arguments, intra-process communication, parameter 등 다양한 옵션 정의 가능
  explicit Argument(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Argument();

private:
  void publish_random_arithmetic_arguments();
  // 파라미터는 나중에 다룸
  void update_parameter();

  // 랜덤 변수의 샘플링 범위를 정해줄 멤버 변수
  float min_random_num_;
  float max_random_num_;

  rclcpp::Publisher<ArithmeticArgument>::SharedPtr arithmetic_argument_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
};
#endif // ARITHMETIC__ARGUMENT_HPP