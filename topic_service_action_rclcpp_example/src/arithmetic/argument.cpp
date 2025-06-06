#include <cstdio>   // C++에서 사용 가능한 C 언어 표준 입출력 라이브러리
#include <memory>   
#include <string>
#include <utility>
#include <random>   // 랜덤 숫자 생성 라이브러리

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "arithmetic/argument.hpp"

using namespace std::chrono_literals;

// --> Argument 클래스 생성자
// : 부모 클래스인 rclcpp::Node에 대한 선언 --> (Node("argument", node_options))
// 매개변수 : 노드 이름, NodeOption 객체
Argument::Argument(const rclcpp::NodeOptions & node_options)
: Node("argument", node_options),
min_random_num_(0.0),
max_random_num_(0.0)
{
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();
    this->declare_parameter("min_random_num", 0.0);
    min_random_num_ = this->get_parameter("min_random_num").get_value<float>();
    this->declare_parameter("max_random_num", 9.0);
    max_random_num_ = this->get_parameter("max_random_num").get_value<float>();
    this->update_parameter();

    // QoS 라이브러리를 통한 QoS 설정 --> 이것도 나중에 QoS 한번 제대로 다룰 예정. 1부 8장 공부
    const auto QOS_RKL10V = 
      rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
    arithmetic_argument_publisher_ = 
      this->create_publisher<ArithmeticArgument>("arithmetic_argument",QOS_RKL10V);
    
    // timer의 경우에는 1초당 한번씩 publish_random_arithmetic_arguments 멤버 함수를 호출하도록 설정
    timer_ = 
      this->create_wall_timer(1s, std::bind(&Argument::publish_random_arithmetic_arguments, this));

}

Argument::~Argument()
{

}

void Argument::publish_random_arithmetic_arguments()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> distribution(min_random_num_, max_random_num_);

  msg_srv_action_interface_example::msg::ArithmeticArgument msg;
  msg.stamp = this->now();
  msg.argument_a = distribution(gen);
  msg.argument_b = distribution(gen);
  arithmetic_argument_publisher_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Published argument_a %.2f", msg.argument_a);
  RCLCPP_INFO(this->get_logger(), "Published argument_b %.2f", msg.argument_b);
}

void Argument::update_parameter()
{
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  while (!parameters_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto param_event_callback = 
    [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
      for (auto & changed_parameter : event->changed_parameters) {
        if (changed_parameter.name == "min_random_num") {
            auto value = rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double();
            min_random_num_ = value;
        } else if (changed_parameter.name == "max_random_num") {
            auto value = rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double();
            max_random_num_ = value;
        }
      }
    };

    parameter_event_sub_ =parameters_client_->on_parameter_event(param_event_callback);
}

void print_help()
{
  printf("For argument node:\n");
  printf("node_name [-h]\n");
  printf("Options:\n");
  printf("\t-h Help             : Print this help function.\n");
}

int main(int argc, char * argv[])
{
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_help();
    return 0;
  }

  rclcpp::init(argc, argv);
  auto argument = std::make_shared<Argument>();
  rclcpp::spin(argument);
  rclcpp::shutdown();

  return 0;
}