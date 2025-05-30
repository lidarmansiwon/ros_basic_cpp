// Copyright 2021 OROCA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>

#include "calculator/calculator.hpp"

// calculator 노드는 sub으로 받은 멤버 변수 argument_a, b와 operator 노드로부터 요청값으로 받은 연산자를 이용하여 연산한다.
// 그리고 operator 노드에게 연산의 결괏값을 서비스 응답값으로 보낸다. 

Calculator::Calculator(const rclcpp::NodeOptions & node_options)
: Node("calculator", node_options),
  argument_a_(0.0),
  argument_b_(0.0),
  argument_operator_(0),
  argument_result_(0.0),
  argument_formula_("")
{
  // 생성자에서 시작 표시.
  RCLCPP_INFO(this->get_logger(), "Run calculator");

  operator_.reserve(4);
  operator_.push_back("+");
  operator_.push_back("-");
  operator_.push_back("*");
  operator_.push_back("/");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  // subscriber 객체에서 callback 함수를 따로 정의하지 않고(std::bind를 사용하지 않음), 람다 함수로 작성. 
  arithmetic_argument_subscriber_ = this->create_subscription<ArithmeticArgument>(
    "arithmetic_argument",
    QOS_RKL10V,
    // 람다 함수에서 클래스 안의 멤버 변수나 멤버 함수를 람다 함수 안에서 사용하기 위해서
    // 클래스의 this 포인터를 명시적으로 람다에 "캡처" 
    // [캡쳐](sub 하는 msg 형식) -> return type (void로 정의하여 아무것도 반환 안함)
    [this](const ArithmeticArgument::SharedPtr msg) -> void
    {
      argument_a_ = msg->argument_a; // 멤버 변수에 저장
      argument_b_ = msg->argument_b; // 멤버 변수에 저장
      // this로 캡쳐한 class의 멤버 변수에 접근해서 저장하는 것.

      RCLCPP_INFO(
        this->get_logger(),
        "Timestamp of the message: sec %ld nanosec %ld",
        msg->stamp.sec,
        msg->stamp.nanosec);

      RCLCPP_INFO(this->get_logger(), "Subscribed argument a: %.2f", argument_a_);
      RCLCPP_INFO(this->get_logger(), "Subscribed argument b: %.2f", argument_b_);
    }
  );

  // 실제 서비스 요청에 해당되는 특정 수행 코드가 수행되는 부분.
  // 서비스 요청 --> request, 서비스 응답 --> response.
  // 아래 함수는 서비스 요청이 있을 때, 실행되는 콜백 함수.
  // 이전에 토픽 subscriber가 받아 저장해둔 멤버 변수 a, b를 연산한 후, 그 결괏값을 서비스 응답값으로 반환.
  auto get_arithmetic_operator =
    [this](
    const std::shared_ptr<ArithmeticOperator::Request> request,
    std::shared_ptr<ArithmeticOperator::Response> response) -> void
    {
      // request->arithmetic_operator를 argument_operator_
      argument_operator_ = request->arithmetic_operator;
      argument_result_ =
        this->calculate_given_formula(argument_a_, argument_b_, argument_operator_);
      response->arithmetic_result = argument_result_;

      std::ostringstream oss;
      oss << std::to_string(argument_a_) << ' ' <<
        operator_[argument_operator_ - 1] << ' ' <<
        std::to_string(argument_b_) << " = " <<
        argument_result_ << std::endl;
      argument_formula_ = oss.str();

      RCLCPP_INFO(this->get_logger(), "%s", argument_formula_.c_str());
    };

  // 서비스 서버 선언
  // public으로 정의한 부모 클래스인 Node 클래스의 함수, create_service를 사용하여 서비스 서버로 선언
  // 서비스의 타입은 ArithmeticOperator, 서비스 이름은 arithmetic_operator, 서비스 콜백 함수는 get_arithmetic_operator
  arithmetic_argument_server_ =
    create_service<ArithmeticOperator>("arithmetic_operator", get_arithmetic_operator);

  using namespace std::placeholders;
  arithmetic_action_server_ = rclcpp_action::create_server<ArithmeticChecker>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "arithmetic_checker",
    std::bind(&Calculator::handle_goal, this, _1, _2),
    std::bind(&Calculator::handle_cancel, this, _1),
    std::bind(&Calculator::execute_checker, this, _1)
  );
}

Calculator::~Calculator()
{
}

float Calculator::calculate_given_formula(
  const float & a,
  const float & b,
  const int8_t & operators)
{
  float argument_result = 0.0;
  ArithmeticOperator::Request arithmetic_operator;

  if (operators == arithmetic_operator.PLUS) {
    argument_result = a + b;
  } else if (operators == arithmetic_operator.MINUS) {
    argument_result = a - b;
  } else if (operators == arithmetic_operator.MULTIPLY) {
    argument_result = a * b;
  } else if (operators == arithmetic_operator.DIVISION) {
    argument_result = a / b;
    if (b == 0.0) {
      RCLCPP_ERROR(this->get_logger(), "ZeroDivisionError!");
      argument_result = 0.0;
      return argument_result;
    }
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Please make sure arithmetic operator(plus, minus, multiply, division).");
    argument_result = 0.0;
  }

  return argument_result;
}

rclcpp_action::GoalResponse Calculator::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ArithmeticChecker::Goal> goal)
{
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Calculator::handle_cancel(
  const std::shared_ptr<GoalHandleArithmeticChecker> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Calculator::execute_checker(const std::shared_ptr<GoalHandleArithmeticChecker> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Execute arithmetic_checker action!");
  rclcpp::Rate loop_rate(1);

  auto feedback_msg = std::make_shared<ArithmeticChecker::Feedback>();
  float total_sum = 0.0;
  float goal_sum = goal_handle->get_goal()->goal_sum;

  while ((total_sum < goal_sum) && rclcpp::ok()) {
    total_sum += argument_result_;
    feedback_msg->formula.push_back(argument_formula_);
    if (argument_formula_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Please check your formula");
      break;
    }
    RCLCPP_INFO(this->get_logger(), "Feedback: ");
    for (const auto & formula : feedback_msg->formula) {
      RCLCPP_INFO(this->get_logger(), "\t%s", formula.c_str());
    }
    goal_handle->publish_feedback(feedback_msg);
    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    auto result = std::make_shared<ArithmeticChecker::Result>();
    result->all_formula = feedback_msg->formula;
    result->total_sum = total_sum;
    goal_handle->succeed(result);
  }
}
