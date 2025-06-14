#ifndef CHECKER__CHECKER_HPP_
#define CHECKER__CHECKER_HPP_

#include <memory.h>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "msg_srv_action_interface_example/action/arithmetic_checker.hpp"

// Checker 클래스는 생성자에서 goal_sum 변수와 rclcpp::NodeOptions를 인자로 받는다. 
// 멤버 함수로는 액션 요청을 위한 send_goal_total_sum 함수, 액션 동작 시 호출되는 콜백 함수
// 멤버 변수로는 rclcpp_action::Client 타입의 스마트 포인터를 확인할 수 있음.
class Checker : public rclcpp:Node
{
public:
  using ArithmeticChecker = msg_srv_action_interface_example::action::ArithmeticChecker;
  using GoalHandleArithmetic = rclcpp_action::ClientGoalHandle<ArithmeticChecker>;

  explicit Checker(
    float goal_sum;
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Checker();

private:
  void send_goal_total_sum(float goal_sum);

  void get_arithmetic_action_goal(
    std::shared_future<rclcpp_action::ClientGoalHandle<ArithmeticChecker>::SharedPtr> future);

  void get_arithmetic_action_feedback(
    GoalHandleArithmeticChecker::SharedPtr, 
    const std::shared_ptr<const ArithmeticChecker::Feedback> feedback);
  
  void get_arithmetic_action_result(
    const GoalHandleArithmeticChecker::WrappedResult & result);

  rclcpp_action::Client<ArithmeticChecker>::SharedPtr arithmetic_action_client_;
};
#endif // CHECKER_CHECKER_HPP_


