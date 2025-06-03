#include <fcntl.h>
#include <getopt.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "arithmetic/operator.hpp"

using namespace std::chrono_literals;

Operator::Operator(const rclcpp::NodeOptions & node_options)
: Node("opterator", node_options)
{
    arithmetic_service_client_ = this->create_client<ArithmeticOperator>("arithmetic_operator");
    while (!arithmetic_service_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }
}

Operator::~Operator()
{

}

void Operator::send_request()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> distribution(1,4);

    auto request = std::make_shared<ArithmeticOperator::Request>();
    request->arithmetic_operator = distribution(gen);

    using ServiceResponseFuture = rclcpp::Client<ArithmeticOperator>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Result %.2f", response->arithmetic_result);
        return;
    };

    auto future_result = 
      arithmetic_service_client_->async_send_request(request, response_received_callback);
}

int getch()
{
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

bool pull_trigger()
{
  const uint8_t KEY_ENTER = 10;
  while (1) {
    if (kbhit()) {
      uint8_t c = getch();
      if (c == KEY_ENTER) {
        return true;
      } else {
        return false;
      }
    }
  }
  return false;
}

void print_help()
{
  printf("For operator node:\n");
  printf("node_name [-h]\n");
  printf("Options:\n");
  printf("\t-h Help           : Print this help function.\n");
}

// Operator 클래스를 객체화하고 반복문을 통하여 send_request 함수를 호출하는 로직을 가짐.
int main(int argc, char * argv[])
{
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_help();
    return 0;
  }

  rclcpp::init(argc, argv);

  auto operator_node = std::make_shared<Operator>();

  while (rclcpp::ok()) {
    rclcpp::spin_some(operator_node);
    operator_node->send_request();

    // pull_trigger 함수를 통해 서비스 요청이 끝난 다음 키보드 요청을 기다린다. 
    // 만약 한 번 더 서비스 요청을 하려면 ENTER key를 누르고, 종료하려면 다른 버튼
    printf("Press Enter for next service call.\n");
    if (pull_trigger() == false) {
      rclcpp::shutdown();
      return 0;
    }
  }
}