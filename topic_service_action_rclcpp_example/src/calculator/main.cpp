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

#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
// 메인문의 인자를 확인할 수 있게 됨.
#include "rcutils/cmdline_parser.h"

#include "calculator/calculator.hpp"


void print_help()
{
  printf("For ROS 2 topic subscriber, service server, action server rclcpp examples:\n");
  printf("calculator [-h]\n");
  printf("Options:\n");
  printf("\t-h Help           : Print this help function.\n");
}

// ros2 run topic_service_action_rclcpp_example calculator -h 
// 다음과 같이 인자를 전달하여 print_help 함수를 실행 시킬 수 있음. 
int main(int argc, char * argv[])
{
  // rcutils_cli_option_exist 함수는 실행 인자를 확인하고 그 값을 문자열 포인터로 반환해주는 역할을 함.
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_help();
    return 0;
  }

  rclcpp::init(argc, argv);

  auto calculator = std::make_shared<Calculator>();

  rclcpp::spin(calculator);

  rclcpp::shutdown();

  return 0;
}
