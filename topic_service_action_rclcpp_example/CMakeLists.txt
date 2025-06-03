cmake_minimum_required(VERSION 3.8)
project(topic_service_action_rclcpp_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(msg_srv_action_interface_example REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include)

# Build 
add_executable(argument src/arithmetic/argument.cpp)
ament_target_dependencies(argument
  msg_srv_action_interface_example
  rclcpp
)
## calculator src/calculator/main.cpp src/calculator/calculator.cpp
##                (Target code)               (의존성 부여를 위한 코드)
# add_executable(calculator ...)
# → calculator라는 실행 파일을 만들겠다는 의미.

# 뒤에 있는 main.cpp(엔트리포인트), calculator.cpp(기능 구현)
# → calculator 실행 파일을 만들 때 필요한 소스 코드 파일 목록.

## calculator src/calculator/main.cpp src/calculator/calculator.cpp	--> calculator라는 실행파일을 만들기 위한 구성 요소 파일들을 보여줌
## main.cpp는 실행의 시작점 (즉, main() 함수가 있는 진짜 타겟 코드)
## calculator.cpp는 노드 클래스 정의 등, 실행에 필요한 "구현 파일", 즉 의존성으로써 필요


add_executable(calculator src/calculator/main.cpp src/calculator/calculator.cpp)
ament_target_dependencies(calculator
  msg_srv_action_interface_example
  rclcpp
  rclcpp_action
)

add_executable(operator src/arithmetic/operator.cpp)
ament_target_dependencies(operator
  msg_srv_action_interface_example
  rclcpp
)

install(TARGETS
  argument
  calculator
  operator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
