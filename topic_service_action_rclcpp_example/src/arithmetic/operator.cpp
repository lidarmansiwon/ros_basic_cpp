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

    using ServiceRespenseFuture = rclcpp::Client<ArithmeticOperator>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Result %.2f", response->arithmetic_result);
        return;
    };

    auto future_result = 
      arithmetic_service_client_->async_send_request(request, response_received_callback);
}