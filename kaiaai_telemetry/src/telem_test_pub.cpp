// Copyright 2023 REMAKE.AI
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "kaiaai_msgs/msg/kaiaai_telemetry.hpp"

using namespace std::chrono_literals;

class TestPublisher : public rclcpp::Node
{
public:
  TestPublisher()
  : Node("kaiaai_telemetry_test_pub"), count_(0)
  {
    publisher_ = this->create_publisher<kaiaai_msgs::msg::KaiaaiTelemetry>("telemetry", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TestPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = kaiaai_msgs::msg::KaiaaiTelemetry();
    msg.seq = count_++;

    msg.odom_pos_x = float(msg.seq);
    msg.odom_pos_y = -msg.odom_pos_x;
    msg.odom_pos_yaw = msg.odom_pos_x * 0.1;

    msg.odom_vel_x = msg.odom_pos_x * 0.5;
    msg.odom_vel_yaw = -msg.odom_vel_x;

    std::vector<float> joint_pos;
    joint_pos.resize(2);
    joint_pos[0] = float(count_)*0.1;
    joint_pos[1] = -joint_pos[0];
    msg.joint_pos = joint_pos; // Makes a copy of joint_pos

    std::vector<float> joint_vel;
    joint_vel.resize(2);
    joint_vel[0] = msg.odom_pos_x*0.01;
    joint_vel[1] = -joint_vel[0];
    msg.joint_vel = joint_vel; // Makes a copy of joint_vel

    std::vector<uint8_t> lds;
    lds.resize(count_ % 250);
    for (long unsigned int i = 0; i < lds.size(); i++)
      lds[i] = (uint8_t)i;
    msg.lds = lds; // Makes a copy of lds

    msg.stamp = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "Publishing seq: %u", msg.seq);
    publisher_->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<kaiaai_msgs::msg::KaiaaiTelemetry>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPublisher>());
  rclcpp::shutdown();
  return 0;
}
