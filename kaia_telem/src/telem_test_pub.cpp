#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "kaia_msgs/msg/kaia_telemetry.hpp"

using namespace std::chrono_literals;

class TestPublisher : public rclcpp::Node
{
public:
  TestPublisher()
  : Node("kaia_telemetry_test_pub"), count_(0)
  {
    publisher_ = this->create_publisher<kaia_msgs::msg::KaiaTelemetry>("telemetry", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TestPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = kaia_msgs::msg::KaiaTelemetry();
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
  rclcpp::Publisher<kaia_msgs::msg::KaiaTelemetry>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPublisher>());
  rclcpp::shutdown();
  return 0;
}
