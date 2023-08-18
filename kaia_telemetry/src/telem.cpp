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

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "kaia_msgs/msg/kaia_telemetry.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define NODE_NAME "kaia_telemetry_node"

#define RESULT_OK      0
#define RESULT_TIMEOUT -1
#define RESULT_FAIL    -2
#define RESULT_CRC_ERROR    -3
#define RESULT_NOT_READY    -4

#define LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
#define LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT    8

#define PackageSampleBytes 2
#define PackageSampleMaxLngth 0x80
#define Node_Default_Quality (10<<2)
#define Node_Sync 1
#define Node_NotSync 2
#define PackagePaidBytes 10
#define PH 0x55AA

typedef enum {
  CT_Normal = 0,
  CT_RingStart  = 1,
  CT_Tail,
} CT;

struct node_info {
  uint8_t    sync_quality;
  uint16_t   angle_q6_checkbit;
  uint16_t   distance_q2;
} __attribute__((packed));

struct node_package {
  uint16_t  package_Head;
  uint8_t   package_CT;
  uint8_t   nowPackageNum;
  uint16_t  packageFirstSampleAngle;
  uint16_t  packageLastSampleAngle;
  uint16_t  checkSum;
  uint16_t  packageSampleDistance[PackageSampleMaxLngth];
} __attribute__((packed));

struct scanPoint {
  uint8_t quality;
  float   angle;
  float   distance;
  bool    startBit;
};


using std::placeholders::_1;

class KaiaTelemetry : public rclcpp::Node
{
public:
  KaiaTelemetry()
  : Node(NODE_NAME)
  {
    this->declare_parameter("laser_scan.buf_len", 561);
    this->declare_parameter("laser_scan.angle_min", 0.0);
    this->declare_parameter("laser_scan.angle_max", 6.283185307179586476925286766559);
    this->declare_parameter("laser_scan.range_min", 0.1);
    this->declare_parameter("laser_scan.range_max", 12.0);
    this->declare_parameter("laser_scan.topic_name_pub", "scan");
    this->declare_parameter("laser_scan.frame_id", "ldf");

    this->declare_parameter("telemetry.topic_name_sub", "telemetry");

    this->declare_parameter("tf.frame_id", "world");
    this->declare_parameter("tf.child_frame_id", "base_footprint");

    this->declare_parameter("joint_states.topic_name_pub", "joint_states");

    this->declare_parameter("odometry.frame_id", "world");
    this->declare_parameter("odometry.child_frame_id", "base_footprint");

    this->declare_parameter("odometry.topic_name_pub", "odom");

    telem_sub_ = this->create_subscription<kaia_msgs::msg::KaiaTelemetry>(
      this->get_parameter("telemetry.topic_name_sub").as_string(),
      rclcpp::SensorDataQoS(), std::bind(&KaiaTelemetry::topic_callback, this, _1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      this->get_parameter("odom.topic_name_pub").as_string(), 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      this->get_parameter("joint_states.topic_name_pub").as_string(), 10);
    laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      this->get_parameter("laser_scan.topic_name_pub").as_string(), 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // 561 default, 561pts*9 FPS=5,049; 505pts*10FPS=5,050
    ranges_.resize(this->get_parameter("laser_scan.buf_len").as_int());

    if (this->get_parameter("laser_scan.buf_len").as_int() == 561)
      RCLCPP_INFO(this->get_logger(), "561");

    clear_ranges_buffer();
    seq_last_ = 0;

    lds_data_idx_ = 0;
    lds_data_length_ = 0;

    recvPos = 0;
    package_Sample_Num = 0;
    package_recvPos = 0;
    package_sample_sum = 0;
    currentByte = 0;

    packageBuffer = (uint8_t*) &package.package_Head;

    package_Sample_Index = 0;
    IntervalSampleAngle = 0;
    IntervalSampleAngle_LastPackage = 0;
    FirstSampleAngle = 0;
    LastSampleAngle = 0;
    CheckSum = 0;
    CheckSumCal = 0;
    SampleNumlAndCTCal = 0;
    LastSampleAngleCal = 0;
    CheckSumResult = true;
    Valu8Tou16 = 0;
    state = 0;
  }

private:
  void topic_callback(const kaia_msgs::msg::KaiaTelemetry & telem_msg) // const
  {
    long int seq_diff = (long int)telem_msg.seq - (long int)seq_last_;
    seq_last_ = telem_msg.seq;
    RCLCPP_INFO(this->get_logger(), "Seq %u (%ld) len %lu", telem_msg.seq, seq_diff, telem_msg.lds.size());

    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.frame_id = this->get_parameter("odom.frame_id").as_string();
    odom_msg.child_frame_id = this->get_parameter("odom.child_frame_id").as_string();
    odom_msg.header.stamp = telem_msg.stamp;
    odom_msg.pose.pose.position.x = telem_msg.odom_pos_x;
    odom_msg.pose.pose.position.y = telem_msg.odom_pos_y;
    odom_msg.pose.pose.position.z = 0;

    //rosserial tf.h tf::createQuaternionFromYaw(double yaw)
    //odom_msg.pose.pose.orientation.x = 0;
    //odom_msg.pose.pose.orientation.y = 0;
    //odom_msg.pose.pose.orientation.z = sin(telem_msg.odom_pos_yaw * 0.5);
    //odom_msg.pose.pose.orientation.w = cos(telem_msg.odom_pos_yaw * 0.5);

    tf2::Quaternion q;
    q.setRPY(0, 0, telem_msg.odom_pos_yaw); // roll, pitch, yaw
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = telem_msg.odom_vel_x;
    odom_msg.twist.twist.angular.z = telem_msg.odom_vel_yaw;

    odom_pub_->publish(odom_msg);


    auto joint_state_msg = sensor_msgs::msg::JointState();
    std::vector<std::string> name;
    name.resize(2);
    name[0] = "wheel_left_joint";
    name[1] = "wheel_right_joint";
    joint_state_msg.name = name;  // Makes a deep copy of name

    std::vector<double> position;
    position.resize(2);
    position[0] = telem_msg.joint_pos[0];
    position[1] = telem_msg.joint_pos[1];
    joint_state_msg.position = position;

    std::vector<double> velocity;
    velocity.resize(2);
    velocity[0] = telem_msg.joint_vel[0];
    velocity[1] = telem_msg.joint_vel[1];
    joint_state_msg.velocity = velocity;
    //float64[] effort
    joint_state_msg.header.stamp = telem_msg.stamp;

    joint_state_pub_->publish(joint_state_msg);


    // https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html
    geometry_msgs::msg::TransformStamped tf_stamped_msg;
    tf_stamped_msg.header.stamp = telem_msg.stamp;
    tf_stamped_msg.header.frame_id = this->get_parameter("tf.frame_id").as_string();
    tf_stamped_msg.child_frame_id = this->get_parameter("tf.child_frame_id").as_string();

    tf_stamped_msg.transform.translation.x = odom_msg.pose.pose.position.x;
    tf_stamped_msg.transform.translation.y = odom_msg.pose.pose.position.y;
    tf_stamped_msg.transform.translation.z = odom_msg.pose.pose.position.z;

    tf_stamped_msg.transform.rotation.x = odom_msg.pose.pose.orientation.x;
    tf_stamped_msg.transform.rotation.y = odom_msg.pose.pose.orientation.y;
    tf_stamped_msg.transform.rotation.z = odom_msg.pose.pose.orientation.z;
    tf_stamped_msg.transform.rotation.w = odom_msg.pose.pose.orientation.w;

    tf_broadcaster_->sendTransform(tf_stamped_msg);

    if (telem_msg.lds.size() > 0)
      process_lds_data(telem_msg);
  }

  void process_lds_data(const kaia_msgs::msg::KaiaTelemetry & telem_msg)
  {
    lds_data_idx_ = 0;
    lds_msg_count_++;

    while (lds_data_idx_ < telem_msg.lds.size()) {
      int err = decode_lds_data(telem_msg);
      switch(err) {
        case RESULT_OK:
          break;
        case RESULT_CRC_ERROR:
//          RCLCPP_INFO(this->get_logger(), "RESULT_CRC_ERROR");
          lds_crc_error_count_++;
          break;
        case RESULT_NOT_READY:
          break;
        case RESULT_FAIL:
//          RCLCPP_INFO(this->get_logger(), "RESULT_FAIL");
          lds_result_fail_count_++;
          break;
        default:
          RCLCPP_WARN(this->get_logger(), "Unexpected decode_lds_data() result code %d", err);
      }
    }
  }

  int get_lds_byte(const kaia_msgs::msg::KaiaTelemetry & telem_msg)
  {
    if (lds_data_idx_ >= telem_msg.lds.size())
      return -1;

    lds_data_length_++;
    return telem_msg.lds[lds_data_idx_++];
  }

  void process_scan_point(const kaia_msgs::msg::KaiaTelemetry & telem_msg,
    uint8_t quality, float angle_deg, float distance_mm, bool startBit)
  {
    (void)quality; // Suppress unused parameter warning
    if (startBit)
    {
      publish_scan(telem_msg);
      clear_ranges_buffer();
      return;
    }

    scan_point_count_total_++;
    if (distance_mm == 0.0)  // Invalid measurement
      return;

    //https://github.com/YDLIDAR/ydlidar_ros2/blob/master/src/ydlidar_node.cpp
    angle_deg = angle_deg >= 360 ? angle_deg - 360 : angle_deg;
    angle_deg = angle_deg < 0 ? angle_deg + 360 : angle_deg;
    float angle_rad = DEG_TO_RAD * angle_deg;

    double laser_scan_angle_min = this->get_parameter("laser_scan.angle_min").as_double();
    double laser_scan_angle_max = this->get_parameter("laser_scan.angle_max").as_double();
    int laser_scan_buf_len = this->get_parameter("laser_scan.buf_len").as_int();
    double laser_scan_angle_increment = ((laser_scan_angle_max - laser_scan_angle_min) / (laser_scan_buf_len - 1));

    int idx = floor((angle_rad - laser_scan_angle_min) / laser_scan_angle_increment);

    if (idx >= 0 && idx < ((long int)ranges_.size()))
    {
      ranges_[idx] = distance_mm*0.001;
      scan_point_count_valid_++;
    }
  }

  void clear_ranges_buffer()
  {
    std::fill(ranges_.begin(), ranges_.end(), 0);
    scan_point_count_valid_ = 0;
    scan_point_count_total_ = 0;
    lds_result_fail_count_ = 0;
    lds_crc_error_count_ = 0;
    lds_msg_count_ = 0;
    lds_data_length_ = 0;
  }

  void publish_scan(const kaia_msgs::msg::KaiaTelemetry & telem_msg)
//  void process_scan_data(const std::vector<uint8_t> & lds)
  {
//    RCLCPP_INFO(this->get_logger(), "publish_scan() total %u valid %u fail %u crc %u msg %u len %u",
//      scan_point_count_total_, scan_point_count_valid_, lds_result_fail_count_, lds_crc_error_count_,
//      lds_msg_count_, lds_data_length_);

    auto laser_scan_msg = sensor_msgs::msg::LaserScan();

    double laser_scan_angle_min = this->get_parameter("laser_scan.angle_min").as_double();
    double laser_scan_angle_max = this->get_parameter("laser_scan.angle_max").as_double();
    int laser_scan_buf_len = this->get_parameter("laser_scan.buf_len").as_int();
    double laser_scan_angle_increment = ((laser_scan_angle_max - laser_scan_angle_min) / (laser_scan_buf_len - 1));

    laser_scan_msg.ranges = ranges_;

    laser_scan_msg.header.stamp = telem_msg.stamp;
    laser_scan_msg.header.frame_id = this->get_parameter("laser_scan.frame_id").as_string();
    laser_scan_msg.angle_min = laser_scan_angle_min;
    laser_scan_msg.angle_max = laser_scan_angle_max;
    laser_scan_msg.angle_increment = laser_scan_angle_increment;
    laser_scan_msg.range_min = this->get_parameter("laser_scan.range_min").as_double();
    laser_scan_msg.range_max = this->get_parameter("laser_scan.range_max").as_double();
    laser_scan_msg.time_increment = 0;
    //float32 scan_time
    //float32[] intensities

    laser_scan_pub_->publish(laser_scan_msg);
  }

  int decode_lds_data(const kaia_msgs::msg::KaiaTelemetry & telem_msg)
  {
    switch(state) {
      case 1:
        goto state1;
      case 2:
        goto state2;
    }

    // Read in a packet; a packet contains up to 40 samples
    // Each packet has a Start and End (absolute) angles
    if (package_Sample_Index == 0) {

      // Read in, parse the packet header: first PackagePaidBytes=10 bytes
      package_Sample_Num = 0;
      package_recvPos = 0;
      recvPos = 0;

      while (true) {
state1:  // hack
        currentByte = get_lds_byte(telem_msg);

        if (currentByte <0 ) {
          state = 1;
          return RESULT_NOT_READY;
        }

        switch (recvPos) {
        case 0:
          if(currentByte!=(PH&0xFF)){
            continue;
          }
          break;
        case 1:
          CheckSumCal = PH;
          if(currentByte!=(PH>>8)){
            recvPos = 0;
            continue;
          }
          break;
        case 2:
          SampleNumlAndCTCal = currentByte;
          if ((currentByte != CT_Normal) && (currentByte != CT_RingStart)){
            recvPos = 0;
            continue;
          }
          break;
        case 3:
          SampleNumlAndCTCal += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
          package_Sample_Num = currentByte;
          break;
        case 4:
          if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
            FirstSampleAngle = currentByte;
          } else {
            recvPos = 0;
            continue;
          }
          break;
        case 5:
          FirstSampleAngle += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
          CheckSumCal ^= FirstSampleAngle;
          FirstSampleAngle = FirstSampleAngle>>1;
          break;
        case 6:
          if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
            LastSampleAngle = currentByte;
          } else {
            recvPos = 0;
            continue;
          }
          break;
        case 7:
          LastSampleAngle += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
          LastSampleAngleCal = LastSampleAngle;
          LastSampleAngle = LastSampleAngle>>1;
          if(package_Sample_Num == 1){
            IntervalSampleAngle = 0;
          }else{
            if(LastSampleAngle < FirstSampleAngle){
              if((FirstSampleAngle > 17280) && (LastSampleAngle < 5760)){
                IntervalSampleAngle = ((float)(23040 + LastSampleAngle - FirstSampleAngle))/(package_Sample_Num-1);
                IntervalSampleAngle_LastPackage = IntervalSampleAngle;
              } else{
                IntervalSampleAngle = IntervalSampleAngle_LastPackage;
              }
            } else{
              IntervalSampleAngle = ((float)(LastSampleAngle -FirstSampleAngle))/(package_Sample_Num-1);
              IntervalSampleAngle_LastPackage = IntervalSampleAngle;
            }
          }
          break;
        case 8:
          CheckSum = currentByte;
          break;
        case 9:
          CheckSum += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
          break;
        }
        packageBuffer[recvPos++] = currentByte;

        if (recvPos  == PackagePaidBytes ){
          package_recvPos = recvPos;
          break;

        }
      }

      // Read in the rest of the packet, i.e. samples
      if(PackagePaidBytes == recvPos){
        recvPos = 0;
        package_sample_sum = package_Sample_Num<<1;

        while (true) {
state2:
          currentByte = get_lds_byte(telem_msg);
          if (currentByte<0){
            state = 2;
            return RESULT_NOT_READY;
          }
          if((recvPos &1) == 1){
            Valu8Tou16 += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
            CheckSumCal ^= Valu8Tou16;
          }else{
            Valu8Tou16 = currentByte;
          }

          packageBuffer[package_recvPos+recvPos] =currentByte;
          recvPos++;
          if(package_sample_sum == recvPos){
            package_recvPos += recvPos;
            break;
          }
        }

        if(package_sample_sum != recvPos){
          state = 0;
          return RESULT_FAIL;
        }
      } else {
        state = 0;
        return RESULT_FAIL;
      }
      CheckSumCal ^= SampleNumlAndCTCal;
      CheckSumCal ^= LastSampleAngleCal;

      if(CheckSumCal != CheckSum){
        CheckSumResult = false;
      }else{
        CheckSumResult = true;
      }
    }

    while(true) {

      uint8_t package_CT;
      node_info node;

      package_CT = package.package_CT;
      if(package_CT == CT_Normal){
        node.sync_quality = Node_Default_Quality + Node_NotSync;
      } else{
        node.sync_quality = Node_Default_Quality + Node_Sync;
      }

      if(CheckSumResult == true){
        int32_t AngleCorrectForDistance;
        node.distance_q2 = package.packageSampleDistance[package_Sample_Index];

        if(node.distance_q2/4 != 0){
          AngleCorrectForDistance = (int32_t)((atan(((21.8*(155.3 -
            (node.distance_q2*0.25f)) )/155.3)/(node.distance_q2*0.25f)))*3666.93);
        }else{
          AngleCorrectForDistance = 0;
        }
        float sampleAngle = IntervalSampleAngle*package_Sample_Index;
        if((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) < 0){
          node.angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle +
            AngleCorrectForDistance + 23040))<<LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
        }else{
          if((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040){
            node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle +
              AngleCorrectForDistance - 23040))<<LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
          }else{
            node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle +
              AngleCorrectForDistance))<<LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
          }
        }
      }else{
        node.sync_quality = Node_Default_Quality + Node_NotSync;
        node.angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
        node.distance_q2 = 0;
        package_Sample_Index = 0;
        state = 0;
        return RESULT_CRC_ERROR;
      }

      // Dump out processed data
      scanPoint point;
      point.distance = node.distance_q2*0.25f;
      point.angle = (node.angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
      point.quality = (node.sync_quality>>LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
      point.startBit = (node.sync_quality & LIDAR_RESP_MEASUREMENT_SYNCBIT);
      //point.sampleIndex = package_Sample_Index;
      //point.firstSampleAngle = FirstSampleAngle/64.0f;
      //point.intervalSampleAngle = IntervalSampleAngle/64.0f;
      //point.angleCorrectionForDistance = AngleCorrectForDistance/64.0f;

      process_scan_point(telem_msg, point.quality, point.angle, point.distance, point.startBit);

      // Dump finished?
      package_Sample_Index++;
      uint8_t nowPackageNum = package.nowPackageNum;
      if(package_Sample_Index >= nowPackageNum){
        package_Sample_Index = 0;
        break;
      }
    }
    state = 0;

    return RESULT_OK;
  }

  rclcpp::Subscription<kaia_msgs::msg::KaiaTelemetry>::SharedPtr telem_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<float> ranges_;
  unsigned int seq_last_;
  unsigned int scan_point_count_valid_;
  unsigned int scan_point_count_total_;
  unsigned int lds_result_fail_count_;
  unsigned int lds_crc_error_count_;
  unsigned int lds_msg_count_;
  unsigned int lds_data_idx_;
  unsigned int lds_data_length_;

  int recvPos;
  uint8_t package_Sample_Num;
  int package_recvPos;
  int package_sample_sum;
  int currentByte;

  node_package package;
  uint8_t *packageBuffer;

  uint16_t package_Sample_Index;
  float IntervalSampleAngle;
  float IntervalSampleAngle_LastPackage;
  uint16_t FirstSampleAngle;
  uint16_t LastSampleAngle;
  uint16_t CheckSum;
  uint16_t CheckSumCal;
  uint16_t SampleNumlAndCTCal;
  uint16_t LastSampleAngleCal;
  bool CheckSumResult;
  uint16_t Valu8Tou16;
  uint8_t state;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KaiaTelemetry>());
  rclcpp::shutdown();
  return 0;
}
