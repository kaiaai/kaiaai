// Copyright 2023 REMAKE.AI, KAIA.AI, MAKERSPET.COM
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
//
// Based on
//   Copyright 2015 - 2018 EAI TEAM http://www.eaibot.com
//   https://github.com/EAIBOT/ydlidar_arduino

#pragma once
#include "lds_ydlidar_x4.h"

class LDS_YDLidarX3PRO : public LDS_YDLidarX4
{
protected:
  uint16_t LastSampleAnglePrev;
  bool scan_completed = false;

public:
  static const std::string get_model_name() { return "YDLIDAR-X3-PRO"; }
  LDS_YDLidarX3PRO() : LDS_YDLidarX4()
  {
    LastSampleAnglePrev = 0;
    scan_completed = false;
  }

  LDS::result_t decode_data(const void * context) override
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
        currentByte = readByte(context);

        if (currentByte < 0) {
          state = 1;
          return RESULT_NOT_READY;
        }

        switch (recvPos) {
        case 0:
          if (currentByte != (PH&0xFF)) {
            continue;
          }
          break;
        case 1:
          CheckSumCal = PH;
          if (currentByte!=(PH>>8)) {
            recvPos = 0;
            continue;
          }
          break;
        case 2:
          SampleNumlAndCTCal = currentByte;
          if ((currentByte != CT_Normal) && (currentByte != CT_RingStart)) {
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

      // Check buffer overflow
      if (package_Sample_Num > PACKAGE_SAMPLE_MAX_LENGTH)
        return RESULT_INVALID_PACKET;

      // Read in the rest of the packet, i.e. samples
      if (PackagePaidBytes == recvPos) {
        recvPos = 0;
        package_sample_sum = package_Sample_Num<<1;

        while (true) {
state2:
          currentByte = readByte(context);
          if (currentByte < 0) {
            state = 2;
            return RESULT_NOT_READY;
          }
          if ((recvPos & 1) == 1) {
            Valu8Tou16 += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
            CheckSumCal ^= Valu8Tou16;
          } else {
            Valu8Tou16 = currentByte;
          }

          packageBuffer[package_recvPos+recvPos] = currentByte;
          recvPos++;
          if(package_sample_sum == recvPos){
            package_recvPos += recvPos;
            break;
          }
        }

        if (package_sample_sum != recvPos) {
          state = 0;
          return RESULT_INVALID_PACKET;
        }
      } else {
        state = 0;
        return RESULT_INVALID_PACKET;
      }
      CheckSumCal ^= SampleNumlAndCTCal;
      CheckSumCal ^= LastSampleAngleCal;

      if (CheckSumCal != CheckSum) {
        CheckSumResult = false;
      } else {
        CheckSumResult = true;
      }
    }

    if (CheckSumResult) {
      scan_completed = FirstSampleAngle <= LastSampleAnglePrev;
      LastSampleAnglePrev = FirstSampleAngle;
    }

    while(true) {

      uint8_t package_CT;
      node_info node;

      package_CT = package.package_CT;
      if (package_CT == CT_Normal) {
        node.sync_quality = Node_Default_Quality + Node_NotSync;
      } else{
        node.sync_quality = Node_Default_Quality + Node_Sync;
      }

      if (CheckSumResult == true) {
        int32_t AngleCorrectForDistance;
        node.distance_q2 = package.packageSampleDistance[package_Sample_Index];

        if (node.distance_q2/4 != 0) {
          AngleCorrectForDistance = (int32_t)((atan(((21.8*(155.3 -
            (node.distance_q2*0.25f)) )/155.3)/(node.distance_q2*0.25f)))*3666.93);
        } else {
          AngleCorrectForDistance = 0;
        }
        float sampleAngle = IntervalSampleAngle*package_Sample_Index;
        if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) < 0) {
          node.angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle +
            AngleCorrectForDistance + 23040))<<LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
        } else {
          if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040) {
            node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle +
              AngleCorrectForDistance - 23040))<<LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
          } else {
            node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle +
              AngleCorrectForDistance))<<LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
          }
        }
      } else {
        node.sync_quality = Node_Default_Quality + Node_NotSync;
        node.angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
        node.distance_q2 = 0;
        package_Sample_Index = 0;
        state = 0;
        return RESULT_CRC_ERROR;
      }

      // Dump out processed data
      float point_distance_mm = node.distance_q2*0.25f;
      float point_angle_deg = (node.angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
      uint8_t point_quality = (node.sync_quality>>LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
      //bool point_startBit = (node.sync_quality & LIDAR_RESP_MEASUREMENT_SYNCBIT);
      //point.sampleIndex = package_Sample_Index;
      //point.firstSampleAngle = FirstSampleAngle/64.0f;
      //point.intervalSampleAngle = IntervalSampleAngle/64.0f;
      //point.angleCorrectionForDistance = AngleCorrectForDistance/64.0f;

      postScanPoint(context, point_angle_deg, point_distance_mm, point_quality, scan_completed);
      scan_completed = false;

      // Dump finished?
      package_Sample_Index++;
      uint8_t nowPackageNum = package.nowPackageNum;
      if (package_Sample_Index >= nowPackageNum) {
        package_Sample_Index = 0;
        break;
      }
    }
    state = 0;

    return RESULT_OK;
  }
};
