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

class LDS_YDLidarX2 : public LDS_YDLidarX4
{

public:
  LDS_YDLidarX2() : LDS_YDLidarX4() {}

public:
  static const std::string get_model_name() { return "YDLIDAR-X2"; }
};
