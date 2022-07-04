// Copyright 2021 Open Source Robotics Foundation, Inc.
// Copyright 2022 LAAS, CNRS.
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

#ifndef ROS2_CONTROL_BOLT__SYSTEM_INTERFACE_BOLT_HPP_
#define ROS2_CONTROL_BOLT__SYSTEM_INTERFACE_BOLT_HPP_

namespace ros2_control_bolt
{

struct PosVelEffortGains
{
  double position;
  double velocity;
  double effort;
  double Kp;
  double Kd;
};

struct GyroAccLineEulerQuater
{
  double gyro_x;
  double gyro_y;
  double gyro_z;
  double accelero_x;
  double accelero_y;
  double accelero_z;
  double line_acc_x;
  double line_acc_y;
  double line_acc_z;
  double euler_x;
  double euler_y;
  double euler_z;
  double quater_x;
  double quater_y;
  double quater_z;
  double quater_w;
};

constexpr const auto HW_IF_GAIN_KP = "gain_kp";
constexpr const auto HW_IF_GAIN_KD = "gain_kd";

std::set<std::string> bolt_list_of_cmd_inter {
  "position",
  "velocity",
  "effort",
  "gain_kp",
  "gain_kd"
};

std::set<std::string> bolt_list_of_state_inter {
  "position",
  "velocity",
  "effort",
  "gain_kp",
  "gain_kd"
};

enum control_mode_t {
  POSITION,
  VELOCITY,
  EFFORT,
  POS_VEL_EFF_GAINS,
  NO_VALID_MODE
};

}  // namespace ros2_control_bolt

#endif  // ROS2_CONTROL_BOLT__SYSTEM_BOLT_HPP_
