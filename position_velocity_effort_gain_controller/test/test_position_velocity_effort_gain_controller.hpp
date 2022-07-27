// Copyright 2020 PAL Robotics S.L.
// Copyright 2022 LAAS CNRS.
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

#ifndef TEST_POSITION_VELOCITY_EFFORT_GAIN_CONTROLLER_HPP_
#define TEST_POSITION_VELOCITY_EFFORT_GAIN_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "position_velocity_effort_gain_controller/position_velocity_effort_gain_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using ros2_control_bolt::HW_IF_GAIN_KP;
using ros2_control_bolt::HW_IF_GAIN_KD;

// subclassing and friending so we can access member variables
class FriendPosVelTorGainsController
: public position_velocity_effort_gain_controller::PosVelTorGainsController
{
  FRIEND_TEST(PosVelTorGainsControllerTest, JointsParameterNotSet);
  FRIEND_TEST(PosVelTorGainsControllerTest, InterfaceParameterNotSet);
  FRIEND_TEST(PosVelTorGainsControllerTest, JointsParameterIsEmpty);
  FRIEND_TEST(PosVelTorGainsControllerTest, InterfaceParameterEmpty);
  FRIEND_TEST(PosVelTorGainsControllerTest, ConfigureParamsSuccess);

  FRIEND_TEST(PosVelTorGainsControllerTest, ActivateWithWrongJointsNamesFails);
  FRIEND_TEST(PosVelTorGainsControllerTest, ActivateWithWrongInterfaceNameFails);
  FRIEND_TEST(PosVelTorGainsControllerTest, ActivateSuccess);
  FRIEND_TEST(PosVelTorGainsControllerTest, CommandSuccessTest);
  FRIEND_TEST(PosVelTorGainsControllerTest, WrongCommandCheckTest);
  FRIEND_TEST(PosVelTorGainsControllerTest, NoCommandCheckTest);
  FRIEND_TEST(PosVelTorGainsControllerTest, CommandCallbackTest);
  FRIEND_TEST(PosVelTorGainsControllerTest, ActivateDeactivateCommandsResetSuccess);
};

class PosVelTorGainsControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpController();//bool set_params_and_activate = false);
  void SetParametersAndActivateController();

protected:
  std::unique_ptr<FriendPosVelTorGainsController> controller_;

  // dummy joint state value used for tests
  const std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3"};

  double pos_cmd_ = 1.1;
  double vel_cmd_ = 2.1;
  double eff_cmd_ = 3.1;
  double kp_cmd_ = 4.1;
  double kd_cmd_ = 5.1;

  CommandInterface joint_1_pos_cmd_{joint_names_[0], HW_IF_POSITION, &pos_cmd_};
  CommandInterface joint_2_pos_cmd_{joint_names_[1], HW_IF_POSITION, &pos_cmd_};
  CommandInterface joint_3_pos_cmd_{joint_names_[2], HW_IF_POSITION, &pos_cmd_};

  CommandInterface joint_1_vel_cmd_{joint_names_[0], HW_IF_VELOCITY, &vel_cmd_};
  CommandInterface joint_2_vel_cmd_{joint_names_[1], HW_IF_VELOCITY, &vel_cmd_};
  CommandInterface joint_3_vel_cmd_{joint_names_[2], HW_IF_VELOCITY, &vel_cmd_};

  CommandInterface joint_1_eff_cmd_{joint_names_[0], HW_IF_EFFORT, &eff_cmd_};
  CommandInterface joint_2_eff_cmd_{joint_names_[1], HW_IF_EFFORT, &eff_cmd_};
  CommandInterface joint_3_eff_cmd_{joint_names_[2], HW_IF_EFFORT, &eff_cmd_};

  CommandInterface joint_1_kp_cmd_{joint_names_[0], HW_IF_GAIN_KP, &kp_cmd_};
  CommandInterface joint_2_kp_cmd_{joint_names_[1], HW_IF_GAIN_KP, &kp_cmd_};
  CommandInterface joint_3_kp_cmd_{joint_names_[2], HW_IF_GAIN_KP, &kp_cmd_};

  CommandInterface joint_1_kd_cmd_{joint_names_[0], HW_IF_GAIN_KD, &kd_cmd_};
  CommandInterface joint_2_kd_cmd_{joint_names_[1], HW_IF_GAIN_KD, &kd_cmd_};
  CommandInterface joint_3_kd_cmd_{joint_names_[2], HW_IF_GAIN_KD, &kd_cmd_};

  // CommandInterface joint_1_pos_cmd_{joint_name_, HW_IF_POSITION, &pos_cmd_};
  // CommandInterface joint_1_vel_cmd_{joint_name_, HW_IF_VELOCITY, &vel_cmd_};
  // CommandInterface joint_1_eff_cmd_{joint_name_, HW_IF_EFFORT, &eff_cmd_};
  // CommandInterface joint_1_kp_cmd_{joint_names_, HW_IF_GAIN_KP, &kp_cmd_};
  // CommandInterface joint_1_kd_cmd_{joint_names_, HW_IF_GAIN_KD, &kd_cmd_};
};

#endif  // TEST_POSITION_VELOCITY_EFFORT_GAIN_CONTROLLER_HPP_
