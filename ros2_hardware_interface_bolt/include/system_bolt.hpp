// Copyright 2021 ros2_control Development Team
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

#ifndef ROS2_CONTROL_BOLT__SYSTEM_BOLT_HPP_
#define ROS2_CONTROL_BOLT__SYSTEM_BOLT_HPP_

/*Connection to ODRI for read sensors and write commands*/
#include "odri_control_interface/calibration.hpp"
#include "odri_control_interface/robot.hpp"
#include "odri_control_interface/imu.hpp"
#include "master_board_sdk/master_board_interface.h"



#include <memory>
#include <set>
#include <string>
#include <vector>
#include <map>
#include "rclcpp/macros.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "visibility_control.h"
#include "semantic_components/imu_sensor.hpp"



using hardware_interface::return_type;

#define rt_printf printf

/**
 * @brief Usefull tool for the demos and programs in order to print data in
 * real time.
 *
 * @param v_name  is a string defining the data to print.
 * @param v the vector to print.
 */
void print_vector(std::string v_name, const Eigen::Ref<const Eigen::VectorXd> v)
{
    v_name += ": [";
    rt_printf("%s", v_name.c_str());
    for (int i = 0; i < v.size(); ++i)
    {
        rt_printf("%0.3f, ", v(i));
    }
    rt_printf("]\n");
}


namespace Eigen
{
/** @brief Eigen shortcut for vector of size 6. */
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<bool, 6, 1> Vector6b;
typedef Matrix<long, 6, 1> Vector6l;
typedef Matrix<int, 6, 1> Vector6i;
/** @brief Eigen shortcut for vector of size 3. */
typedef Matrix<long, 3, 1> Vector3l;
/** @brief Eigen shortcut for vector of size 4. */
typedef Matrix<long, 4, 1> Vector4l;

}  // namespace Eigen


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

class SystemBoltHardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(SystemBoltHardware)

  ROS2_CONTROL_BOLT_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & system_info) override;

  ROS2_CONTROL_BOLT_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_BOLT_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_BOLT_PUBLIC
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  
  ROS2_CONTROL_BOLT_PUBLIC
  return_type calibration();
  
  ROS2_CONTROL_BOLT_PUBLIC
  return_type start() override;

  ROS2_CONTROL_BOLT_PUBLIC
  return_type stop() override;

  ROS2_CONTROL_BOLT_PUBLIC
  return_type read() override;

  ROS2_CONTROL_BOLT_PUBLIC
  return_type write() override;

  ROS2_CONTROL_BOLT_PUBLIC
  return_type display();



private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;
  
  //Joint number from urdf
  std::map<std::string,int> joint_name_to_array_index_;

  // Store the command for the simulated robot
  std::map<std::string,PosVelEffortGains> hw_commands_;
  std::map<std::string,PosVelEffortGains> hw_states_;
  std::map<std::string,control_mode_t> control_mode_;

  // Store the imu data
  std::map<std::string,GyroAccLineEulerQuater> imu_states_;


  std::map<std::string,control_mode_t> new_modes_;

  //Definition of multiple variables about Bolt
  // Joint
  Eigen::Vector6i motor_numbers_;
  Eigen::Vector6b motor_reversed_polarities_;
  Eigen::Vector6d joint_lower_limits_;
  Eigen::Vector6d joint_upper_limits_;
  Eigen::Vector6d position_offsets_;

  // IMU
  Eigen::Vector3l rotate_vector_;
  Eigen::Vector4l orientation_vector_;
  

  //Network id
  std::string eth_interface_;

  //robot 
  std::shared_ptr<odri_control_interface::Robot> robot_;
  std::shared_ptr<odri_control_interface::JointModules> joints_;
  std::shared_ptr<odri_control_interface::JointCalibrator> calib_;
  std::shared_ptr<MasterBoardInterface> main_board_ptr_;
  

  double motor_constants_;
  double gear_ratios_;
  double max_currents_;
  double max_joint_velocities_;
  double safety_damping_;

};



}  // namespace ros2_control_bolt

#endif  // ROS2_CONTROL_BOLT__SYSTEM_BOLT_HPP_
