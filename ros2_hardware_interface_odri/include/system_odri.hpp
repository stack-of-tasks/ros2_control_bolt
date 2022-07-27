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

#ifndef ROS2_CONTROL_ODRI__SYSTEM_ODRI_HPP_
#define ROS2_CONTROL_ODRI__SYSTEM_ODRI_HPP_

/*Connection to ODRI for read sensors and write commands*/
#include <odri_control_interface/calibration.hpp>
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

#include "system_interface_odri.hpp"


using hardware_interface::return_type;

#define rt_printf printf

/**
 * @brief Useful tool for the demos and programs in order to print data in
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


namespace ros2_control_odri
{


class SystemOdriHardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(SystemOdriHardware)

  ROS2_CONTROL_ODRI_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & system_info) override;

  ROS2_CONTROL_ODRI_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_ODRI_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_ODRI_PUBLIC
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  ROS2_CONTROL_ODRI_PUBLIC
  return_type calibration();

  ROS2_CONTROL_ODRI_PUBLIC
  return_type start() override;

  ROS2_CONTROL_ODRI_PUBLIC
  return_type stop() override;

  ROS2_CONTROL_ODRI_PUBLIC
  return_type read() override;

  ROS2_CONTROL_ODRI_PUBLIC
  return_type write() override;

  ROS2_CONTROL_ODRI_PUBLIC
  return_type display();



private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Give some information on the current robot state.
  void display_robot_state();

  // Read desired starting position.
  return_type read_desired_starting_position();

  // Read default joint cmd and state values
  return_type read_default_cmd_state_value(std::string &default_joint_cs);

  // Read default cmd or state value.
  // default_joint_cs: "default_joint_cmd" or "default_joint_state"
  // default_joint_cs:
  //Joint number from urdf
  std::map<std::string,int> joint_name_to_array_index_;

  // Store the command for the simulated robot
  std::map<std::string,PosVelEffortGains> hw_commands_;
  std::map<std::string,PosVelEffortGains> hw_states_;
  std::map<std::string,control_mode_t> control_mode_;

  // Store the imu data
  std::map<std::string,GyroAccLineEulerQuater> imu_states_;


  std::map<std::string,control_mode_t> new_modes_;

  //Definition of multiple variables about Odri
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

  // Starting desired position.
  Eigen::VectorXd eig_des_start_pos_;

};



}  // namespace ros2_control_odri

#endif  // ROS2_CONTROL_ODRI__SYSTEM_ODRI_HPP_
