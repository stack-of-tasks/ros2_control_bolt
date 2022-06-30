#ifndef POSITION_VELOCITY_EFFORT_GAIN_CONTROLLER_HPP_
#define POSITION_VELOCITY_EFFORT_GAIN_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>


#include "controller_interface/controller_interface.hpp"
#include "forward_command_controller/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <system_interface_bolt.hpp>
#include "controller_interface/helpers.hpp"

namespace position_velocity_effort_gain_controller{
    
    using CmdType = std_msgs::msg::Float64MultiArray;
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class PosVelTorGainsController: public controller_interface::ControllerInterface{

        public:
            std::vector<std::string> joint_names_;
            std::string interface_name_;

            std::vector<std::string> command_interface_types_;

            realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
            rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

        public:
            
            PosVelTorGainsController();
        
            ~PosVelTorGainsController() = default;
          
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
          
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        
            controller_interface::return_type init(const std::string & controller_name) ;
         
            CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) ;
          
            CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) ;
            
            CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) ;

            controller_interface::return_type update() override;

            void declare_parameters();
            CallbackReturn read_parameters() ;

    };

}

#endif 

