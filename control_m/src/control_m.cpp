

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "control_m/control_m.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "system_interface_bolt.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "controller_interface/helpers.hpp"

namespace BoltExampleController{

    PosVelTorGainsController::PosVelTorGainsController() : controller_interface::ControllerInterface(),
    rt_command_ptr_(nullptr), joints_command_subscriber_(nullptr){
        interface_name_ = ros2_control_bolt::HW_IF_GAIN_KP;
    }


    void PosVelTorGainsController::declare_parameters(){
        get_node()->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>());
        get_node()->declare_parameter<std::string>("interface_name", "");
    }


    CallbackReturn PosVelTorGainsController::read_parameters(){
        joint_names_ = get_node()->get_parameter("joints").as_string_array();

        if (joint_names_.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
            return CallbackReturn::ERROR;
        }

        if (interface_name_.empty()){
            interface_name_ = get_node()->get_parameter("interface_name").as_string();
        }

        if (interface_name_.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
            return CallbackReturn::ERROR;
        }

        for (const auto & joint : joint_names_){
            command_interface_types_.push_back(joint + "/" + interface_name_);
        }

        return CallbackReturn::SUCCESS;
    }


    controller_interface::return_type PosVelTorGainsController::init(const std::string & controller_name){
        auto ret = ControllerInterface::init(controller_name);
        if(ret != controller_interface::return_type::OK){
            return ret;
        }

        try{
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
            auto_declare<std::string>("interface_name", "");
        }
        catch (const std::exception & e){
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::return_type::ERROR;
        }

        return controller_interface::return_type::OK;
    }


    CallbackReturn PosVelTorGainsController :: on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/){
        auto ret = this->read_parameters();
        if (ret != CallbackReturn::SUCCESS)
        {
            return ret;
        }

        joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
            "~/commands", rclcpp::SystemDefaultsQoS(),
            [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

        RCLCPP_INFO(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }


    controller_interface::InterfaceConfiguration PosVelTorGainsController::command_interface_configuration() const{
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        command_interfaces_config.names = command_interface_types_;

        return command_interfaces_config;
    }


    controller_interface::InterfaceConfiguration PosVelTorGainsController::state_interface_configuration() const{
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE};
    }


    CallbackReturn PosVelTorGainsController :: on_activate( const rclcpp_lifecycle::State & /*previous_state*/){
        //  check if we have all resources defined in the "points" parameter
        //  also verify that we *only* have the resources defined in the "points" parameter
        // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
        if (
            !controller_interface::get_ordered_interfaces(
            command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
            command_interface_types_.size() != ordered_interfaces.size())
        {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
            command_interface_types_.size(), ordered_interfaces.size());
            return CallbackReturn::ERROR;
        }

        // reset command buffer if a command came through callback when controller was inactive
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

        RCLCPP_INFO(get_node()->get_logger(), "activate successful");
        return CallbackReturn::SUCCESS;
    }


    CallbackReturn PosVelTorGainsController :: on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/){
        // reset command buffer
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
        return CallbackReturn::SUCCESS;
    }
    

    controller_interface :: return_type PosVelTorGainsController::update(){
        auto joint_commands = rt_command_ptr_.readFromRT();

        // no command received yet
        if (!joint_commands || !(*joint_commands)){
            return controller_interface::return_type::OK;
        }

        if ((*joint_commands)->data.size() != command_interfaces_.size()){
            RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(), *(get_node()->get_clock()), 1000,
            "command size (%zu) does not match number of interfaces (%zu)",
            (*joint_commands)->data.size(), command_interfaces_.size());
            return controller_interface::return_type::ERROR;
        }

        for (auto index = 0ul; index < command_interfaces_.size(); ++index){
            command_interfaces_[index].set_value((*joint_commands)->data[index]);
        }

        return controller_interface::return_type::OK;
    }
   
   
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(BoltExampleController::PosVelTorGainsController, controller_interface::ControllerInterface)
