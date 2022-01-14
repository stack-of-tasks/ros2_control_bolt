/*Inspiré de demo_create_solo12_robot.cpp*/



/*Modifs :
   
   Vector12* --> Vector6*
   Boucles for pour les couples : i<12 --> i<6
   MANQUE LES DONNEES DE CALIBRATION
   MAX CURRENT = 12
   AUTO*12 --> AUTO*6
   POSITION OFFSETS A REDEFINIR
 
*/


#include <odri_control_interface/calibration.hpp>
#include <odri_control_interface/robot.hpp>

#include <odri_control_interface/utils.hpp>

using namespace odri_control_interface;

#include <iostream>
#include <stdexcept>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<bool, 6, 1> Vector6b;

typedef Eigen::Matrix<long, 3, 1> Vector3l;
typedef Eigen::Matrix<long, 4, 1> Vector4l;
typedef Eigen::Matrix<long, 6, 1> Vector6l;
typedef Eigen::Matrix<int, 6, 1> Vector6i;

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        throw std::runtime_error(
            "Please provide the interface name "
            "(i.e. using 'ifconfig' on linux");
    }

    nice(-20);  // Give the process a high priority.

    auto main_board_ptr_ = std::make_shared<MasterBoardInterface>(argv[1]);

    Vector6i motor_numbers;
    motor_numbers << 0, 3, 2, 1, 5, 4;
    Vector6b motor_reversed;
    motor_reversed << true, false, true, true, false, false;

    Vector6d joint_lower_limits;
    joint_lower_limits << -0.5, -1.7, -3.4, -0.5, -1.7, -3.4;     //Modif d'après lecture des capteurs (demo bolt)
    Vector6d joint_upper_limits;
    joint_upper_limits << 0.5, 1.7, +3.4, +0.5, +1.7, +3.4;       //Modif d'après lecture des capteurs (demo bolt)

    // Define the joint module.
    auto joints = std::make_shared<JointModules>(main_board_ptr_,
                                                 motor_numbers,
                                                 0.025,
                                                 9.,
                                                 12.,	  //MAX CURRENT = 12
                                                 motor_reversed,
                                                 joint_lower_limits,
                                                 joint_upper_limits,
                                                 80.,
                                                 0.5);

    // Define the IMU.
    Vector3l rotate_vector;
    Vector4l orientation_vector;
    rotate_vector << 1, 2, 3;
    orientation_vector << 1, 2, 3, 4;
    auto imu = std::make_shared<IMU>(
        main_board_ptr_, rotate_vector, orientation_vector);

    // Define controller to calibrate the joints.
    std::vector<CalibrationMethod> directions = {
        AUTO, AUTO, AUTO, AUTO, AUTO, AUTO};                           // AUTO*12 --> AUTO*6
    Vector6d position_offsets;
    position_offsets << 0.238, -0.308, 0.276, -0.115, -0.584, 0.432;   // REDEFINI !
    auto calib_ctrl = std::make_shared<JointCalibrator>(
        joints, directions, position_offsets, 5., 0.05, 2., 0.001);

    // Define the robot.
    auto robot = std::make_shared<Robot>(main_board_ptr_, joints, imu, calib_ctrl);

    // Start the robot.
    robot->Start();

    

    Vector6d torques;

    double kp = 3.;
    double kd = 0.05;
    int c = 0;
    std::chrono::time_point<std::chrono::system_clock> last =
        std::chrono::system_clock::now();
    bool is_calibrated = false;
    while (!robot->IsTimeout())
    {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now()-last)).count() > 0.001)
        {
            last = std::chrono::system_clock::now();  // last+dt would be better
            robot->ParseSensorData();

            if (robot->IsReady())
            {
                if (!is_calibrated)
                {
                    is_calibrated = calib_ctrl->Run();
                    if (is_calibrated)
                    {
                        std::cout << "Calibration is done." << std::endl;
                    }
                }
                else
                {
                    // Run the main controller.
                    auto pos = robot->joints->GetPositions();
                    auto vel = robot->joints->GetVelocities();
                    // Reverse the positions;
                    for (int i = 0; i < 6; i++)
                    {
                        torques[i] = -kp * pos[i] - kd * vel[i];
                    }
                    robot->joints->SetTorques(torques);
                }
            }

            // Checks if the robot is in error state (that is, if any component
            // returns an error). If there is an error, the commands to send
            // are changed to send the safety control.
            robot->SendCommand();

            c++;
            if (c % 1000 == 0)
            {
                std::cout << "Joints: ";
                joints->PrintVector(joints->GetPositions());
                std::cout << std::endl;
            }
        }
        else
        {
            std::this_thread::yield();
        }
    }
    std::cout << "Normal program shutdown." << std::endl;
    return 0;
}

