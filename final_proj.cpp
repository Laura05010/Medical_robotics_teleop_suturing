#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <thread>
#include <chrono>
#include "ik.h"
#include "common.h"

namespace robotContext
{
    franka::Robot *robot;
    franka::Gripper *gripper;
    franka::Model *model;
}

namespace global_variable
{
    std::array<double, 16> current_ee_pose;
    bool flag_done_collecting;
    bool gripper_held;
    float step = 0.11;
    float step_size_general = 0.11; // Step size for general movement mode
    float step_size_precise = 0.01; // Step size for precise movement mode
}

double distance(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
{
    Eigen::Vector3d diff = v1 - v2;
    return diff.norm();
}

int main(int argc, char **argv)
{
    global_variable::flag_done_collecting = false;
    global_variable::gripper_held = true; // Set gripper initially held

    std::vector<std::array<double, 16>> ee_poses;
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    InverseKinematics ik_controller(1, IKType::M_P_PSEUDO_INVERSE);
    try
    {
        franka::Robot robot(argv[1]);
        robotContext::robot = &robot;
        franka::Model model = robot.loadModel();
        robotContext::model = &model;
        franka::Gripper gripper(argv[1]);
        robotContext::gripper = &gripper;
        // robot.setJointImpedance({{3000, 3000, 3000, 3000, 3000, 3000, 3000}});
        robot.setCartesianImpedance({{2500, 2500, 2500, 300, 300, 300}});

        // // set collision behavior
        // robot.setCollisionBehavior({{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        //                            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        //                            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        //                            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});

        int choice{};
        // franka::RobotState current_robot_state = robot.readOnce();
        try
        {
            std::array<double, 7> q_goal = {{-M_PI_2 - 0.1, -M_PI_4, 0, -0.5, 0, M_PI_2, M_PI_4 - 0.2}};
            MotionGenerator motion_generator(0.15, q_goal);
            std::cout << "Press Enter to place robot in needle grasping position";
            std::cin.ignore();
            robot.control(motion_generator);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::array<double, 7> q_goal1 = {{-M_PI_2 - 0.1, 2 * M_PI_4, 1.765 * M_PI_4, -2.25 * M_PI_4, 0, M_PI_2 + 0.1, M_PI_4 - 0.2}};
            MotionGenerator motion_generator_new(0.15, q_goal1);
            robot.control(motion_generator_new);
            std::cout << "Robot is ready to grasp the needle." << std::endl;
        }
        catch (franka::Exception const &e)
        {
            std::cout << e.what() << std::endl;
            return -1;
        }

        std::cout << "Ready to move: teleop" << std::endl;
        while (true)
        {
            franka::RobotState current_robot_state = robot.readOnce();
            char key;
            std::cin >> key;
            Eigen::Vector3d target_position = {0.0, 0.0, 0.0};
            switch (key)
            {
            case 'w': // move left
            {
                target_position[1] += global_variable::step;
            }
            break;
            case 's': // move right
            {
                target_position[1] -= global_variable::step;
            }
            break;
            case 'a': // move backwards, towards us
            {
                target_position[0] -= global_variable::step;
            }
            break;
            case 'd': // move forwards, away from us
            {
                target_position[0] += global_variable::step;
            }
            break;
            case 'z': // move up
            {
                target_position[2] += global_variable::step;
            }
            break;
            case 'x': // move down
            {
                target_position[2] -= global_variable::step;
            }
            break;
            case 'g': // close gripper
            {
                if (!global_variable::gripper_held)
                    break; // Skip if gripper is not held
                gripper.homing();
                gripper.grasp(0.0005, 0.1, 120);
                // Wait 1s and check afterwards, if the object is still grasped.
                std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000));
                std::cout << "Grasped object." << std::endl;
                global_variable::gripper_held = false; // Gripper is now closed
            }
            break;
            case 'h': // release gripper
            {
                if (global_variable::gripper_held)
                    break; // Skip if gripper is already held
                gripper.homing();
                gripper.move(0.08, 0.1);
                std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(3000));
                std::cout << "Released object." << std::endl;
                global_variable::gripper_held = true; // Gripper is now held
            }
            break;
            case 'o': // Go into general mode
            {
                global_variable::step = global_variable::step_size_general;
                std::cout << "Changed to GENERAL step size" << std::endl;
            }
            break;
            case 'p': // Go into precise mode
            {
                global_variable::step = global_variable::step_size_precise;
                std::cout << "Changed to PRECISE step size" << std::endl;
            }
            break;
            case 'k': // Move the robot to the area of the suture
            {
                std::array<double, 7> q_suture = {{-0.80, 2 * M_PI_4, 1.765 * M_PI_4, -2.25 * M_PI_4, 0, M_PI_2 + 0.2, M_PI_4 - 0.2}};
                MotionGenerator motion_generator(0.15, q_suture);
                std::cout << "Moving the robot to the area of the suture" << std::endl;
                std::cin.ignore();
                robot.control(motion_generator);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                std::array<double, 7> q_adjust = {{-M_PI_2 + 0.5, 2 * M_PI_4, 1.7 * M_PI_4, -3.8 * M_PI_4 + 0.5, 0, M_PI_2 + 1.6, M_PI_4 + 0.1}};
                MotionGenerator motion_generator_adjust(0.15, q_adjust);
                robot.control(motion_generator_adjust);
                std::cout << "Succesfully reached suturing area." << std::endl;
            }
            break;
            case 'm':
            { // Get current robot state
                franka::RobotState current_robot_state = robot.readOnce();

                // Increment the desired joint angle for the 7th joint by a small amount
                std::array<double, 7> q_goal = current_robot_state.q;
                q_goal[6] += 0.3; // Adjust as needed based on the rotation you desire

                // Control the robot to achieve the desired joint angles
                MotionGenerator motion_generator(0.15, q_goal);

                // Set joint impedance (optional)
                // robot.setJointImpedance({{3000, 3000, 3000, 3000, 3000, 3000, 3000}});
                // // Identical to the previous line (default franka::ControllerMode::kJointImpedance)
                // robot.control(motion_generator, franka::ControllerMode::kJointImpedance);

                // // Set Cartesian impedance (optional)
                // robot.setCartesianImpedance({{2000, 2000, 2000, 100, 100, 100}});
                // // Runs my_external_motion_generator_callback with the Cartesian impedance controller
                // robot.control(motion_generator, franka::ControllerMode::kCartesianImpedance);

                robot.control(motion_generator, franka::ControllerMode::kCartesianImpedance);

                std::cout << "7th joint rotated." << std::endl;
            }
            break;
            default:
            {
                std::cout << "Invalid input." << std::endl;
            }
            break;
            }
            if (key != 'g' && key != 'h' && key != 'k'&& key != 'm')
            {
                global_variable::current_ee_pose = current_robot_state.O_T_EE;
                global_variable::current_ee_pose[12] += target_position[0];
                global_variable::current_ee_pose[13] += target_position[1];
                global_variable::current_ee_pose[14] += target_position[2];

                Eigen::Matrix4d pose = Eigen::Matrix4d::Map(global_variable::current_ee_pose.data());

                std::cout << "Pose Matrix:\n"
                          << pose << std::endl;

                Eigen::Matrix3d rotationMatrix = pose.block<3, 3>(0, 0);
                Eigen::JacobiSVD<Eigen::Matrix3d> svd(rotationMatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
                rotationMatrix = svd.matrixU() * svd.matrixV().transpose();
                pose.block<3, 3>(0, 0) = rotationMatrix;

                std::cout << "Pose Matrix:\n"
                          << pose << std::endl;
                std::cout << "Ready for next command\n";

                double time = 0.0;
                robot.control([&time, &pose, &ik_controller](const franka::RobotState &robot_state,
                                                             franka::Duration period) -> franka::JointVelocities
                              {
                    time += period.toSec();
                    franka::JointVelocities output_velocities = ik_controller(robot_state, period, pose);
                    Eigen::Map<const Eigen::Matrix<double, 7, 1>> output_eigen_velocities(robot_state.dq.data());
                    Eigen::Vector3d current_position(robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14]);
                    Eigen::Vector3d desired_position(pose(0, 3), pose(1, 3), pose(2, 3));
                    double dist = distance(current_position, desired_position);

                    if (time >= 15.0 || (output_eigen_velocities.norm() < 0.0005 && dist < 0.0005))
                    {
                        output_velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                        return franka::MotionFinished(output_velocities);
                    }
                    return output_velocities; });
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            }
        }
    }
    catch (franka::Exception const &e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}
