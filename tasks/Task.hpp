/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef COUPLED_CONTROL_TASK_TASK_HPP
#define COUPLED_CONTROL_TASK_TASK_HPP

#include <base-logging/Logging.hpp>
#include <base/commands/Joints.hpp>
#include <base/commands/Motion2D.hpp>
#include <base/samples/Joints.hpp>
#include <coupled_control/TaskBase.hpp>
#include <coupled_control/coupledControl.hpp>
#include <fstream>
#include <iostream>
#include <vector>

namespace coupled_control
{

class Task : public TaskBase
{
    friend class TaskBase;

protected:
    coupled_control::coupledControl *coupledControl;

    // Property variables.
    bool is_vector_double;
    int position_commands;
    double m_max_speed;
    int arm_num_joints;
    std::vector<double> arm_model_initial_config;
    std::vector<double> arm_real_initial_config;
    std::vector<double> arm_joints_direction;
    double smooth_factor;
    int negative_angles;
    std::string final_movement_file;
    int performing_final_movement;

    // Input variables
    int trajectory_status;
    base::commands::Motion2D motion_command;
    int current_segment;
    base::samples::RigidBodyState pose;
    base::Waypoint current_waypoint;
    // Trajectory the rover should follow, given by Kinova Planning
    std::vector<base::Waypoint> trajectory;
    int size_path;
    std::vector<int> assignment;
    // Index received from Kinova Planning to know if the performed arm movement
    // is the final one
    int kinova_final_movement_index;
    motion_planning::ArmProfile arm_profile;
    motion_planning::ArmProfile final_movement_matrix;
    base::samples::Joints current_config;
    std::vector<double> vector_current_config;

    // Output variables
    base::commands::Motion2D modified_motion_command;

    // Local variables
    std::vector<double> next_config;
    std::vector<double> arm_joints_speed;
    int saturation;
    int max_arm_speed;
    std::vector<double> config_change;
    base::commands::Motion2D last_motion_command;
    int first_command = 1;
    std::vector<std::vector<double>> arm_final_movement;
    int final_movement_counter = -1;
    bool received_arm_profile;

public:
    Task(std::string const &name = "coupled_control::Task");
    Task(std::string const &name, RTT::ExecutionEngine *engine);
    ~Task();

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
    std::vector<std::vector<double>> readMatrixFile(std::string movement_file);
};
}

#endif
