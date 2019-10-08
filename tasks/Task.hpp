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
    coupled_control::coupledControl* coupledControl;

    // Property variables
    int position_commands;
    double m_max_speed;
    double gain;
    int num_joints;
    std::vector<double> model_initial_config;
    std::vector<double> real_initial_config;
    std::vector<double> joints_direction;
    double smooth_factor;
    int negative_angles;
    std::string sweep_movement_file;

    // Input variables
    base::commands::Motion2D motion_command;
    int current_segment;
    int trajectory_status;

    int size_path;
    std::vector<int> assignment;
    std::vector<double> manipulator_config;

    base::samples::Joints current_config;

    // Output variables
    base::commands::Motion2D modified_motion_command;

    // Local variables
    std::vector<double> next_config;
    std::vector<float> arm_joints_speed;
    int saturation;
    int max_arm_speed;
    std::vector<double> config_change;
    std::vector<double> vector_current_config;
    base::commands::Motion2D last_motion_command;
    int first_command = 1;

    std::vector<std::vector<double>> arm_sweep;
    int sweep_counter;

  public:
    Task(std::string const& name = "coupled_control::Task");
    Task(std::string const& name, RTT::ExecutionEngine* engine);
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
