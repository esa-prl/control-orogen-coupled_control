/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace coupled_control;
using namespace Eigen;

Task::Task(std::string const& name) : TaskBase(name) {}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine) : TaskBase(name, engine) {}

Task::~Task() {}

bool Task::configureHook()
{
    if (!TaskBase::configureHook()) return false;

    // Constant variables
    position_commands = _position_commands.get();  // Position or velocity commands
    m_max_speed = _m_max_speed.get();              // Maximum manipulator's joints angular speed
    gain = _gain.get();                            // Position control gain
    num_joints = _num_joints.get();                // Number of manipulator's joints

    model_initial_config =
        _model_initial_config.get();  // Initial configuration of the arm (IK model)
    real_initial_config = _real_initial_config.get();  // Initial configuration of the arm (real)
    joints_direction = _joints_direction.get();        // Direction of each joint movements

    smooth_factor = _smooth_factor.get();  // Smooth transitions between modified motion commands
    if (smooth_factor < 0)
        smooth_factor = 0;        // No smoothing
    else if (smooth_factor > 0.9)
        smooth_factor = 0.9;      // Max smoothing 90%

    negative_angles = _negative_angles.get();  // Wrapping angles between 0 and 2pi or -pi and pi

    sweep_movement_file = _sweep_movement_file.get();

    next_config.resize(num_joints);
    arm_joints_speed.resize(num_joints);
    config_change.resize(num_joints);
    vector_current_config.resize(num_joints);

    for (int i = 0; i < num_joints; i++)
        config_change.at(i) =
            joints_direction.at(i) * (model_initial_config.at(i) + real_initial_config.at(i));

    saturation = 0;

    arm_sweep = readMatrixFile(sweep_movement_file);
    sweep_counter = 0;

    LOG_INFO_S << "Configured coupled_control" << std::endl;

    return true;
}
bool Task::startHook()
{
    if (!TaskBase::startHook()) return false;

    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    _trajectory_status.read(trajectory_status);

    if(trajectory_status != 2)
    {
        if (_size_path.read(size_path) == RTT::NewData)  // Rover path size
        {
            // Resize assignment and joints local vectors
            assignment.resize(size_path);
            manipulator_config.resize(size_path * num_joints);

            _assignment.read(assignment);
            _manipulator_config.read(manipulator_config);  // Joint position along the trajectory
        }

        if (_motion_command.read(motion_command) == RTT::NewData)  // Actual joint position
        {
            if (first_command == 1) _motion_command.read(last_motion_command);

            _current_config.read(current_config);  // Current arm configuration
            _current_segment.read(current_segment);

            // Changing from base::samples::Joints to vector<double>
            for (int i = 0; i < num_joints; i++)
            {
                base::JointState& joint(current_config[i]);
                vector_current_config[i] = joint.position;
            }

            LOG_INFO_S << "Coupled control: inputs received. Current segment:" << current_segment
                      << std::endl;
            // Next manipulator's joints configuration
            coupledControl->selectNextManipulatorPosition(
                current_segment, assignment, manipulator_config, next_config, negative_angles);

            // Range input angles from 0 to 2pi

            std::cout << "Current configuration: ";

            for (int i = 0; i < num_joints; i++)
            {
                std::cout << vector_current_config.at(i) << "  ";
                vector_current_config.at(i) = coupledControl->constrainAngle(
                    joints_direction.at(i) * vector_current_config.at(i), negative_angles);
                vector_current_config.at(i) = coupledControl->constrainAngle(
                    vector_current_config.at(i) - config_change.at(i), negative_angles);
            }
            std::cout << endl;

            // Position control
            coupledControl->manipulatorMotionControl(
                gain, saturation, m_max_speed, next_config, vector_current_config, arm_joints_speed);

            modified_motion_command = motion_command;
            if (saturation == 1)
            {
                // Maximum manipulator's joints speed
                max_arm_speed = coupledControl->findMaxValue(arm_joints_speed);

                // Rover motion command is modified
                coupledControl->modifyMotionCommand(m_max_speed,
                                                    abs(arm_joints_speed.at(max_arm_speed)),
                                                    arm_joints_speed,
                                                    motion_command,
                                                    modified_motion_command);

                saturation = 0;
                first_command = 0;
                LOG_INFO_S << "Coupled control: saturation" << std::endl;
            }
            else
            {
                // Rover motion command is not modified
                LOG_INFO_S << "Conversion relation: " << 1 << std::endl;
                LOG_INFO_S << "Coupled control: no saturation" << std::endl;
            }

            modified_motion_command.translation =
                modified_motion_command.translation * (1 - smooth_factor)
                + last_motion_command.translation * smooth_factor;
            modified_motion_command.rotation = modified_motion_command.rotation * (1 - smooth_factor)
                                               + last_motion_command.rotation * smooth_factor;

            if (motion_command.translation == 0) modified_motion_command.translation = 0;
            if (motion_command.rotation == 0) modified_motion_command.rotation = 0;

            last_motion_command.translation = modified_motion_command.translation;
            last_motion_command.rotation = modified_motion_command.rotation;

            // Sending outputs
            _modified_motion_command.write(modified_motion_command);

            if (position_commands == 0)
            {
                // Changing from vector<double> to base::commands::Joints (speeds)
                std::vector<std::string> names{
                    "ARM_JOINT_1", "ARM_JOINT_2", "ARM_JOINT_3", "ARM_JOINT_4", "ARM_JOINT_5"};
                base::commands::Joints velocity_command(
                    base::commands::Joints::Speeds(arm_joints_speed, names));
                velocity_command.time = base::Time::now();
                _manipulator_command.write(velocity_command);
            }
            else
            {
                std::cout << "Manipulator configuration goal: ";
                for (int i = 0; i < num_joints; i++)
                {
                    next_config.at(i) = coupledControl->constrainAngle(
                        next_config.at(i) + config_change.at(i), negative_angles);
                    next_config.at(i) = coupledControl->constrainAngle(
                        joints_direction.at(i) * next_config.at(i), negative_angles);
                    std::cout << next_config.at(i) << "  ";
                }
                std::cout << endl;

                // Changing from vector<double> to base::commands::Joints
                std::vector<std::string> names{
                    "ARM_JOINT_1", "ARM_JOINT_2", "ARM_JOINT_3", "ARM_JOINT_4", "ARM_JOINT_5"};
                base::commands::Joints position_command(
                    base::commands::Joints::Positions(next_config, names));
                position_command.time = base::Time::now();
                _manipulator_command.write(position_command);
            }

            LOG_INFO_S << "Motion command. Translation: " << modified_motion_command.translation
                      << ". Rotation: " << modified_motion_command.rotation << "." << std::endl;
        }
    }
    else
    { 
        if(_current_config.read(current_config) == RTT::NewData)  // Current arm configuration
        {

            if(sweep_counter < arm_sweep.size())
            {
                bool config_reached = true;
                // Changing from base::samples::Joints to vector<double>
                std::cout<<"Config ratios: "; 
                for (int i = 0; i < num_joints; i++)
                {
                    base::JointState& joint(current_config[i]);
                    vector_current_config[i] = joint.position;
                    double config_ratio = arm_sweep[sweep_counter][i]/vector_current_config[i];
                    std::cout<<config_ratio<<" ";
                    if((config_ratio > 1.01 || config_ratio < 0.99)&&(abs(vector_current_config[i])>0.05)) 
                        config_reached = false;
                }
                std::cout<<std::endl;
                if(config_reached) sweep_counter++;

                next_config = arm_sweep[sweep_counter];

                std::cout<<"Received: ["<<vector_current_config[0]<<" "<<vector_current_config[1]<<" "<<vector_current_config[2]<<" "<<vector_current_config[3]<<" "<<vector_current_config[4]<<"]"<<std::endl; 
                std::cout<<"Sending: ["<<next_config[0]<<" "<<next_config[1]<<" "<<next_config[2]<<" "<<next_config[3]<<" "<<next_config[4]<<"]"<<std::endl;

                // Changing from vector<double> to base::commands::Joints (speeds)
                std::vector<std::string> names{
                    "ARM_JOINT_1", "ARM_JOINT_2", "ARM_JOINT_3", "ARM_JOINT_4", "ARM_JOINT_5"};
                base::commands::Joints position_command(
                    base::commands::Joints::Positions(next_config, names));
                position_command.time = base::Time::now();
                _manipulator_command.write(position_command);
                _motion_command.read(motion_command);
                base::commands::Motion2D a;
                a = motion_command;
                a.translation = 0;
                a.rotation = 0;
                _modified_motion_command.write(a);

            }
        }
    }
}
void Task::errorHook() { TaskBase::errorHook(); }
void Task::stopHook() { TaskBase::stopHook(); }
void Task::cleanupHook() { TaskBase::cleanupHook(); }

std::vector<std::vector<double>> Task::readMatrixFile(std::string movement_file)
{
    LOG_DEBUG_S << "Reading movement " << movement_file;
    std::vector<std::vector<double>> movement_matrix;
    std::string line;
    std::ifstream e_file(movement_file.c_str(), std::ios::in);
    double n_row = 0, n_col = 0;
    std::vector<double> row;

    if (e_file.is_open())
    {
        while (std::getline(e_file, line))
        {
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ' '))
            {
                double val;
                std::stringstream numeric_value(cell);
                numeric_value >> val;
                row.push_back(val);
                n_col++;
            }
            movement_matrix.push_back(row);
            row.clear();
            n_row++;
        }
        e_file.close();

        n_col /= n_row;
        LOG_INFO_S << "COUPLED CONTROL: Movement of " << n_col << " x " << n_row << " loaded.";
    }
    else
    {
        LOG_WARN_S << "COUPLED_CONTROL: Problem opening the movement file";
        return movement_matrix;
    }
    return movement_matrix;
}
