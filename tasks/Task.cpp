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
    is_vector_double = _is_vector_double.get();    // Sending std::vector::double or base::samples::Joints
    position_commands = _position_commands.get();  // Position or velocity commands
    m_max_speed = _m_max_speed.get();              // Maximum manipulator's joints angular speed
    gain = _gain.get();                            // Position control gain
    arm_num_joints = _arm_num_joints.get();                // Number of manipulator's joints

    arm_model_initial_config =
        _arm_model_initial_config.get();  // Initial configuration of the arm (IK model)
    arm_real_initial_config = _arm_real_initial_config.get();  // Initial configuration of the arm (real)
    arm_joints_direction = _arm_joints_direction.get();        // Direction of each joint movements

    smooth_factor = _smooth_factor.get();  // Smooth transitions between modified motion commands
    if (smooth_factor < 0)
        smooth_factor = 0;        // No smoothing
    else if (smooth_factor > 0.95)
        smooth_factor = 0.95;      // Max smoothing 90%

    negative_angles = _negative_angles.get();  // Wrapping angles between 0 and 2pi or -pi and pi

    final_movement_file = _final_movement_file.get();


    next_config.resize(arm_num_joints);
    arm_joints_speed.resize(arm_num_joints);
    config_change.resize(arm_num_joints);
    vector_current_config.resize(arm_num_joints);

    for (int i = 0; i < arm_num_joints; i++)
        config_change.at(i) =
            arm_joints_direction.at(i) * (arm_model_initial_config.at(i) + arm_real_initial_config.at(i));

    saturation = 0;

    if(final_movement_file.size() > 0)
    {
        arm_final_movement = readMatrixFile(final_movement_file);
        final_movement_counter = -1;
        performing_final_movement = true;
    }
    else
    {
        final_movement_counter = -1;
        performing_final_movement = false;
    }

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

    if(trajectory_status != 2 && final_movement_counter < 0)
    {
        if (_size_path.read(size_path) == RTT::NewData)  // Rover path size
        {
            // Resize joints local vectors
            arm_profile.position.resize(size_path);
            for(int i = 0; i < size_path; i++)
                arm_profile.position[i].resize(num_joints);

            _arm_profile.read(arm_profile);  // Joint position along the trajectory
        }

        if (_motion_command.read(motion_command) == RTT::NewData)  // Actual joint position
        {
            if (first_command == 1) _motion_command.read(last_motion_command);
    
            if(!is_vector_double)
            {
                _current_config.read(current_config);  // Current arm configuration
                // Changing from base::samples::Joints to vector<double>
                for (int i = 0; i < arm_num_joints; i++)
                {
                    base::JointState& joint(current_config[i]);
                    vector_current_config[i] = joint.position;
                }
            }
            else
                _current_config_vector_double.read(vector_current_config);  // Current arm configuration
                
            _current_segment.read(current_segment);


            LOG_INFO_S << "Coupled control: inputs received. Current segment:" << current_segment
                      << std::endl;
            // Next manipulator's joints configuration
            coupledControl->selectNextManipulatorPosition(
                current_segment, &arm_profile.position, &next_config, negative_angles);

            // Range input angles from 0 to 2pi

            for (int i = 0; i < arm_num_joints; i++)
            {
                std::cout << vector_current_config.at(i) << "  ";
                vector_current_config.at(i) = coupledControl->constrainAngle(
                    arm_joints_direction.at(i) * vector_current_config.at(i), negative_angles);
                vector_current_config.at(i) = coupledControl->constrainAngle(
                    vector_current_config.at(i) - config_change.at(i), negative_angles);
            }

            // Position control

            // Rover motion command is modified
            coupledControl->modifyMotionCommand(gain,
                                                next_config,
                                                vector_current_config,
                                                m_max_speed,
                                                arm_joints_speed,
                                                modified_motion_command);

            first_command = 0;

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
                std::vector<std::string> names;
                std::vector<float> aux_arm_joints_speed;

                for (int i = 0; i < num_joints; i++)
                {
                    names.push_back(std::string("ARM_JOINT_%i",i));
                    aux_arm_joints_speed.push_back((float)arm_joints_speed[i]);
                }

                base::commands::Joints velocity_command(
                    base::commands::Joints::Speeds(aux_arm_joints_speed, names));
                velocity_command.time = base::Time::now();
                if(is_vector_double)
                    _manipulator_command_vector_double.write(arm_joints_speed);
                else
                    _manipulator_command.write(velocity_command);    
            }
            else
            {
                for (int i = 0; i < arm_num_joints; i++)
                {
                    next_config.at(i) = coupledControl->constrainAngle(
                        next_config.at(i) + config_change.at(i), negative_angles);
                    next_config.at(i) = coupledControl->constrainAngle(
                        arm_joints_direction.at(i) * next_config.at(i), negative_angles);

                    names.push_back(std::string("ARM_JOINT_%i",i));
                }

                // Changing from vector<double> to base::commands::Joints
                base::commands::Joints position_command(
                    base::commands::Joints::Positions(next_config, names));
                position_command.time = base::Time::now();
                if(is_vector_double)
                    _manipulator_command_vector_double.write(next_config);
                else
                    _manipulator_command.write(position_command);    
            }

            LOG_INFO_S << "Motion command. Translation: " << modified_motion_command.translation
                      << ". Rotation: " << modified_motion_command.rotation << "." << std::endl;
        }
    }
    else if(performing_final_movement)
    { 
        if(final_movement_counter < 0) final_movement_counter = 0;
        if(_current_config.read(current_config) == RTT::NewData)  // Current arm configuration
        {

            if(final_movement_counter < arm_final_movement.size())
            {
                bool config_reached = true;
                // Changing from base::samples::Joints to vector<double>
                for (int i = 0; i < arm_num_joints; i++)
                {
                    base::JointState& joint(current_config[i]);
                    vector_current_config[i] = joint.position;
                    double config_ratio = arm_final_movement[final_movement_counter][i]/vector_current_config[i];
                    if((config_ratio > 1.01 || config_ratio < 0.99)&&(abs(vector_current_config[i])>0.05)) 
                        config_reached = false;
                }
                if(config_reached) final_movement_counter++;

                next_config = arm_final_movement[final_movement_counter];

                // Changing from vector<double> to base::commands::Joints
                std::vector<std::string> names;
                for(int i = 0; i < num_joints; i++)
                    names.push_back(std::string("ARM_JOINT_%i",i));
                base::commands::Joints position_command(
                    base::commands::Joints::Positions(next_config, names));
                position_command.time = base::Time::now();
                if(is_vector_double)
                    _manipulator_command_vector_double.write(next_config);
                else
                    _manipulator_command.write(position_command);    
                _motion_command.read(motion_command);
                base::commands::Motion2D stop;
                stop = motion_command;
                stop.translation = 0;
                stop.rotation = 0;
                _modified_motion_command.write(stop);

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
