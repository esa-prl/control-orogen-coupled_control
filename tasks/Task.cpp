/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace coupled_control;
using namespace Eigen;

Task::Task(std::string const &name) : TaskBase(name)
{
}

Task::Task(std::string const &name, RTT::ExecutionEngine *engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    if (!TaskBase::configureHook()) return false;

    // Constant variables
    is_vector_double
        = _is_vector_double
              .get(); // Sending std::vector::double or base::samples::Joints
    position_commands
        = _position_commands.get(); // Position or velocity commands
    m_max_speed
        = _m_max_speed.get(); // Maximum manipulator's joints angular speed
    arm_num_joints = _arm_num_joints.get(); // Number of manipulator's joints
    performing_final_movement
        = _performing_final_movement.get(); // Variable to check arm final
                                            // movement input type: it's 0 if
                                            // it's from a file, 1 if it's from
                                            // a matrix given by the arm planner

    arm_model_initial_config
        = _arm_model_initial_config
              .get(); // Initial configuration of the arm (IK model)
    arm_real_initial_config
        = _arm_real_initial_config
              .get(); // Initial configuration of the arm (real)
    arm_joints_direction
        = _arm_joints_direction.get(); // Direction of each joint movements

    smooth_factor
        = _smooth_factor
              .get(); // Smooth transitions between modified motion commands
    if (smooth_factor < 0)
        smooth_factor = 0; // No smoothing
    else if (smooth_factor > 0.95)
        smooth_factor = 0.95; // Max smoothing 90%

    negative_angles
        = _negative_angles
              .get(); // Wrapping angles between 0 and 2pi or -pi and pi

    next_config.resize(arm_num_joints);
    arm_joints_speed.resize(arm_num_joints);
    config_change.resize(arm_num_joints);
    vector_current_config.resize(arm_num_joints);

    for (int i = 0; i < arm_num_joints; i++)
        config_change.at(i)
            = arm_joints_direction.at(i) * (arm_model_initial_config.at(i)
                                            + arm_real_initial_config.at(i));

    saturation = 0;

    received_arm_profile = false;

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

    // Reading from kinova planning an index to know if the performed arm
    // movement is the last one
    _kinova_final_movement_port.read(kinova_final_movement_index);

    // If rover is far from the sample
    if (kinova_final_movement_index != 1 && final_movement_counter < 0)
    {

        if (_arm_profile.read(arm_profile)
            == RTT::NewData) // Joint position along the trajectory
        {
            received_arm_profile = true;
        }

        if (_motion_command.read(motion_command)
            && received_arm_profile) // Actual joint position
        {
            if (first_command == 1) _motion_command.read(last_motion_command);

            if (!is_vector_double)
            {
                _current_config.read(
                    current_config); // Current arm configuration
                // Changing from base::samples::Joints to vector<double>
                for (int i = 0; i < arm_num_joints; i++)
                {
                    base::JointState &joint(current_config[i]);
                    vector_current_config[i] = joint.position;
                }
            }
            else
                _current_config_vector_double.read(
                    vector_current_config); // Current arm configuration
            if (!_current_segment.read(current_segment) == RTT::NoData
                && !_pose.read(pose) == RTT::NoData
                && !_current_waypoint.read(current_waypoint) == RTT::NoData)
            {

                // Reading the trajectory from the Kinova Planning
                _trajectory.read(trajectory);

                LOG_INFO_S
                    << "Coupled control: inputs received. Current segment:"
                    << current_segment << std::endl;
                // Next manipulator's joints configuration
                coupledControl->selectNextManipulatorPosition(
                    current_segment,
                    &arm_profile.position,
                    &next_config,
                    negative_angles);

                // Range input angles from 0 to 2pi
                for (int i = 0; i < arm_num_joints; i++)
                {
                    vector_current_config.at(i)
                        = coupledControl->constrainAngle(
                            arm_joints_direction.at(i)
                                * vector_current_config.at(i),
                            negative_angles);
                    vector_current_config.at(i)
                        = coupledControl->constrainAngle(
                            vector_current_config.at(i) - config_change.at(i),
                            negative_angles);
                }

                // Position control

                // Rover motion command is modified
                modified_motion_command = motion_command;

                // Next waypoint position
                std::vector<double> goal_pose
                    = {trajectory[current_segment - 1].position[0],
                       trajectory[current_segment - 1].position[1],
                       trajectory[current_segment - 1].position[2]};

                // Rover current position
                std::vector<double> current_pose
                    = {pose.position.x(), pose.position.y(), pose.getYaw()};

                coupledControl->modifyMotionCommand(next_config,
                                                    vector_current_config,
                                                    goal_pose,
                                                    current_pose,
                                                    m_max_speed,
                                                    arm_joints_speed,
                                                    modified_motion_command);

                if (isnan(arm_joints_speed[0])) throw(0);
                first_command = 0;

                modified_motion_command.translation
                    = modified_motion_command.translation * (1 - smooth_factor)
                      + last_motion_command.translation * smooth_factor;
                modified_motion_command.rotation
                    = modified_motion_command.rotation * (1 - smooth_factor)
                      + last_motion_command.rotation * smooth_factor;

                if (motion_command.translation == 0)
                    modified_motion_command.translation = 0;
                if (motion_command.rotation == 0)
                    modified_motion_command.rotation = 0;

                last_motion_command.translation
                    = modified_motion_command.translation;
                last_motion_command.rotation = modified_motion_command.rotation;

                // Sending outputs
                _modified_motion_command.write(modified_motion_command);

                std::vector<std::string> names;

                if (position_commands == 0)
                {
                    // Changing from vector<double> to base::commands::Joints
                    // (speeds)
                    std::vector<float> aux_arm_joints_speed;

                    for (int i = 0; i < arm_num_joints; i++)
                    {
                        names.push_back(std::string("ARM_JOINT_%i", i));
                        aux_arm_joints_speed.push_back(
                            (float)arm_joints_speed[i]);
                    }

                    base::commands::Joints velocity_command(
                        base::commands::Joints::Speeds(aux_arm_joints_speed,
                                                       names));
                    velocity_command.time = base::Time::now();
                    if (is_vector_double)
                        _manipulator_command_vector_double.write(
                            arm_joints_speed);
                    else
                        _manipulator_command.write(velocity_command);
                }
                else
                {
                    for (int i = 0; i < arm_num_joints; i++)
                    {
                        next_config.at(i) = coupledControl->constrainAngle(
                            next_config.at(i) + config_change.at(i),
                            negative_angles);
                        next_config.at(i) = coupledControl->constrainAngle(
                            arm_joints_direction.at(i) * next_config.at(i),
                            negative_angles);

                        names.push_back(std::string("ARM_JOINT_%i", i));
                    }

                    // Changing from vector<double> to base::commands::Joints
                    base::commands::Joints position_command(
                        base::commands::Joints::Positions(next_config, names));
                    position_command.time = base::Time::now();
                    if (is_vector_double)
                        _manipulator_command_vector_double.write(next_config);
                    else
                        _manipulator_command.write(position_command);
                }

                LOG_INFO_S << "Motion command. Translation: "
                           << modified_motion_command.translation
                           << ". Rotation: " << modified_motion_command.rotation
                           << "." << std::endl;
            }
        }
    }
    // If rover is near to the sample
    else if (performing_final_movement == 1
             || performing_final_movement == 2
                    && (kinova_final_movement_index != 0))

    {
        if (final_movement_counter < 0)
        {

            // If the arm final movement is obtained from a file
            if (performing_final_movement == 1)
            {
                final_movement_file = _final_movement_file.get();
                arm_final_movement = readMatrixFile(final_movement_file);
            }
            // If the arm final movement is obtained from a matrix given by  the
            // arm planner
            else if (performing_final_movement == 2)
            {
                if (_final_movement_matrix_port.read(final_movement_matrix)
                    == RTT::NewData) // Final movement matrix
                {
                    // Initializing arm_final_movement_matrix  LAURA
                    arm_final_movement.resize(arm_num_joints);

                    for (int i = 0; i < arm_num_joints; i++)

                        arm_final_movement[i].resize(
                            final_movement_matrix.position.size());

                    arm_final_movement = final_movement_matrix.position;
                }
            }

            final_movement_counter = 0;
        }

        if (!is_vector_double)
        {
            _current_config.read(current_config); // Current arm configuration
            // Changing from base::samples::Joints to vector<double>
            for (int i = 0; i < arm_num_joints; i++)
            {
                base::JointState &joint(current_config[i]);
                vector_current_config[i] = joint.position;
            }
        }
        else
            _current_config_vector_double.read(
                vector_current_config); // Current arm configuration

        // Range input angles from 0 to 2pi
        for (int i = 0; i < arm_num_joints; i++)
        {
            vector_current_config.at(i) = coupledControl->constrainAngle(
                arm_joints_direction.at(i) * vector_current_config.at(i),
                negative_angles);
            vector_current_config.at(i) = coupledControl->constrainAngle(
                vector_current_config.at(i) - config_change.at(i),
                negative_angles);
        }

        if (final_movement_counter < arm_final_movement.size())
        {
            bool config_reached = true;
            // Changing from base::samples::Joints to vector<double>
            for (int i = 0; i < arm_num_joints; i++)
            {
                double config_ratio
                    = arm_final_movement[final_movement_counter][i]
                      / vector_current_config[i];
                if ((config_ratio > 1.01 || config_ratio < 0.99)
                    && (abs(vector_current_config[i]) > 0.05))
                    config_reached = false;
            }
            if (config_reached) final_movement_counter++;

            if (final_movement_counter < arm_final_movement.size())
                next_config = arm_final_movement[final_movement_counter];
            else
            {
                std::cout << "Ended picking movement\n";
                performing_final_movement = 3;
            }

            std::vector<std::string> names;
            // Changing from vector<double> to base::commands::Joints
            if (position_commands == 0)
            {
                // Changing from vector<double> to base::commands::Joints
                // (speeds)
                std::vector<float> aux_arm_joints_speed;
                if (performing_final_movement == 3)
                {
                    for (int i = 0; i < arm_num_joints; i++)
                        arm_joints_speed[i] = 0;
                }
                else
                {
                    coupledControl->getArmSpeed(m_max_speed,
                                                next_config,
                                                vector_current_config,
                                                arm_joints_speed);
                }

                for (int i = 0; i < arm_num_joints; i++)
                {
                    names.push_back(std::string("ARM_JOINT_%i", i));
                    aux_arm_joints_speed.push_back((float)arm_joints_speed[i]);
                }

                base::commands::Joints velocity_command(
                    base::commands::Joints::Speeds(aux_arm_joints_speed,
                                                   names));
                velocity_command.time = base::Time::now();
                if (is_vector_double)
                    _manipulator_command_vector_double.write(arm_joints_speed);
                else
                    _manipulator_command.write(velocity_command);
            }
            else
            {
                for (int i = 0; i < arm_num_joints; i++)
                {
                    next_config.at(i) = coupledControl->constrainAngle(
                        next_config.at(i) + config_change.at(i),
                        negative_angles);
                    next_config.at(i) = coupledControl->constrainAngle(
                        arm_joints_direction.at(i) * next_config.at(i),
                        negative_angles);

                    names.push_back(std::string("ARM_JOINT_%i", i));
                }

                // Changing from vector<double> to base::commands::Joints
                base::commands::Joints position_command(
                    base::commands::Joints::Positions(next_config, names));
                position_command.time = base::Time::now();
                if (is_vector_double)
                    _manipulator_command_vector_double.write(next_config);
                else
                    _manipulator_command.write(position_command);
            }

            _motion_command.read(motion_command);
            base::commands::Motion2D stop;
            stop = motion_command;
            stop.translation = 0;
            stop.rotation = 0;
            _modified_motion_command.write(stop);
        }
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

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
        LOG_INFO_S << "COUPLED CONTROL: Movement of " << n_col << " x " << n_row
                   << " loaded.";
    }
    else
    {
        LOG_WARN_S << "COUPLED_CONTROL: Problem opening the movement file";
        return movement_matrix;
    }
    return movement_matrix;
}
