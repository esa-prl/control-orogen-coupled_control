/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace coupled_control;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

	// Constant variables
	mMaxSpeed = _mMaxSpeed.get(); // Maximum manipulator's joints angular speed
	gain = _gain.get(); // Position control gain
	num_joints = _num_joints.get(); // Number of manipulator's joints

	_assignment.read(assignment);
	_manipulatorConfig.read(manipulatorConfig); // Joint position along the trajectory
	_sizePath.read(sizePath);

	nextConfig.resize(num_joints);
	jW.resize(num_joints);

	saturation = 0;

	std::cout << "Configuring coupled_control" << std::endl;
	
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
	if(_currentConfig.read(currentConfig) == RTT::NewData) // Actual joint position 
	{
		std::cout << "Coupled control: inputs received" << std::endl;
		_currentSegment.read(current_segment); // Current waypoint
		_motionCommand.read(motion_command); // Rover motion command

		// Next manipulator's joints configuration
		coupledControl->selectNextManipulatorPosition(current_segment, assignment, manipulatorConfig, nextConfig);

		// Position control
		coupledControl->manipulatorMotionControl(gain, saturation, mMaxSpeed, nextConfig, currentConfig, jW);

		if(saturation == 0)
		{	
			// Rover motion command is not modified 
			_modifiedMotionCommand.write(motion_command);
			saturation = 0;
			_manipulatorCommand.write(jW);
			std::cout << "Coupled control: no saturation" << std::endl;
		}
		else
		{
			// Maximum manipulator's joints speed
			maxJW = coupledControl->findMaxValue(jW);
			// Rover motion command is modified 
			coupledControl->modifyMotionCommand(mMaxSpeed, jW.at(maxJW), jW, motion_command, modified_motion_command);
			_modifiedMotionCommand.write(modified_motion_command);
			_manipulatorCommand.write(jW);
			saturation = 0;
			std::cout << "Coupled control: saturation" << std::endl;
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
