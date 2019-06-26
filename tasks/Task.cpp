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

	assignment.resize(150);
	manipulatorConfig.resize(150*5);

	/*for(int i = 0; i < 150; i++) 
	{
		assignment.at(i) = i;
	}
	int c = 0;
	for(int i = 0; i < 25; i++) 
		{
			manipulatorConfig.at(c) = 0;
			c++;
			manipulatorConfig.at(c) = 0;
			c++;
			manipulatorConfig.at(c) = 0;
			c++;
			manipulatorConfig.at(c) = 0;
			c++;
			manipulatorConfig.at(c) = 0;
			c++;
		}
	for(int i = 25; i < 100; i++) 
	{
		manipulatorConfig.at(c) = 0.05*(i-25);
		c++;
		manipulatorConfig.at(c) = 0.05*(i-25);
		c++;
		manipulatorConfig.at(c) = -0.05*(i-25);
		c++;
		manipulatorConfig.at(c) = -0.05*(i-25);
		c++;
		manipulatorConfig.at(c) = 0.05*(i-25);
		c++;
	}

	for(int i = 100; i < 150; i++) 
	{
		manipulatorConfig.at(c) = 0.1*(i-25);
		c++;
		manipulatorConfig.at(c) = 0.05*(i-25);
		c++;
		manipulatorConfig.at(c) = -0.05*(i-25);
		c++;
		manipulatorConfig.at(c) = -0.05*(i-25);
		c++;
		manipulatorConfig.at(c) = 0.05*(i-25);
		c++;
	}*/

	_assignment.read(assignment);
	_manipulatorConfig.read(manipulatorConfig); // Joint position along the trajectory

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
	
	if(_motionCommand.read(motion_command) == RTT::NewData) // Actual joint position 
	{
		_currentConfig.read(currentConfig); // Current arm configuration
		_currentSegment.read(current_segment);

		std::cout << "Coupled control: inputs received. Current segment:" << current_segment << std::endl;
		// Next manipulator's joints configuration
		coupledControl->selectNextManipulatorPosition(current_segment, assignment, manipulatorConfig, nextConfig);
	
		// Range input angles from 0 to 2pi

		std::cout << "Current configuration: ";

		for(int i = 0; i < num_joints; i++)
		{
			currentConfig.at(i) = coupledControl->constrainAngle(currentConfig.at(i));
			std::cout << currentConfig.at(i) << "  ";
		}
		std::cout << endl;

		// Position control
		coupledControl->manipulatorMotionControl(gain, saturation, mMaxSpeed, nextConfig, currentConfig, jW);

		if(saturation == 0)
		{	
			// Rover motion command is not modified 
			modified_motion_command = motion_command;
			_modifiedMotionCommand.write(modified_motion_command);
			saturation = 0;
			_manipulatorCommand.write(jW);
			std::cout << "Coupled control: no saturation" << std::endl;
		}
		else
		{
			// Maximum manipulator's joints speed
			maxJW = coupledControl->findMaxValue(jW);
			// Rover motion command is modified 
			coupledControl->modifyMotionCommand(mMaxSpeed, abs(jW.at(maxJW)), jW, motion_command, modified_motion_command);
			_modifiedMotionCommand.write(modified_motion_command);
			_manipulatorCommand.write(jW);
			saturation = 0;
			std::cout << "Coupled control: saturation" << std::endl;
		}
		std::cout << "Motion command. Translation: " << modified_motion_command.translation << ". Rotation: " << modified_motion_command.rotation << "." << std::endl;
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
