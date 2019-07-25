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
	positionCommands = _positionCommands.get();		// Position or velocity commands
	mMaxSpeed = _mMaxSpeed.get(); 					// Maximum manipulator's joints angular speed
	gain = _gain.get(); 							// Position control gain
	numJoints = _numJoints.get(); 					// Number of manipulator's joints
	modelInitialConfig = _modelInitialConfig.get(); // Initial configuration of the arm (IK model)
	realInitialConfig = _realInitialConfig.get();	// Initial configuration of the arm (real)		
	jointsDirection = _jointsDirection.get();	 	// Direction of each joint movements
    smoothFactor = _smoothFactor.get();             // Smooth transitions between modified motion commands
    if(smoothFactor < 0) smoothFactor = 0;          // No smoothing
    else if(smoothFactor > 0.9) smoothFactor = 0.9; // Max smoothing 90%
	negativeAngles = _negativeAngles.get();	    	// Wrapping angles between 0 and 2pi or -pi and pi

	nextConfig.resize(numJoints);
	jW.resize(numJoints);
	configChange.resize(numJoints);
    current_config.resize(numJoints);

	for(int i = 0; i < numJoints; i++)	configChange.at(i) = jointsDirection.at(i)*(modelInitialConfig.at(i) + realInitialConfig.at(i));


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

	if(_sizePath.read(sizePath) == RTT::NewData) // Rover path size
    {
        //Resize assignment and joints local vectors
        assignment.resize(sizePath);
        manipulatorConfig.resize(sizePath*numJoints);


        _assignment.read(assignment);
        _manipulatorConfig.read(manipulatorConfig); // Joint position along the trajectory
    }

	if(_motionCommand.read(motion_command) == RTT::NewData) // Actual joint position 
	{
        if(firstCommand == 1) _motionCommand.read(last_motion_command);

		_currentConfig.read(currentConfig); // Current arm configuration
		_currentSegment.read(current_segment);

		//Changing from base::samples::Joints to vector<double>
		for(int i = 0; i < numJoints; i++)
		{
			base::JointState& joint(currentConfig[i]);
		    current_config[i] = joint.position;
		}

		std::cout << "Coupled control: inputs received. Current segment:" << current_segment << std::endl;
		// Next manipulator's joints configuration
		coupledControl->selectNextManipulatorPosition(current_segment, assignment, manipulatorConfig, nextConfig, negativeAngles);
				
		// Range input angles from 0 to 2pi

		std::cout << "Current configuration: ";

		for(int i = 0; i < numJoints; i++)
		{
			std::cout << current_config.at(i) << "  ";
			current_config.at(i) = coupledControl->constrainAngle(jointsDirection.at(i)*current_config.at(i), negativeAngles);
			current_config.at(i) = coupledControl->constrainAngle(current_config.at(i)-configChange.at(i), negativeAngles);
		}
		std::cout << endl;

		// Position control
		coupledControl->manipulatorMotionControl(gain, saturation, mMaxSpeed, nextConfig, current_config, jW);

		modified_motion_command = motion_command;
		if(saturation == 1)
		{
			// Maximum manipulator's joints speed
			maxJW = coupledControl->findMaxValue(jW);

			// Rover motion command is modified 
			coupledControl->modifyMotionCommand(mMaxSpeed, abs(jW.at(maxJW)), jW, motion_command, modified_motion_command);

			saturation = 0;
            firstCommand = 0;
			std::cout << "Coupled control: saturation" << std::endl;
		}
		else
		{
			// Rover motion command is not modified 
			std::cout << "Conversion relation: " << 1 << std::endl;
			std::cout << "Coupled control: no saturation" << std::endl;
		}

        modified_motion_command.translation = modified_motion_command.translation*(1-smoothFactor)+last_motion_command.translation*smoothFactor;
        modified_motion_command.rotation = modified_motion_command.rotation*(1-smoothFactor)+last_motion_command.rotation*smoothFactor;

        if(motion_command.translation == 0) modified_motion_command.translation = 0;
        if(motion_command.rotation == 0) modified_motion_command.rotation = 0;

        last_motion_command.translation = modified_motion_command.translation; 
        last_motion_command.rotation = modified_motion_command.rotation;

		// Sending outputs
		_modifiedMotionCommand.write(modified_motion_command);

		if(positionCommands == 0) 
		{
			//Changing from vector<double> to base::commands::Joints (speeds)
            std::vector<std::string> names{"ARM_JOINT_1","ARM_JOINT_2","ARM_JOINT_3","ARM_JOINT_4","ARM_JOINT_5"};
			base::commands::Joints velocityCommand(base::commands::Joints::Speeds(jW,names));
            velocityCommand.time = base::Time::now();
			_manipulatorCommand.write(velocityCommand);
		}
		else 
		{
		    std::cout << "Manipulator configuration goal: ";
			for(int i = 0; i < numJoints; i++)
				{
					nextConfig.at(i) = coupledControl->constrainAngle(nextConfig.at(i)+configChange.at(i), negativeAngles);
					nextConfig.at(i) = coupledControl->constrainAngle(jointsDirection.at(i)*nextConfig.at(i), negativeAngles);
			        std::cout << nextConfig.at(i) << "  ";
				}
		    std::cout << endl;

			//Changing from vector<double> to base::commands::Joints (speeds)
            std::vector<std::string> names{"ARM_JOINT_1","ARM_JOINT_2","ARM_JOINT_3","ARM_JOINT_4","ARM_JOINT_5"};
			base::commands::Joints positionCommand(base::commands::Joints::Positions(nextConfig,names));
            positionCommand.time = base::Time::now();
			_manipulatorCommand.write(positionCommand);
			
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
