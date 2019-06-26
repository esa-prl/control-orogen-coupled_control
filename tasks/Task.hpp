/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef COUPLED_CONTROL_TASK_TASK_HPP
#define COUPLED_CONTROL_TASK_TASK_HPP

#include <coupled_control/TaskBase.hpp>
#include <coupled_control/coupledControl.hpp>
#include <base/samples/Joints.hpp>
#include <base/commands/Joints.hpp>
#include <base/commands/Motion2D.hpp>
#include <vector>



namespace coupled_control{

    class Task : public TaskBase
    {
		friend class TaskBase;
		protected:
			coupled_control::coupledControl *coupledControl;

			// Property variables
			double mMaxSpeed;
			double gain;
			int num_joints;
			int maxJW;
			int saturation;

			// Input variables
			base::commands::Motion2D motion_command;
			int current_segment;
			std::vector<int> assignment;
			std::vector<double> manipulatorConfig;
			std::vector<double> currentConfig;

			// Output variables
			base::commands::Motion2D modified_motion_command;
			std::vector<double> manipulator_command;

			// Local variables
			std::vector<double> nextConfig;
			std::vector<double> jW;
		


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
    };
}

#endif

