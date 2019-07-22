/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef COUPLED_CONTROL_TASK_TASK_HPP
#define COUPLED_CONTROL_TASK_TASK_HPP

#include <coupled_control/TaskBase.hpp>
#include <coupled_control/coupledControl.hpp>
#include <base/samples/Joints.hpp>
#include <base/commands/Joints.hpp>
#include <base/commands/Motion2D.hpp>
#include <base-logging/Logging.hpp>
#include <vector>



namespace coupled_control{

    class Task : public TaskBase
    {
		friend class TaskBase;
		protected:
			coupled_control::coupledControl *coupledControl;

			// Property variables
			int positionCommands;
			double mMaxSpeed;
			double gain;
			int numJoints=5;
			std::vector<double> modelInitialConfig;
			std::vector<double> realInitialConfig;
			std::vector<double> jointsDirection;
            double smoothFactor;

			// Input variables
			base::commands::Motion2D motion_command;
			int current_segment;

			int sizePath;
			std::vector<int> assignment;
			std::vector<double> manipulatorConfig;

			base::samples::Joints currentConfig;

			// Output variables
			base::commands::Motion2D modified_motion_command;

			// Local variables
			std::vector<double> nextConfig;
			std::vector<float> jW;
			int saturation;
			int maxJW;
			std::vector<double> configChange;
			std::vector<double> current_config;
			base::commands::Motion2D last_motion_command;
            int firstCommand = 1;



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

