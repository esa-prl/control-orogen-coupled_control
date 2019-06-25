require 'orocos'
require 'readline'

include Orocos
Orocos.initialize

Orocos.conf.load_dir('../../../../bundles/rover/config/orogen/')

Orocos.run 'coupled_control::Task' => 'coupled_control' do

	coupled_control = Orocos.name_service.get 'coupled_control'
  	Orocos.conf.apply(coupled_control, ['exoter'], :override => true)
  	coupled_control.configure

	coupled_control.start

	motionCommand_writer = coupled_control.motionCommand.writer
	assignment_writer = coupled_control.assignment.writer
	manipulatorConfig.writer = coupled_control.manipulatorConfig.writer
	currentConfig.writer = coupled_control.currentConfig.writer

#    goal = goal_writer.new_sample
    motionCommand = Types::Base::Commands::Motion2D.new()
	assignment = Types::Std::Vector<int>.new()
	manipulatorConfig = Types::Std::Vector<double>.new()
	currentConfig = Types::Std::Vector<double>.new()

	motionCommand.translation = 1.0
	motionCommand.rotation = 1.0

	assignment.
        goal.position[0] = 85.00
        goal.position[1] = 80.00
    else
        goal.position[0] = 6.00
        goal.position[1] = 3.00
    end
    goal.position[2] = 0.00
    goal.heading = 0.00
    goal_writer.write(goal)

	Readline::readline("Press ENTER to exit\n")

end
