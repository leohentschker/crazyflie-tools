classdef RoadmapBuilder
    
    properties(Constant)
        cf_model = CrazyflieModel()
    end
    methods
        % helper method to get the optimal trajectory for configuration
        function traj = getTrajectoryForConfiguration(obj)
            runner = CrazyflieRunner();
            display('asdasd');
        end
        
        function initial_constraint = generate_random_input(obj)
            initial_position = Point(getStateFrame(obj.cf_model.manip));
            initial_position.base_z = .5;
            display('initial config');
            display(initial_position)
        end
    end
end