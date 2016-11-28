classdef CrazyflieRunner

    properties(Constant)
        % optimization variables
        min_trajectory_duration = .1
        max_trajectory_duration = 30
        total_timesteps = 11
        
        % allow the quadcopter to drift by two coordinates
        final_x_offset = 0
    end
    
    properties
        cf_model = CrazyflieModel()
        initial_position
        u0
        optimizer
        final_position
    end
    
    methods
        % get the initial position of the crazyflie
        function obj = set_initial_position(obj, pitch, roll)
            % initialize with the frame of the crazyflie
            obj.initial_position = Point(getStateFrame(obj.cf_model.manip));

            % set the initial positions that we specified
            obj.initial_position.base_pitch = pitch;
            obj.initial_position.base_roll = roll;

        end
        
        % set the initial thrust of the crazyflie
        function obj = set_u0(obj, u0)
            obj.u0 = u0;
        end

        % set the desired final position for the crazyflie
        function obj = set_final_position(obj)
            obj.final_position = obj.initial_position;

            % we want the final position to be completely level
            % and raised by a unit if "1"
            obj.final_position.base_z = 1;
            obj.final_position.base_x = obj.final_x_offset;
            obj.final_position.base_pitch = 0;
            obj.final_position.base_roll = 0;

        end
        
        % gets the trajectory optimizer for the crazyflie
        function obj = set_trajectory_optimizer(obj)
            obj.optimizer = DircolTrajectoryOptimization(obj.cf_model, CrazyflieRunner.total_timesteps, [CrazyflieRunner.min_trajectory_duration CrazyflieRunner.max_trajectory_duration]);  
        end

        % set the initial position and initial thrust constraints
        function obj = set_constraints(obj)
            % construct the constraints
            initial_position_constraint = ConstantConstraint(double(obj.initial_position));
            thrust_constraint = ConstantConstraint(obj.u0);
            final_position_constraint = ConstantConstraint(double(obj.final_position));

            % add the initial constraints
            obj.optimizer = obj.optimizer.addStateConstraint(initial_position_constraint, 1);
            obj.optimizer = obj.optimizer.addInputConstraint(thrust_constraint, 1);

            % add the final constraints
            obj.optimizer = obj.optimizer.addStateConstraint(final_position_constraint, obj.total_timesteps);
            
            % add in running costs
            obj.optimizer = obj.optimizer.addRunningCost(@CrazyflieRunner.cost);
            obj.optimizer = obj.optimizer.addFinalCost(@CrazyflieRunner.finalCost);

        end

        % initialize the trajectory optimizer
        function obj = initialize_runner(obj, pitch, roll, u0)
            % set the trajectory optimizer
            obj = obj.set_trajectory_optimizer();
            
            % set the initial variables
            obj = obj.set_initial_position(pitch, roll);
            obj = obj.set_u0(u0);
            obj = obj.set_final_position();

            % set the constraints from the variables
            obj = obj.set_constraints();
        end
        
        % return an initial trajectory for the crazyflie to start out with
        function initial_trajectory = get_initial_trajectory(runner, tf0)
            initial_trajectory.x = PPTrajectory(foh([0,tf0],[double(runner.initial_position), double(runner.final_position)]));
            initial_trajectory.u = ConstantTrajectory(runner.u0);
        end

        % run the simulation for the crazyflie
        function [xtraj, utraj] = run_simulation(obj)
            tf0 = 2;

            % get our initial guess at the trajectory
            initial_trajectory = obj.get_initial_trajectory(tf0);

            % solve for the actual optimized trajectory
            [xtraj, utraj, ~, ~, ~] = obj.optimizer.solveTraj(tf0, initial_trajectory);

        end
        
        % fully simulates the process of running the quadcopter
        function [xtraj, utraj] = simulate(obj, initial_pitch, initial_roll, u0)
            % initialize the values associated with the quadcopter
            obj = obj.initialize_runner(initial_pitch, initial_roll, u0);
            
            % set the constraints of the runner
            obj = obj.set_constraints();
            
            % simulate and return the xtrajectory as well as the velocity
            % trajectory
            [xtraj, utraj] = obj.run_simulation();
        end
    end
    
    methods(Static)
        
        % cost functions for the optimizer
        function [g,dg] = cost(~,x,u)
          R = eye(7);
          g = u'*R*u;
          dg = [0,zeros(1,size(x,1)),2*u'*R];
        end

        function [h,dh] = finalCost(t,x)
          h = t;
          dh = [1,zeros(1,size(x,1))];
        end
    end
    
end
