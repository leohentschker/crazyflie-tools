classdef LQRSimulator

    properties(Constant)
        % initialize Q and Qf to the same values determined by Landry
        Q = diag([300 300 300 2.5 2.5 300 .001 .001 .001 .001 .001 5])
        Qf = diag([300 300 300 2.5 2.5 300 .001 .001 .001 .001 .001 5])
        R = eye(7)
    end
    
    properties
        model = CrazyflieModel()
        r = RigidBodyManipulator('crazyflie.urdf', struct('floating', true))
    end
    
    methods
        
        % get the ideal trajectory and velocity for the object
        function [ideal_xtraj, ideal_utraj] = get_ideal_traj(obj, initial_position)
            % extract the pitch and the roll from the initial position
            pitch = initial_position(TrajectorySimulator.pitch_index);
            roll = initial_position(TrajectorySimulator.roll_index);
            
            % determine the file that most closely matches the pitch and
            % roll
            traj_file = TrajectorySimulator.get_traj_file(pitch, roll);
            
            % load the ideal_trajectory variable from the traj_file
            load(traj_file);
            
            ideal_xtraj = ideal_traj.xtraj;
            ideal_utraj = ideal_traj.utraj;
        end
        
        function system = get_controller(obj, ideal_xtraj, ideal_utraj)
            % set the correct output frames
            ideal_xtraj = ideal_xtraj.setOutputFrame(obj.model.getStateFrame);
            ideal_utraj = ideal_utraj.setOutputFrame(obj.model.getInputFrame);

            % run tvlqr to get the controller
            [controller, ~] = tvlqr(obj.model,ideal_xtraj,ideal_utraj,obj.Q,obj.R,obj.Qf);
            
            % set the output frame as the input frame of the model
            controller = controller.setOutputFrame(obj.model.getInputFrame);
            controller = controller.setInputFrame(obj.model.getOutputFrame);
            
            % construct the system based on the controller
            system = feedback(obj.model, controller);
        end
        
        function simulate_system(obj, initial_pos, system, time)
            % construct the system trajectory
            systraj = system.simulate([0 time], initial_pos);
            
            % set the output frame
            systraj = systraj.setOutputFrame(getStateFrame(obj.model.manip));

            % make the visualizer and play back the trajectory
            v = obj.model.manip.constructVisualizer();
            v.playback(systraj, struct('slider', true));
        end

        function simulate_initial_position(obj, initial_position)
            
            % determine the ideal trajectory
            [ideal_xtraj, ideal_utraj] = obj.get_ideal_traj(initial_position);
            
            % get the system for the trajectory
            system = obj.get_system(ideal_xtraj, ideal_utraj);
            
            % simulate the system for the specified amount of time
            traj_breaks = ideal_xtraj.getBreaks();
            total_time = traj_breaks(end);
            obj.simulate_system(system, initial_position, total_time);
        end
    end
end