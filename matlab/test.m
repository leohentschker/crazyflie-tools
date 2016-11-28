function [xtraj, utraj] = test()
builder = RoadmapBuilder();
[xtraj, utraj] = builder.run_simulations();
return;

% initialize a runner
runner = CrazyflieRunner();

% vis = constructVisualizer(runner.cf_model.manip);
r = RigidBodyManipulator('crazyflie.urdf', struct('floating', true));
vis = r.constructVisualizer();

% generate random inputs
[pitch, roll, u0] = builder.generate_random_inputs();

runner = runner.initialize_runner(pitch, roll, u0);

% test_traj = PPTrajectory(foh([0,2],[double(runner.initial_position), double(runner.final_position)]));
utraj = ConstantTrajectory(runner.u0);

sys = cascade(utraj, runner.cf_model);

% simulate the motion of the quad
xtraj = simulate(sys, utraj.tspan, initial_pos);

% set the correct output frame of the trajectory
xtraj = setOutputFrame(xtraj, getStateFrame(obj.r));

% test_traj = test_traj.setOutputFrame(runner.cf_model.manip.getStateFrame);

vis.playback(xtraj, struct('slider', true));

return;

% simulate the configuration
u0 = [0 0 0 0 0 0 runner.cf_model.nominal_thrust * 5]';
[xtraj, utraj] = runner.simulate(.4, 0, u0);

% construct the visualizer
vis = constructVisualizer(runner.cf_model.manip);

% set the output frame of the x trajectory 
xtraj = xtraj.setOutputFrame(runner.cf_model.manip.getStateFrame);

% playback the visualization
vis.playback(xtraj);

end