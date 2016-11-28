function [xtraj, utraj] = test()
% initialize a runner
runner = CrazyflieRunner();

% simulate the configuration
[xtraj, utraj] = runner.simulate(.4, 0);

% construct the visualizer
vis = constructVisualizer(runner.cf_model.manip);

% set the output frame of the x trajectory 
xtraj = xtraj.setOutputFrame(runner.cf_model.manip.getStateFrame);

% playback the visualization
vis.playback(xtraj);

end