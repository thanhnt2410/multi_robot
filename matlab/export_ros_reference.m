function export_ros_reference(destination)
%EXPORT_ROS_REFERENCE Export the failure case for independent ROS/Python comparison.
% Run from matlab/: export_ros_reference('/tmp/patrol_matlab_reference.csv').
if nargin < 1
    destination = fullfile(tempdir, 'patrol_matlab_reference.csv');
end
c = tikm.config('failure');
out = tikm.simulate(c);
writematrix([out.t out.theta], destination);
fprintf('Reference written to %s\n', destination);
end
