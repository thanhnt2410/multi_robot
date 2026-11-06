function results = run_paper_simulation(options)
%RUN_PAPER_SIMULATION Reproduce the main mechanisms of Boldrer et al. (2023).
%   results = run_paper_simulation;
%   results = run_paper_simulation(struct('visible',false,'saveFigures',true));
%   Requires base MATLAB only. See README_MATLAB.md for scope and assumptions.
if nargin < 1, options = struct; end
options = tikm.defaults(options, struct('visible',true,'saveFigures',true, ...
    'outputDir',fullfile(fileparts(mfilename('fullpath')),'results'), ...
    'animate',false));
if options.saveFigures && ~exist(options.outputDir,'dir'), mkdir(options.outputDir); end
rng(42,'twister');
names = {'detection','failure','perturbation'};
results = struct;
for k = 1:numel(names)
    c = tikm.config(names{k});
    results.(names{k}) = tikm.simulate(c);
end
c = tikm.config('failure'); c.feedback = false;
results.openLoop = tikm.simulate(c);
results.detection.targets = tikm.targets(results.detection);
results.detection.coverage = tikm.coverage(results.detection);
tikm.figures(results,options);
if options.saveFigures
    save(fullfile(options.outputDir,'simulation_results.mat'),'results','options');
end
d = results.detection; f = results.failure; o = results.openLoop;
fprintf('\nKuramoto-Lissajous simulation (seed 42)\n');
fprintf('Detection bound: %.4f s; sampled detections: %d/%d\n', ...
    d.bounds.detectionTime, sum(isfinite(d.targets.firstTime)),numel(d.targets.firstTime));
fprintf('Grid coverage at final time: %.2f %%\n',100*d.coverage.fraction(end));
fprintf('Final phase error, feedback/open loop: %.5g / %.5g rad\n', ...
    f.phaseError(end),o.phaseError(end));
fprintf('Nominal safety bound r_max: %.5f m (detection scenario)\n',d.bounds.robotRadius);
fprintf('Results directory: %s\n',options.outputDir);
if options.animate, animate_paper_simulation(results.detection); end
end
