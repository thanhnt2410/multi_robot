function figures(r,opt)
%FIGURES Save interpretable plots, including transient safety limitations.
if opt.visible, vis='on'; else, vis='off'; end
d=r.detection; c=d.config;
f=figure('Name','Detection and coverage','Visible',vis,'Color','w');
tiledlayout(2,2);
nexttile; hold on; g=linspace(0,2*pi,4001); [x,y]=tikm.position(g,c);
plot(x,y,':','Color',[.6 .6 .6]);
phi=linspace(0,2*pi,81);
for i=1:c.N
    plot(d.x(1,i)+c.sensorRadius*cos(phi),d.y(1,i)+c.sensorRadius*sin(phi),'Color',[.7 .85 1]);
end
plot(d.x(1,:),d.y(1,:),'bo','MarkerFaceColor','b');
axis equal; xlim([-c.A-c.sensorRadius c.A+c.sensorRadius]);
ylim([-c.B-c.sensorRadius c.B+c.sensorRadius]); title('Initial equilibrium and sensing disks');
xlabel('x [m]'); ylabel('y [m]');
nexttile; plot(d.t,d.targets.cumulative,'LineWidth',1.5); hold on;
xline(d.bounds.detectionTime,'--r','Theoretical bound');
yline(c.targetCount,':'); grid on; xlabel('Time [s]'); ylabel('Targets detected');
title('Moving targets: cumulative detections');
nexttile; plot(d.t,100*d.coverage.fraction,'k','LineWidth',1.5); hold on;
plot(d.t,100*d.coverage.individualFraction(:,1),'--b');
grid on; ylim([0 102]); xlabel('Time [s]'); ylabel('Grid coverage [%]');
legend('Whole team','Robot 1','Location','southeast'); title('Cumulative coverage');
nexttile; imagesc(linspace(-c.A,c.A,c.gridSize(1)), ...
    linspace(-c.B,c.B,c.gridSize(2)),double(d.coverage.seen));
axis xy equal tight; clim([0 1]); colorbar; title('Grid visited by team');
finish(f,'01_detection_coverage',opt);

f=figure('Name','Temporary failure comparison','Visible',vis,'Color','w');
tiledlayout(2,2);
nexttile; phases(r.failure,'Feedback: temporary stop');
nexttile; phases(r.openLoop,'Open loop: temporary stop');
nexttile; plot(r.failure.t,r.failure.phaseError,'b','LineWidth',1.5); hold on;
plot(r.openLoop.t,r.openLoop.phaseError,'--r','LineWidth',1.5);
grid on; xlabel('Time [s]'); ylabel('Relative phase RMS [rad]'); legend('Feedback','Open loop');
nexttile; plot(r.failure.t,r.failure.minDistance,'b'); hold on;
yline(2*r.failure.config.robotRadius,'--r','Body diameter');
grid on; xlabel('Time [s]'); ylabel('Minimum pair distance [m]'); title('Transient safety is monitored, not enforced');
finish(f,'02_failure_comparison',opt);

p=r.perturbation;
f=figure('Name','Perturbation resilience','Visible',vis,'Color','w');
tiledlayout(2,2);
nexttile; phases(p,'Finite phase perturbation');
nexttile; plot(p.t,p.phaseError,'LineWidth',1.5); grid on;
xlabel('Time [s]'); ylabel('Relative phase RMS [rad]'); title('Recovery of formation');
nexttile; plot(p.t,p.maxNeighborCos); hold on; yline(0,'--r'); grid on;
xlabel('Time [s]'); ylabel('max cos(neighbor phase difference)'); title('Sufficient condition: below zero');
nexttile; plot(p.t,p.minDistance); hold on;
yline(2*p.config.robotRadius,'--r'); grid on;
xlabel('Time [s]'); ylabel('Minimum pair distance [m]');
finish(f,'03_perturbation',opt);
end
function phases(s,name)
plot(s.t,s.theta,'LineWidth',1); hold on; grid on;
if all(isfinite(s.config.failureWindow))
    xline(s.config.failureWindow(1),'--k'); xline(s.config.failureWindow(2),'--k');
end
if isfinite(s.config.perturbTime), xline(s.config.perturbTime,'--k'); end
xlabel('Time [s]'); ylabel('Unwrapped phase [rad]'); title(name);
end
function finish(f,name,opt)
if opt.saveFigures
    exportgraphics(f,fullfile(opt.outputDir,[name '.png']),'Resolution',160);
    savefig(f,fullfile(opt.outputDir,[name '.fig']));
end
if ~opt.visible, close(f); end
end
