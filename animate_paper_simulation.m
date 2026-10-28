function animate_paper_simulation(sim,stride)
%ANIMATE_PAPER_SIMULATION Play an existing ideal simulation without rerunning.
%   animate_paper_simulation(results.detection,5)
if nargin<2, stride=5; end
c=sim.config;
f=figure('Name','Kuramoto-Lissajous animation','Color','w'); hold on;
g=linspace(0,2*pi,4001); [x,y]=tikm.position(g,c); plot(x,y,':','Color',[.7 .7 .7]);
rectangle('Position',[-c.A -c.B 2*c.A 2*c.B]);
h=plot(sim.x(1,:),sim.y(1,:),'bo','MarkerFaceColor','b');
phi=linspace(0,2*pi,61); rings=gobjects(c.N,1);
for i=1:c.N, rings(i)=plot(nan,nan,'Color',[.6 .8 1]); end
axis equal; xlim([-c.A-c.sensorRadius c.A+c.sensorRadius]);
ylim([-c.B-c.sensorRadius c.B+c.sensorRadius]); xlabel('x [m]'); ylabel('y [m]');
for k=1:stride:numel(sim.t)
    if ~isgraphics(f), break; end
    set(h,'XData',sim.x(k,:),'YData',sim.y(k,:));
    for i=1:c.N
        set(rings(i),'XData',sim.x(k,i)+c.sensorRadius*cos(phi), ...
            'YData',sim.y(k,i)+c.sensorRadius*sin(phi));
    end
    title(sprintf('%s | t = %.2f s',c.name,sim.t(k))); drawnow;
end
end
