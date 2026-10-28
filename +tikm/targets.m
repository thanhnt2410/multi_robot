function out = targets(sim)
%TARGETS Moving point targets, perfectly reflecting rectangular walls.
% Finite time sampling can miss brief contacts; this is a numerical check,
% not a proof. Targets are detected at distance <= sensorRadius.
c=sim.config; n=c.targetCount;
initial=[(2*rand(n,1)-1)*c.A,(2*rand(n,1)-1)*c.B];
heading=2*pi*rand(n,1);
speed=c.targetSpeed(1)+diff(c.targetSpeed)*rand(n,1);
velocity=[speed.*cos(heading),speed.*sin(heading)];
firstTime=inf(n,1); count=zeros(numel(sim.t),1);
for k=1:numel(sim.t)
    px=reflect(initial(:,1)+velocity(:,1)*sim.t(k),c.A);
    py=reflect(initial(:,2)+velocity(:,2)*sim.t(k),c.B);
    active=find(~isfinite(firstTime));
    if isempty(active), count(k:end)=n; break; end
    hit=false(numel(active),1);
    for i=1:c.N
        hit=hit | (px(active)-sim.x(k,i)).^2+(py(active)-sim.y(k,i)).^2<=c.sensorRadius^2;
    end
    firstTime(active(hit))=sim.t(k);
    count(k)=sum(isfinite(firstTime));
end
out=struct('initial',initial,'velocity',velocity,'firstTime',firstTime, ...
    'cumulative',count,'newDetections',[count(1);diff(count)]);
end
function q=reflect(q,L)
u=mod(q+L,4*L);
q=L-abs(u-2*L);
end
