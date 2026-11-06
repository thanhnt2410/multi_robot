function out = coverage(sim)
%COVERAGE Grid approximation of cumulative coverage; also test each robot.
c=sim.config;
[gx,gy]=meshgrid(linspace(-c.A,c.A,c.gridSize(1)), ...
    linspace(-c.B,c.B,c.gridSize(2)));
seen=false(numel(gx),c.N); fraction=zeros(numel(sim.t),1);
individual=zeros(numel(sim.t),c.N);
for k=1:numel(sim.t)
    for i=1:c.N
        seen(:,i)=seen(:,i) | (gx(:)-sim.x(k,i)).^2+ ...
            (gy(:)-sim.y(k,i)).^2<=c.sensorRadius^2;
    end
    individual(k,:)=mean(seen,1); fraction(k)=mean(any(seen,2));
end
out=struct('gridX',gx,'gridY',gy,'seen',reshape(any(seen,2),size(gx)), ...
    'fraction',fraction,'individualFraction',individual);
end
