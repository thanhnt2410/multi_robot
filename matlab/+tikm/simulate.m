function out = simulate(c)
%SIMULATE Fixed-step RK4 with exact event boundaries; unwrapped phases.
[adj,theta0,bounds]=tikm.setup(c);
events=[c.failureWindow c.perturbTime];
events=events(isfinite(events) & events>=0 & events<=c.T);
t=unique([0:c.dt:c.T c.T events]).';
theta=zeros(numel(t),c.N); theta(1,:)=theta0.';
for k=1:numel(t)-1
    z=theta(k,:).';
    if abs(t(k)-c.perturbTime)<1e-10
        z(c.perturbAgent)=z(c.perturbAgent)+c.perturbAmount;
        theta(k,:)=z.';
    end
    h=t(k+1)-t(k); mid=(t(k+1)+t(k))/2;
    stopped=[];
    if mid>=c.failureWindow(1) && mid<c.failureWindow(2), stopped=c.failureAgents; end
    fun=@(q) tikm.phase_rhs(q,c,adj,stopped);
    k1=fun(z); k2=fun(z+h*k1/2); k3=fun(z+h*k2/2); k4=fun(z+h*k3);
    theta(k+1,:)=(z+h*(k1+2*k2+2*k3+k4)/6).';
end
if abs(t(end)-c.perturbTime)<1e-10
    theta(end,c.perturbAgent)=theta(end,c.perturbAgent)+c.perturbAmount;
end
[x,y]=tikm.position(theta,c);
offset=theta-theta0.';
common=angle(mean(exp(1i*offset),2));
phaseError=sqrt(mean(angle(exp(1i*(offset-common))).^2,2));
minDistance=inf(size(t)); maxNeighborCos=-inf(size(t));
for i=1:c.N
    for j=i+1:c.N
        minDistance=min(minDistance,hypot(x(:,i)-x(:,j),y(:,i)-y(:,j)));
        if adj(i,j), maxNeighborCos=max(maxNeighborCos,cos(theta(:,j)-theta(:,i))); end
    end
end
out=struct('config',c,'t',t,'theta',theta,'theta0',theta0, ...
    'adjacency',adj,'x',x,'y',y,'bounds',bounds,'phaseError',phaseError, ...
    'minDistance',minDistance,'maxNeighborCos',maxNeighborCos);
end
