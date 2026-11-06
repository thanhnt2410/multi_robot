function [adj,theta0,bounds] = setup(c)
%SETUP Undirected ring, rotating equilibrium, and paper's Eqs. (6)-(8).
assert(c.N>2 && c.a+c.b==c.N,'Require N>2 and a+b=N.');
assert(mod(c.a,2)==1 && gcd(c.a,c.b)==1, ...
    'Use odd a and coprime a,b for the nondegenerate curve.');
assert(isequal(sort(c.order),1:c.N),'order must permute 1:N.');
assert(c.p>c.N/4 && c.p<3*c.N/4,'p is outside the stable interval.');
adj=zeros(c.N);
theta0=zeros(c.N,1);
for k=1:c.N
    i=c.order(k); j=c.order(mod(k,c.N)+1);
    adj(i,j)=1; adj(j,i)=1;
    theta0(i)=2*pi*c.p*(k-1)/c.N;
end
bounds.kappa=gcd(c.N,c.p);
bounds.phasePositions=c.N/bounds.kappa;
bounds.robotRadius=sin(pi/c.N)*c.A*c.B/hypot(c.A*c.a,c.B*c.b);
bounds.coverageRadius=max(c.B*sin(pi/(2*c.a)),c.A*sin(pi/(2*c.b)));
bounds.detectionRadius=sin(pi/bounds.phasePositions)*hypot(c.A,c.B);
bounds.detectionTime=2*pi/(c.omega*bounds.phasePositions);
end
