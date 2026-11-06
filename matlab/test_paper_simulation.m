function test_paper_simulation
%TEST_PAPER_SIMULATION Numerical checks for equations, events, and recovery.
c=tikm.config('detection'); [adj,z,b]=tikm.setup(c);
v=tikm.phase_rhs(z,c,adj,[]);
assert(max(abs(v-c.omega))<1e-12,'Equilibrium RHS is incorrect.');
assert(abs(b.detectionTime-8.3775804096)<1e-8,'Detection bound mismatch.');
assert(c.sensorRadius>b.coverageRadius && c.sensorRadius>=b.detectionRadius);
s=tikm.simulate(c);
assert(max(s.phaseError)<1e-9,'Rotating equilibrium drifted.');
assert(min(s.minDistance)>2*c.robotRadius,'Nominal collision-free case collided.');
cv=tikm.coverage(s);
assert(cv.fraction(end)==1,'Team did not cover all sampled grid points.');
rng(42); tg=tikm.targets(s);
assert(all(isfinite(tg.firstTime)),'Not all sampled targets were detected.');
assert(max(tg.firstTime)<=b.detectionTime+c.dt, ...
    'Sampled detection exceeded bound: inspect sampling and geometry.');

c=tikm.config('perturbation'); s=tikm.simulate(c);
k=find(abs(s.t-c.perturbTime)<1e-10,1);
assert(s.phaseError(k)>0.01,'Perturbation was not applied.');
assert(max(s.maxNeighborCos)<0,'Sufficient resilience condition was violated.');
assert(s.phaseError(end)<1e-4,'Perturbation did not recover.');
c2=c; c2.dt=c.dt/2; s2=tikm.simulate(c2);
assert(max(abs(s.theta(end,:)-s2.theta(end,:)))<1e-6,'RK4 step refinement failed.');

c=tikm.config('failure'); s=tikm.simulate(c);
mask=s.t>=c.failureWindow(1) & s.t<=c.failureWindow(2);
assert(max(abs(diff(s.theta(mask,c.failureAgents))),[],'all')<1e-12, ...
    'Failed agents moved during the stop.');
assert(s.phaseError(end)<1e-3,'Feedback formation did not recover.');
c.feedback=false; o=tikm.simulate(c);
assert(o.phaseError(end)>0.1,'Open-loop baseline unexpectedly recovered.');
fprintf('All ideal-model numerical checks passed.\n');
end
