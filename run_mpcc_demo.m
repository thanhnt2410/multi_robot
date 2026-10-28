function out = run_mpcc_demo(options)
%RUN_MPCC_DEMO Numerical car/virtual-robot experiment inspired by Secs VI-VII.
%   out=run_mpcc_demo(struct('T',8));
%   out=run_mpcc_demo(struct('T',30,'failureWindow',[9.6 19.5]));
% Requires Optimization Toolbox (fmincon). This is not recorded hardware data.
% Uses numerical arc-length interpolation rather than the paper's polynomial.
if nargin<1, options=struct; end
assert(exist('fmincon','file')==2,'MPCC requires Optimization Toolbox / fmincon.');
options=tikm.defaults(options,struct('T',8,'dt',0.1,'M',15, ...
    'failureWindow',[inf inf],'feedback',true,'visible',true, ...
    'outputDir',fullfile(fileparts(mfilename('fullpath')),'results')));
assert(options.T>0 && options.dt>0 && options.M>=1);
c=tikm.config('failure'); c.T=options.T; c.dt=options.dt;
c.failureWindow=options.failureWindow; c.feedback=options.feedback;
[adj,theta0,bounds]=tikm.setup(c);
par=struct('l',0.175,'c',0.94,'alpha',36.8,'beta',0.94, ...
    'q',[0.01 0.025 0.01 0.01],'dt',options.dt,'M',options.M);
g=linspace(0,2*pi,30001).';
path.gamma=g;
path.s=cumtrapz(g,hypot(-c.A*c.a*sin(c.a*g),c.B*c.b*cos(c.b*g)));
path.length=path.s(end);
physical=[2 3 4]; % paper's two stopped virtual agents are 1 and 5
theta0=mod(theta0,2*pi);
state=zeros(c.N,5);
for i=1:c.N
    z=theta0(i); [x,y]=tikm.position(z,c);
    dx=-c.A*c.a*sin(c.a*z); dy=c.B*c.b*cos(c.b*z);
    state(i,:)=[x y atan2(dy,dx) c.omega*hypot(dx,dy) interp1(g,path.s,z,'pchip')];
end
events=c.failureWindow(isfinite(c.failureWindow));
events=events(events>=0 & events<=c.T);
t=unique([0:c.dt:c.T c.T events]).';
nt=numel(t); theta=zeros(nt,c.N); theta(1,:)=theta0.';
states=zeros(nt,c.N,5); states(1,:,:)=reshape(state,[1 c.N 5]);
control=nan(nt-1,c.N,2); exitFlags=nan(nt-1,c.N); cost=exitFlags;
warm=repmat([0;0.15],1,par.M,c.N);
lb=repmat([-pi/9;0.1],par.M,1); ub=repmat([pi/9;0.25],par.M,1);
solver=optimoptions('fmincon','Algorithm','sqp','Display','off', ...
    'MaxIterations',60,'MaxFunctionEvaluations',6000,'OptimalityTolerance',1e-4);
for k=1:nt-1
    h=t(k+1)-t(k); par.dt=h;
    stopped=[]; mid=(t(k)+t(k+1))/2;
    if mid>=c.failureWindow(1) && mid<c.failureWindow(2), stopped=c.failureAgents; end
    desired=tikm.phase_rhs(theta(k,:).',c,adj,stopped);
    for i=1:c.N
        if ismember(i,physical)
            if ismember(i,stopped)
                state(i,4)=0;
            else
                objective=@(u) horizonCost(u,state(i,:).',desired(i),c,par,path);
                [u,val,flag]=fmincon(objective,reshape(warm(:,:,i),[],1), ...
                    [],[],[],[],lb,ub,[],solver);
                U=reshape(u,2,par.M);
                state(i,:)=carStep(state(i,:).',U(:,1),par).';
                warm(:,:,i)=[U(:,2:end),U(:,end)];
                control(k,i,:)=reshape(U(:,1),[1 1 2]);
                exitFlags(k,i)=flag; cost(k,i)=val;
            end
            theta(k+1,i)=phaseFromS(state(i,5),path);
        else
            theta(k+1,i)=theta(k,i)+h*desired(i);
            [state(i,1),state(i,2)]=tikm.position(theta(k+1,i),c);
        end
    end
    states(k+1,:,:)=reshape(state,[1 c.N 5]);
    if mod(k,10)==0, fprintf('MPCC: %.1f / %.1f s\n',t(k+1),c.T); end
end
xy=states(:,:,1:2); lateral=zeros(nt,c.N); minimumDistance=inf(nt,1);
% Local search around progress avoids jumping branches at intersections.
for k=1:nt
    for i=physical
        nearby=theta(k,i)+linspace(-0.15,0.15,151);
        [px,py]=tikm.position(nearby,c);
        lateral(k,i)=min(hypot(states(k,i,1)-px,states(k,i,2)-py));
    end
    for i=1:c.N
        for j=i+1:c.N
            minimumDistance(k)=min(minimumDistance(k), ...
                hypot(states(k,i,1)-states(k,j,1),states(k,i,2)-states(k,j,2)));
        end
    end
end
out=struct('t',t,'config',c,'options',options,'bounds',bounds, ...
    'theta',theta,'state',states,'xy',xy,'control',control, ...
    'solverExitFlag',exitFlags,'solverCost',cost,'physicalAgents',physical, ...
    'localPathDistance',lateral,'minDistance',minimumDistance);
if ~exist(options.outputDir,'dir'), mkdir(options.outputDir); end
save(fullfile(options.outputDir,'mpcc_results.mat'),'out');
if options.visible, vis='on'; else, vis='off'; end
f=figure('Name','Car MPCC approximation','Visible',vis,'Color','w');
tiledlayout(2,2);
nexttile; [px,py]=tikm.position(g,c); plot(px,py,':k'); hold on;
for i=physical, plot(states(:,i,1),states(:,i,2)); end
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]'); title('Physical car paths');
nexttile; plot(t,theta); grid on; xlabel('Time [s]'); ylabel('Phase [rad]');
nexttile; plot(t,lateral(:,physical)); grid on;
xlabel('Time [s]'); ylabel('Local path distance [m]');
nexttile; plot(t,minimumDistance); hold on; yline(2*c.robotRadius,'--r');
grid on; xlabel('Time [s]'); ylabel('Minimum pair distance [m]');
exportgraphics(f,fullfile(options.outputDir,'04_mpcc.png'),'Resolution',160);
if ~options.visible, close(f); end
bad=sum(exitFlags(:,physical)<=0,'all');
fprintf('MPCC: %d solver calls with exitflag <= 0. Inspect solverExitFlag.\n',bad);
end
function value=horizonCost(u,z,desired,c,p,path)
U=reshape(u,2,p.M); value=0;
for j=1:p.M
    % Stage cost evaluated before propagation; sum M stages (chosen convention).
    gamma=phaseFromS(z(5),path);
    [rx,ry]=tikm.position(gamma,c);
    tangent=[-c.A*c.a*sin(c.a*gamma); c.B*c.b*cos(c.b*gamma)];
    normT=norm(tangent); tangent=tangent/normT;
    error=z(1:2)-[rx;ry];
    lag=dot(tangent,error); lat=dot([-tangent(2);tangent(1)],error);
    value=value+(desired-z(4)/normT)^2+p.q(1)*lag^2+p.q(2)*lat^2 ...
        +p.q(3)*U(1,j)^2+p.q(4)*U(2,j)^2;
    z=carStep(z,U(:,j),p);
end
end
function gamma=phaseFromS(s,path)
cycles=floor(s/path.length);
gamma=interp1(path.s,path.gamma,mod(s,path.length),'pchip')+2*pi*cycles;
end
function z=carStep(z,u,p)
f=@(q) [q(4)*cos(q(3));q(4)*sin(q(3));q(4)*tan(u(1))/p.l; ...
    -p.c*q(4)+p.alpha*u(2)-p.beta;q(4)];
k1=f(z); k2=f(z+p.dt*k1/2); k3=f(z+p.dt*k2/2); k4=f(z+p.dt*k3);
z=z+p.dt*(k1+2*k2+2*k3+k4)/6;
end
