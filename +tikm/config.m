function c = config(name)
%CONFIG Paper parameters where reported; illustrative choices otherwise.
c = struct('name',name,'N',5,'A',1.8,'B',0.8,'a',3,'b',2, ...
    'omega',0.3,'p',2,'order',[1 2 5 3 4],'dt',0.01,'T',45, ...
    'feedback',true,'failureAgents',[],'failureWindow',[inf inf], ...
    'perturbTime',inf,'perturbAgent',1,'perturbAmount',0, ...
    'robotRadius',0.125,'sensorRadius',NaN,'targetCount',10000, ...
    'targetSpeed',[0.2 1.0],'gridSize',[101 81]);
switch lower(name)
    case 'detection'
        c.N=15; c.A=5; c.B=5; c.a=7; c.b=8; c.p=7;
        c.order=1:c.N; c.omega=0.05; c.T=10.5; c.robotRadius=0.02;
        % N, omega and target count match Fig. 3. Geometry and target
        % trajectories are chosen here, not digitized from Fig. 2.
    case 'failure'
        c.failureAgents=[1 5]; c.failureWindow=[9.6 19.5];
        % Geometry and ring match Sec. VII. Failure times match Fig. 6;
        % Fig. 6's complete numerical configuration is not specified.
    case 'perturbation'
        c.N=10; c.A=5; c.B=5; c.a=7; c.b=3; c.p=3;
        c.order=1:c.N; c.T=65; c.omega=0.3; c.robotRadius=0.03;
        c.perturbTime=5; c.perturbAgent=1; c.perturbAmount=0.12;
        % Geometry matches Fig. 4; perturbation magnitude is selected here.
    otherwise
        error('tikm:UnknownCase','Unknown scenario: %s',name);
end
c.sensorRadius=1.01*max([sin(pi/(c.N/gcd(c.N,c.p)))*hypot(c.A,c.B), ...
    c.B*sin(pi/(2*c.a)),c.A*sin(pi/(2*c.b))]);
end
