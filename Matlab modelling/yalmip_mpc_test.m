yalmip('clear')

% Model data
A = [1, 0, 0, 0; 
     0, 0.999976355162077, 0, 0; 
     0, 0.123485441953946, 0.772709770193292, 0.190091811880446; 
     0, 1.21727203076061, 0.0309729232429332, 0.968340552949701];
B = [-3.561e-06; 0; 0.0007592; 0.007484]; % Previous B, without I/C1 term
% B = [-3.56125356125356e-06; 0; 0.000759225545468568;
% 0.00748418218371487];
% B = [-3.56125356125356e-06; ...
%     0.0804912667429565; ...
%     0.00415319237390651; ...
%     0.0567373574035019]; % Correct B, with I/C1 term
nx = 4; % Number of states
nu = 1; % Number of inputs

% Below are the state-space matrices from the linearization toolbox in
% simulink from the diff. eq.
A = linSys.A;
B = linSys.B;
[nx, nu] = size(B);


% MPC data
% Q = diag([1 0 0 0]);
Q = diag([100 100]);
R = 1;
N = 10;


C = [1 0 0 0;0 0 1 0];
ny = height(C);

u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
r = sdpvar(repmat(ny,1,N+1),repmat(1,1,N+1));
% r = [0;40];
% sdpvar r
% d = sdpvar(1);
pastu = sdpvar(1);

constraints = []; %[-10 <= diff([pastu u{:}]) <= 10];
objective = 0;
for k = 1:N
    objective = objective + (r{k} - C*x{k})'*Q*(r{k} - C*x{k}) + u{k}'*R*u{k};
    constraints = [constraints, x{k+1} == A*x{k} + B*u{k}];
    constraints = [constraints, 0 <= u{k} <= 40];
    if k < N
        constraints = [constraints, -1 <= (u{k+1}-u{k}) <= 1];
    end
    constraints = [constraints, 0 <= x{k+1}(3) <= 50]; % Constraint on the third state (T_s)
    %constraints = [constraints, 0 <= x{k+1}(4) <= 40]; % Constraint on the fourth state (T_c)

end
% objective = objective + x{k}'*Q*x{k};

parameters_in = {x{1},[r{:}],pastu};
solutions_out = {[u{:}], [x{:}]};

controller = optimizer(constraints, objective,sdpsettings('solver','mosek'),parameters_in,solutions_out);
x = [1;0;20;20];
figure(1);clf;
disturbance = randn(1)*.01;
oldu = 0;
hold on
xhist = x;
past_inputs = [];
i = 1;
while xhist(1,i) > 0.95
    % future_r = sin((i:i+N)/40);    
    future_r = repmat([0;40], 1, length(i:i+N));
    inputs = {x,future_r,oldu};
    [solutions,diagnostics] = controller{inputs};    
    U = solutions{1};oldu = U(1);
    X = solutions{2};
    if diagnostics == 1
        error('The problem is infeasible');
    end    
    subplot(1,3,1);stairs(i:i+length(U)-1,U,'b');title('Current');
    subplot(1,3,2);cla;stairs(i:i+N,X(1,:),'g');hold on;stairs(i:i+N,future_r(1,:),'k');title('SoC');
    stairs(1:i,xhist(1,:),'g')    
    subplot(1,3,3);cla;plot(i:i+N,X(3,:),'r');title('Surface Temperature')
    x = A*x + B*U(1);
    xhist = [xhist x];
    past_inputs(i) = oldu;
    pause(0.01)   
    % The measured disturbance actually isn't constant, it changes slowly
    % disturbance = 0.99*disturbance + 0.01*randn(1);
i = i+1;
end

figure(2)
subplot(4,1,1);stairs(past_inputs,'k','linewidth',1);legend('Current');ylabel('Current [A]')
subplot(4,1,2);plot(100*xhist(1,:),'k','linewidth',1);legend('SoC');ylabel('SoC [%]')
subplot(4,1,3);plot(xhist(3,:),'k','linewidth',1);legend('Surface Temp');ylabel('Temperature [°C]')
subplot(4,1,4);plot(xhist(4,:),'k','linewidth',1);legend('Core Temp');ylabel('Temperature [°C]');xlabel('Time [s]');