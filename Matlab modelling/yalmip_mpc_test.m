yalmip('clear')

% Model data
A = [1, 0, 0, 0; 
     0, 1, 0, 0; 
     0, 0.1235, 0.7727, 0.1901; 
     0, 1.217, 0.03097, 0.9683];
B = [-3.561e-06; 0; 0.0007592; 0.007484];
nx = 4; % Number of states
nu = 1; % Number of inputs

% MPC data
Q = diag([1 0 0 0]);
R = 10;
N = 20;


ny = 1;
C = [1 0];

u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
r = sdpvar(repmat(ny,1,N+1),repmat(1,1,N+1));
d = sdpvar(1);
pastu = sdpvar(1);

constraints = [-10 <= diff([pastu u{:}]) <= 10];
objective = 0;
for k = 1:N
    objective = objective + x{k}'*Q*x{k} ;
    constraints = [constraints, x{k+1} == A*x{k} + B*u{k}];
    constraints = [constraints, 0 <= u{k} <= 40];
    constraints = [constraints, 0 <= x{k+1}(3) <= 40]; % Constraint on the third state (T_s)
    constraints = [constraints, 0 <= x{k+1}(4) <= 40]; % Constraint on the fourth state (T_c)

end
% objective = objective + x{k}'*Q*x{k};

parameters_in = {x{1},[r{:}],d,pastu};
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
while xhist(1,i) > 0.90
    future_r = 0*sin((i:i+N)/40);    
    inputs = {x,future_r,disturbance,oldu};
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
    disturbance = 0.99*disturbance + 0.01*randn(1);
i = i+1;
end

figure(2)
subplot(4,1,1);stairs(past_inputs,'k','linewidth',1);legend('Current');ylabel('Current [A]')
subplot(4,1,2);plot(100*xhist(1,:),'k','linewidth',1);legend('SoC');ylabel('SoC [%]')
subplot(4,1,3);plot(xhist(3,:),'k','linewidth',1);legend('Surface Temp');ylabel('Temperature [°C]')
subplot(4,1,4);plot(xhist(4,:),'k','linewidth',1);legend('Core Temp');ylabel('Temperature [°C]');xlabel('Time [s]');