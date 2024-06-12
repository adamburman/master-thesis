% Import MPT3 library
import mpt3.*

% Define the system matrices
A = [1 0 0 0;...
        0 9.97688825315994e-120 0 0;...
        0 0.000855949360167465 0.772709770193294 0.190091811880446;...
        0 0.00437443381747106 0.0309729232429332 0.968340552949701];
B = [-3.56125356125356e-06;...
        0.000293756563663899;...
        0.000758974104725855;...
        0.00748289716506867];
C = [1 0 0 0;0 0 1 0];

% Define prediction horizon
N = 4;

% Define the system
sys = LTISystem('A', A, 'B', B, 'C', C);

% Define constraints
sys.u.min = 0;
sys.u.max = 40;
sys.x.min = [0; -0.5; 0; 0];
sys.x.max = [1; 0.5; 50; 50];

% Define cost function
sys.x.penalty = QuadFunction(diag([100 0 100 0]));
sys.u.penalty = QuadFunction(1);

% Formulate the MPC problem
mpc = MPCController(sys, N);

% Compute the explicit solution
exp_mpc = mpc.toExplicit();

% Export the explicit MPC controller to a MATLAB function
% exp_mpc.toMatlab('explicit_mpc_controller');

% Simulation parameters
T = 100; % Simulation time
x0 = [1; 0; 20; 20]; % Initial state

% Initialize state and input arrays
x = zeros(4, T+1);
u = zeros(1, T);
x(:,1) = x0;

% Run the simulation
for t = 1:T
    % Obtain the control input from the explicit MPC controller
    % u(t) = explicit_mpc_controller(x(:,t));
    exp_mpc_sim_results = exp_mpc.simulate(x(:,t),1);
    u(t) = exp_mpc_sim_results.U;    
    % Evolve the state using the system dynamics
    x(:,t+1) = A*x(:,t) + B*u(t);
end

% Plot the results
figure(3);clf;
subplot(2,1,1);
plot(0:T, x(1,:), 'b-', 0:T, x(2,:), 'r-');
xlabel('Time');
ylabel('States');
legend('x1', 'x2');
title('State Trajectories');

subplot(2,1,2);
stairs(0:T-1, u, 'k-', 'LineWidth', 1.5);
xlabel('Time');
ylabel('Control Input');
title('Control Input Trajectory');
