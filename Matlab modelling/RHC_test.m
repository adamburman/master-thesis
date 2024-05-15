% System matrices
A = [1, 0, 0, 0; 
     0, 1, 0, 0; 
     0, 0.1235, 0.7727, 0.1901; 
     0, 1.217, 0.03097, 0.9683];
B = [-3.561e-06; 0; 0.0007592; 0.007484];
C = [1, 0, 0, 0; 0, 0, 0, 1];
D = [0; 0];

% Constraints
Tf = 20;
Ts_min = 0; Ts_max = 40;
Tc_min = 0; Tc_max = 40;
I_min = 0; I_max = 40;

% Initial state
x0 = [1; 0; Tf; Tf];

% Problem setup
n = size(A, 1); % Number of states
m = size(B, 2); % Number of inputs
N = 20; % Horizon length

% State cost matrix
Q = diag([100, 0, 0, 0]); % Minimize SoC
R = 0.1; % Input cost matrix
Pf = 10 * Q; % Terminal cost matrix

tf = 200; % Simulation time

% Equality constraints
I = eye(n);
Aeq1 = kron(eye(N), I) + kron(diag(ones(N-1, 1), -1), -A);
Aeq2 = kron(eye(N), -B);
Aeq = [Aeq1, Aeq2];
beq = [x0; zeros(n*(N-1), 1)];  % Modified to include x0 correctly

% Inequality constraints
F_state = kron(eye(N), [0 0 1 0; 0 0 -1 0; 0 0 0 1; 0 0 0 -1]);
h_state = repmat([Ts_max; -Ts_min; Tc_max; -Tc_min], N, 1);

F_input = kron(eye(N), [1; -1]);
h_input = repmat([I_max; -I_min], N, 1);

F = blkdiag(F_state, F_input);
G = [zeros(size(F_state, 1), N*m); kron(eye(N), [1; -1])];
h = [h_state; h_input];

% Initialization
x = x0;
x_vec = [x0 zeros(n, tf)];
u_vec = zeros(m, tf);

% MPC loop
for iter = 1:tf
    % Objective function
    Qbar = blkdiag(kron(eye(N-1), Q), Pf);
    Rbar = kron(eye(N), R);
    H = blkdiag(Qbar, Rbar);
    f = [];

    % Solve QP
    Z = quadprog(2*H, f, F, h, Aeq, beq);

    % Extract control input and state trajectory
    x_traj = reshape(Z(1:N*n), [n, N]);  % Extract state trajectory
    u_traj = reshape(Z(N*n+1:end), [m, N]);  % Extract control trajectory

    % Apply first control input
    u = u_traj(:, 1);

    % Update the state using system dynamics
    x = A * x + B * u;

    % Store the updated state and control input
    x_vec(:, iter+1) = x;
    u_vec(:, iter) = u;

    % Update beq for the next iteration to include the new state
    beq = [x; zeros(n*(N-1), 1)];
end

% Plotting results
figure(1);
subplot(3, 1, 1);
plot(0:tf, x_vec);
legend('SoC', 'V1', 'Ts', 'Tc');
xlabel('Time [s]');
ylabel('States');
title('State trajectories');

subplot(3, 1, 2);
stairs(0:tf-1, u_vec);
xlabel('Time [s]');
ylabel('Control input');
title('Control input trajectory');

subplot(3, 1, 3);
plot(0:tf, x_vec(1,:));
legend('SoC');
xlabel('Time [s]');
ylabel('States');
title('State trajectories');

fprintf('Code finished\n');
