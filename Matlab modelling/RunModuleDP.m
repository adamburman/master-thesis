N = 200;  % Number of decision variables
holdTime = 200;  % Duration each current value is held
orderHold = 0;
initialGuess = zeros(1, N);

if exist('initialCurrentGuessDP.mat', 'file')
    load('initialCurrentGuessDP.mat');
end

if length(initialCurrentGuessDP) > N
    initialGuess = initialCurrentGuessDP(1:N);
else
    initialGuess(1:length(initialCurrentGuessDP)) = initialCurrentGuessDP;
end

initialGuess = 10*ones(1, N);
initialGuess(1:35) = 40*ones(1,35);
% initialGuess = 

A = [];
b = [];
Aeq = [];
beq = [];
lb = zeros(1, N);
ub = 40 * ones(1, N);

% Construct A matrix for rate of change constraints
A = zeros(2 * (N - 1), N);
b = ones(2 * (N - 1), 1);

% For I(k+1) - I(k) <= 1 and I(k) - I(k+1) <= 1
for k = 1:(N - 1)
    A(k, k) = -1;
    A(k, k + 1) = 1;

    A(N - 1 + k, k) = 1;
    A(N - 1 + k, k + 1) = -1;
end

options = optimoptions('fmincon', 'Display', 'iter', ...
    'MaxFunctionEvaluations', 200 * N, ...
    'MaxIterations', 1e4, ...
    'UseParallel', true);

optVars = fmincon(@(I) DPModuleCostFunction(I, holdTime, orderHold), initialGuess, A, b, Aeq, beq, lb, ub, @(I) DPModuleConstraints(I, holdTime, orderHold), options);

DPModuleSolution = repelem(optVars, holdTime);
%%

figure(1); clf; hold on;
plot(repelem(initialGuess, holdTime), '--k', 'linewidth', 1);
plot(DPModuleSolution, 'r', 'LineWidth', 1);
legend('Initial guess', 'Solver output');

x0 = [1 0 20 20 1]';
x = x0;

for k = 2:N * holdTime
    x(:, k) = DPBatteryModuleDynamics(x(:, k - 1), DPModuleSolution(k), 1);
end

figure(2); clf; titleStrings = {'SoC', 'V1', 'Ts', 'Tc', 'SoE'};
for i = 1:5
    subplot(6, 1, i);
    plot(x(i, :),'LineWidth',1);
    title(titleStrings{i});
end
subplot(6, 1, 6);
plot(DPModuleSolution);
title('DP Solution');

playCompletionSound();