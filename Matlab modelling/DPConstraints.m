function [c, ceq] = DPConstraints(I, holdTime, orderHold)
if orderHold == 0
    % Initial state of the system
    x(:, 1) = [1 0 20 20 1]'; % [SoC, V1, Ts, Tc, SoE]
    N = length(I); % Number of control intervals
    TMax = 40; % Maximum allowable core temperature
    deltaT = 1; % Time step for dynamics
    I = repelem(I, holdTime); % Expand control inputs over holdTime

    % Inequality and equality constraints arrays
    c = [];
    ceq = [];

    % Dynamics with zero-order hold
    for k = 2:N * holdTime
        x(:, k) = DPBatteryDynamics(x(:, k-1), I(k-1), deltaT);

        % Add temperature constraint (core temperature should be <= TMax)
        c = [c; x(4, k) - TMax];
    end

    % No equality constraints are needed
    % ceq = [];

    % Ensure SoE must reach exactly zero to satisfy the end condition
    % ceq = [ceq; x(5, end)];
elseif orderHold == 1
    % Initial state of the system
    x(:, 1) = [1 0 20 20 1]'; % [SoC, V1, Ts, Tc, SoE]
    N = length(I); % Number of control intervals
    TMax = 40; % Maximum allowable core temperature
    deltaT = 1; % Time step for dynamics

    % Calculate the total simulation steps
    totalSteps = N * holdTime;

    % Time steps for interpolation
    timeSteps = linspace(1, totalSteps, N);

    % Expanded current array with linear interpolation between control inputs
    I_expanded = interp1(timeSteps, I, 1:totalSteps);

    % Inequality and equality constraints arrays
    c = [];
    ceq = [];

    % Dynamics with linear interpolation
    for k = 2:totalSteps
        x(:, k) = DPBatteryDynamics(x(:, k-1), I_expanded(k-1), deltaT);

        % Add temperature constraint (core temperature should be <= TMax)
        c = [c; x(4, k) - TMax];
    end

    % No equality constraints are needed
    % ceq = [];

    % Ensure SoE must reach exactly zero to satisfy the end condition
    % ceq = [ceq; x(5, end)];
end
end
