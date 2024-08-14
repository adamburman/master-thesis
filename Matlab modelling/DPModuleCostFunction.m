function J = DPCostFunction(I, holdTime, orderHold)
if orderHold == 0
    % Initial state of the system
    x(:, 1) = [1 0 20 20 1]'; % [SoC, V1, Ts, Tc, SoE]
    N = length(I);
    deltaT = 1;
    I = repelem(I, holdTime);

    % Apply dynamics over extended time due to zero-order hold
    for k = 2:N * holdTime
        x(:, k) = DPBatteryModuleDynamics(x(:, k - 1), I(k - 1), deltaT);
    end

    % Find the first index where SoE is zero or below
    finalStep = find(x(5, :) <= 0.0, 1, 'first');

    % Cost is based on the time it takes to reach zero SoE
    if isempty(finalStep)
        % If SoE never reaches zero, penalize heavily
        J = N * holdTime + abs(x(5, end)) * 1e5;
    else
        % Cost is primarily determined by the time to reach zero SoE
        J = 1e1*finalStep;
    end
    J = J + trapz(I)/1e5;
    % figure(3);clf;plot(I);
elseif orderHold == 1
    % Initial state of the system
    x(:, 1) = [1 0 20 20 1]'; % [SoC, V1, Ts, Tc, SoE]
    N = length(I);
    deltaT = 1;

    % Calculate the total simulation steps
    totalSteps = N * holdTime;

    % Time steps for interpolation
    timeSteps = linspace(1, totalSteps, N);

    % Expanded current array with linear interpolation between control inputs
    I = interp1(timeSteps, I, 1:totalSteps);

    % Apply dynamics over expanded time with linear interpolation
    for k = 2:totalSteps
        x(:, k) = DPBatteryModuleDynamics(x(:, k - 1), I(k - 1), deltaT);
    end

    % Find the first index where SoE is zero or below
    finalStep = find(x(5, :) <= 0.0, 1, 'first');

    % Cost is based on the time it takes to reach zero SoE
    if isempty(finalStep)
        % If SoE never reaches zero, penalize heavily
        J = totalSteps + abs(x(5, end)) * 1000;
    else
        % Cost is primarily determined by the time to reach zero SoE
        J = finalStep;
    end
    % figure(3);clf;plot(I);
end
end
