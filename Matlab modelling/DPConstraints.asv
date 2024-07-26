function [c, ceq] = DPConstraints(I, holdTime)
    % Initial state of the system
    x(:,1) = [1 0 20 20 1]';%x0(:);
    N = length(I);    % Number of steps
    TMax = 40;        % Maximum allowable core temperature
    % holdTime = 500;    % Each current value is held for 22 seconds
    deltaT = 1;       % Time step
    I = repelem(I, holdTime);
    % Inequality constraints array
    c = [];
    ceq = [];

    % Dynamics with zero-order hold
    for k = 2:N*holdTime
            x(:, k) = DPBatteryDynamics(x(:, k-1), I(k-1), deltaT);
            % Add temperature constraint (core temperature should be <= TMax)
            c = [c; x(4, k) - TMax];
        % Input rate of change
        c = [c; I(k) - I(k-1) - 1; I(k-1) - I(k) - 1];
    end
    c = [c; -x(5,end)];
    % No equality constraints are needed
    % ceq = [ceq; x(5,end)];
end
