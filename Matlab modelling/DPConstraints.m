function [c, ceq] = DPConstraints(I)
    % Initial state of the system
    x(:,1) = [1 0 20 20 1]';
    N = length(I);    % Number of steps
    TMax = 50;        % Maximum allowable core temperature
    deltaT = 1;       % Time step

    % Inequality constraints array
    c = [];
    for k = 2:N
        x(:, k) = DPBatteryDynamics(x(:, k-1), I(k-1), deltaT);
        % Add temperature constraint (core temperature should be <= TMax)
        c = [c; x(4, k) - TMax];
    end
    
    % No equality constraints are needed
    ceq = [x(5,N) - 0.8];
end
