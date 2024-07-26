function [c, ceq] = DPConstraints(I, x0, lastI)
    % Initial state of the system
    x(:,1) = x0(:);%[1 0 20 20 1]';
    N = length(I);    % Number of steps
    TMax = 40;        % Maximum allowable core temperature
    deltaT = 1;       % Time step
    zeroOrderHoldCurrent = [];
    for i = 1:length(I)
        zeroOrderHoldCurrent = [zeroOrderHoldCurrent repmat(I(i), 1, 22)];
    end
    % Inequality constraints array
    c = [];
    ceq = [];
    % if ~isempty(lastI)
    %     c = [c; I(1) - lastI - 1;lastI - I(1) - 1];
    % end
    % if ~isempty(lastI)
    %     % Overlap first 10 with last 5 of previous
    %     ceq = [ceq; I(1) - lastI(end)];
    % end
    for k = 2:length(zeroOrderHoldCurrent)
        x(:, k) = DPBatteryDynamics(x(:, k-1), zeroOrderHoldCurrent(k-1), deltaT);
        % Add temperature constraint (core temperature should be <= TMax)
        c = [c; x(4, k) - TMax];
        % Input rate of change
        % c = [c; I(k) - I(k-1) - 1; I(k-1) - I(k) - 1];
        c = [c; zeroOrderHoldCurrent(k) - zeroOrderHoldCurrent(k-1) - 1; zeroOrderHoldCurrent(k-1) - zeroOrderHoldCurrent(k) - 1];
    end
    
    % No equality constraints are needed
    ceq = [x(5,end) - 0];
end
