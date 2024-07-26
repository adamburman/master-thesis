function J = DPCostFunction(I, x0, lastI)
    x(:,1) = x0;%[1 0 20 20 1]';
    N = length(I);
    deltaT = 1;
    zeroOrderHoldCurrent = [];
    for i = 1:length(I)
        zeroOrderHoldCurrent = [zeroOrderHoldCurrent repmat(I(i), 1, 22)];
    end
    for k = 2:length(zeroOrderHoldCurrent)
        x(:, k) = DPBatteryDynamics(x(:, k-1), zeroOrderHoldCurrent(k-1), deltaT);
    end
    % Minimize steps to reach SoE <= 0.95 (or adjust threshold as needed)
    J = find(x(5,:) <= 0.0, 1, 'first');
    % J = J;
    if isempty(J)
        J = N + N*x(5,end);  % No solution found, penalize with maximum steps
    end
    if ~isempty(lastI)
        J = J + norm(abs(I(1:length(lastI)) - lastI)).^2;
    end
end
