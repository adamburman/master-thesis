function J = DPCostFunction(I, holdTime)
    x(:,1) = [1 0 20 20 1]';%x0;
    N = length(I);
    % holdTime = 500; % Each current value is held for 22 seconds
    deltaT = 1; % Time step
    I = repelem(I,holdTime);
    % Apply dynamics over extended time due to zero-order hold
    for k = 2:N*holdTime
        % for t = 1:holdTime
            x(:, k) = DPBatteryDynamics(x(:, k-1), I(k-1), deltaT);
        % end
    end

    % Minimize steps to reach SoE <= 0.95 (or adjust threshold as needed)
    J = find(x(5,:) <= 0.0, 1, 'first');
    % J = x(5,end);%trapz(x(5,:));
    % J = 2*J;
    if isempty(J)
        % J = N*holdTime*(1+x(5,end));  % No solution found, penalize with maximum steps
        J = N*holdTime + holdTime*x(5,end);
    end
    % if ~isempty(lastI)
    %     J = J + norm(abs(I(1:length(lastI)) - lastI)).^2;
    % end
end
