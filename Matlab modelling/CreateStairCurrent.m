% Given pulse discharge time and current vectors
pulseDischargeTime = [0, 3600, 4536, 7200, 8136, 10800, 11736, 14400, ...
                      15336, 18000, 18936, 21600, 22536, 25200, 26136, ...
                      28800, 29736, 32400, 33336, 36000, 36936, 40536];
                  
pulseDischargeCurrent = [0, 30, 0, 30, 0, 30, 0, 30, 0, 30, 0, 30, 0, 30, ...
                         0, 30, 0, 30, 0, 30, 0]/30*8.3278;

% Create time and current vectors for stair plot
stair_time = [];
stair_current = [];

% Iterate through each interval defined by pulse discharge time
for i = 1:length(pulseDischargeTime)-1
    % Add time points for the start and end of the interval
    stair_time = [stair_time, pulseDischargeTime(i), pulseDischargeTime(i+1)-1];
    
    % Add current values for the interval (first point is the current at the start)
    stair_current = [stair_current, pulseDischargeCurrent(i), pulseDischargeCurrent(i)];
end

% Plot stair plot
plot(stair_time, stair_current);
xlabel('Time (s)');
ylabel('Current (A)');
title('Pulse Discharge Current Profile (Stair Plot)');
