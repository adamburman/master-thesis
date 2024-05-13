x0 = [1 0 20 20]';

dydx_int = zeros(4,1e4);
dydx_int(:,1) = x0;
dydx = zeros(4,1e4);
% for i = 2:10000
% dydx(:,i) = cell_model_diff_eq(dydx_int(:,i), input_current(i));
% dydx_int(:, i) = dydx_int(:,i-1) + dydx(:,i);
% end

for i = 1:(9999)
    dydx_int(:,i+1) = dydx_int(:,i) + [-input_current(i)/(3600*Cn);...
                                    -dydx_int(2,i)/(R1*C1) + input_current(i)/C1;...
                                    (Tf-dydx_int(3,i))/(Ru*Cs) - (dydx_int(3,i)-dydx_int(4,i))/(Rc*Cs);...
                                    (dydx_int(3,i)-dydx_int(4,i))/(Rc*Cs) + 1/Cs*(dydx_int(2,i)+R0*input_current(i))*input_current(i)];

end


function dydx = cell_model_diff_eq(x, u)
    Cn = 2.3; % Ah
Cc = 62.7; % J/K
Cs = 4.5; % J/K
Ru = 15; % K/W
Rc = 1.94; % K/W
Tf = 20; % Centrigrade
Em = 4.1897;
R0 = 0.0099131;



R1 = 3e-4;
C1 = 12.6138;
    dydx = zeros(4,1);
    dydx(1) = -u/(3600*Cn);
    dydx(2) = -x(2)/(R1*C1) + u/C1;
    dydx(3) = (Tf-x(3))/(Ru*Cs) - (x(3)-x(4))/(Rc*Cs);
    dydx(4) = (x(3)-x(4))/(Rc*Cc) + 1/Cc*(x(2)+R0*u)*u;


end