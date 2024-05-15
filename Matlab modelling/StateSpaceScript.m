% Cn = 78; %2.3; % Ah
% Cc = 62.7; % J/K
% Cs = 4.5; % J/K
% Ru = 15; % K/W
% Rc = 1.94; % K/W
% Tf = 20; % Centrigrade
% Em = 4.1897;
% R0 = 0.0099131;
% R1 = 3e-4;
% C1 = 12.6138;

% Ac = [0 0 0 0;...
%     -R1/C1 0 0 0;...
%     ]


% Operating point, starting with TcBar = 40 Centigrade
TcBar = 40;
TsBar = (Ru*TcBar + Rc*Tf) / (Ru+Rc);
IBar = sqrt(Cc*(TcBar - TsBar) / (Rc*Cc*(R1+R0)));
V1Bar = R1*IBar;

fprintf(sprintf('Operating points:\nV1 = %.4f\nTs = %.4f\nTc = %.4f\nI = %.4f\n', V1Bar, TsBar, TcBar, IBar))

syms SoC V1 Ts Tc I
x = [SoC V1 Ts Tc]';
u = I;

    % x = soc, V1, Ts, Tc
f = [-1/(3600*Cn)*I;...
    -R1/C1*V1;...
    (Tf-Ts)/(Ru*Cs) - (Ts - Tc)/(Rc*Cs);...
    (Ts-Tc)/(Rc*Cc) + I*(V1 + R0*I)/Cc];

h = [SoC; Tc]; %Had I as well but mpc doesn't allow feedforward


Al = double(subs(jacobian(f, x'), I, IBar));
Bl = double(subs(jacobian(f, u),[I V1], [IBar V1Bar]));
Cl = double(jacobian(h, x'));
Dl = double(jacobian(h, u));

lSys = ss(Al,Bl,Cl,Dl);
ldSys = c2d(lSys, 1);
x0 = [1 0 Tf Tf]';

