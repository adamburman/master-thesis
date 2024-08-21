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

%%
% Operating point, starting with TcBar = 40 Centigrade
TcBar = 40;
TsBar = (Ru*TcBar + Rc*Tf) / (Ru+Rc);
IBar = sqrt(Cc*(TcBar - TsBar) / (Rc*Cc*(R1+R0)));
V1Bar = R1*IBar;
%%
fprintf(sprintf('Operating points:\nV1 = %.4f\nTs = %.4f\nTc = %.4f\nI = %.4f\n', V1Bar, TsBar, TcBar, IBar))

syms SoC V1 Ts Tc I
x = [SoC V1 Ts Tc]';
u = I;

    % x = soc, V1, Ts, Tc
f = [-1/(3600*Cn)*I;...
    -R1/C1*V1 + I/C1;...
    (Tf-Ts)/(Ru*Cs) - (Ts - Tc)/(Rc*Cs);...
    (Ts-Tc)/(Rc*Cc) + I*(V1 + R0*I)/Cc]; % State 2 (V1) didn't have + I/C1

h = [Ts]; %Had I as well but mpc doesn't allow feedforward


Al = double(subs(jacobian(f, x'), [I V1], [IBar V1Bar]));
Bl = double(subs(jacobian(f, u),[I V1], [IBar V1Bar]));
Cl = double(jacobian(h, x'));
Dl = double(jacobian(h, u));

% Al = double(subs(jacobian(f, x'), [I V1], [0 0]));
% Bl = double(subs(jacobian(f, u),[I V1], [0 0]));
% Cl = double(jacobian(h, x'));
% Dl = double(jacobian(h, u));

lSysModule = ss(Al,Bl,Cl,Dl);
ldSysModule = c2d(lSysModule, 1);
% x0 = [1 0 Tf Tf]';

%%
% x = x0(1:4);
% 
% figure(3);clf;
% for k = 1:28799
% % x(:,k+1) = linSysModule.A*x(:,k) + linSysModule.B*testC(k);
% x(:,k+1) = linsys19.A*x(:,k) + linsys19.B*testC(k);
% 
% if mod(k, 1000) == 0
% subplot(4,1,1);
% plot(x(1,:))
% subplot(4,1,2);
% plot(x(2,:))
% subplot(4,1,3);
% plot(x(3,:))
% subplot(4,1,4);
% plot(x(4,:))
% pause(0.01)
% end
% end
% % figure(1);clf;
% % subplot(4,1,1);
% % plot(x(1,:))
% % subplot(4,1,2);
% % plot(x(2,:))
% % subplot(4,1,3);
% % plot(x(3,:))
% % subplot(4,1,4);
% % plot(x(4,:))