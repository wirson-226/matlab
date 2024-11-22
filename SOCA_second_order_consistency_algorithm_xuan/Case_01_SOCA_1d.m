clear;
close all;
clc;


%initial state
X0 = [1;2;3;4;5];
V0 = [0;0;0;0;0];

%邻接矩阵 A
A = [
    0 1 0 1 0;
    1 0 1 0 1;
    0 1 0 0 1;
    1 0 0 0 1;
    0 1 1 1 0
];

% D 矩阵

D = diag(sum(A,2));

% L矩阵
L = D - A;
r = 1;
U0 = -(L * X0 + r * L * V0 );


[t,y] = ode45(@(t,y) odefcn(t,y,L,r), [0,10], [X0,V0,U0]);

% Graphs
% y [X1;x2;x3;x4;x5;v1;v2;v3;v4;v5;u1;u2;u3;u4;u5]
X = y(:,1:5);
V = y(:,6:10);
U = y(:,11:15);
figure(1);
plot(t,X);
title('position');
xlabel('time');
ylabel('X');
figure(2);
plot(t,V);
title('velocity');
xlabel('time');
ylabel('V');
figure(3);
plot(t,U);
title('Input acceleration');
xlabel('time');
ylabel('U');

% function
function dydt = odefcn(t,y,L,r)
% [X1;x2;x3;x4;x5;v1;v2;v3;v4;v5;u1;u2;u3;u4;u5] dydt 导数
dydt = zeros(15,1);
dydt(1:5) = y(6:10);
dydt(6:10) = y(11:15);
dydt(11:15) = -(L * y(6:10) + r * L * y(11:15) );
end

