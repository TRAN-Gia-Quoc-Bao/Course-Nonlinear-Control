%% Robot arm control

%% Default commands
close all; clear all; clc;

%% Parameters
global J Bm k m l g JR
J = 4e-4; Bm = 0.015; Bl = 0; k = 0.8; m = 0.3; l = 0.3; g = 9.8;
JR = 4.2e-4;

%% Linear system
A = [0 1 0 0; -k/J -Bl/J k/J 0; 0 0 0 1; k/J 0 -k/J -Bm/J];
B = [0; 0; 0; 1/J];
C = [1 0 0 0];
D = [0];
sysLinear = ss(A, B, C, D);
rank(ctrb(sysLinear)); % = 4, controllable
rank(obsv(sysLinear)); % = 4, observable

%% I/O
% % Perform I/O linearization
% n = 4;
% % The following command creates a vector x = [x(1), x(2), x(3), x(4)] of symbolic variables
% x = sym('x', [n, 1]);
% % create the symbolic control variable
% syms u;
% 
% % It is important not to create functions here but vector and/or expressions
% f = [       x(2);
%             -1/J*(k*(x(1) - x(3)) + m*g*l*cos(x(1)));
%             x(4);
%             1/J*(-Bm*x(4) + k*(x(1) - x(3)) + u)];
% dyn = matlabFunction(f, 'vars', [x; u]);
% h = pi/2 - x(1);
% 
% [a, b, diffeo] = computeLinearization(f, h, x, u);

% Or just load this if you do not have the Symbolic Math Toolbox (we calculated the results)
% load('calculatedFunctions.mat');
% System after I/O linearization
sysIO = ss([0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 0], [0; 0; 0; 1], [1 0 0 0], 0);
rank(ctrb(sysIO)); % = 4, controllable
rank(obsv(sysIO)); % = 4, observable
P = -100*ones(4, 1);
F = acker(sysIO.a, sysIO.b, P);
G = inv(sysIO.c*inv(-sysIO.a + sysIO.b*F)*sysIO.b);
sysCL = ss(sysIO.a - sysIO.b*F, sysIO.b*G, sysIO.c, sysIO.d);
step(sysCL); title('Linear control of the I/O linearized system - unit step response', 'FontSize', 14); grid on; set(findall(gca, 'Type', 'Line'), 'LineWidth', 2); set(gca, 'FontSize', 14);

%% Integral action
sysIOInt = ss([0 1 0 0 0; 0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 0; -1 0 0 0 0], [0; 0; 0; 1; 0], [1 0 0 0 0], 0);
rank(ctrb(sysIOInt)); % = 5, controllable
rank(obsv(sysIOInt)); % = 4, non-observable
PInt = 100*[-1 -1 -1 -1 -10];
FInt = acker(sysIOInt.a, sysIOInt.b, PInt);
% Qe = [0 0 0 0 0; 0 0 0 0 0; 0 0 0 0 0; 0 0 0 0 0; 0 0 0 0 1];
% Re = 0.01;
% FInt = lqr(sysIOInt.a, sysIOInt.b, Qe, Re);
F = FInt(1 : 4);
H = FInt(5);
sysCLInt = ss(sysIOInt.a - sysIOInt.b*FInt, [0; 0; 0; 0; 1], sysIOInt.c, sysIOInt.d);
step(sysCLInt); title('Linear control of the I/O linearized system with integral action - unit step response', 'FontSize', 14); grid on; set(findall(gca, 'Type', 'Line'), 'LineWidth', 2); set(gca, 'FontSize', 14);