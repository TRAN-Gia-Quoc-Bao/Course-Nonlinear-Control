%% Blending control

%% Default commands
close all; clear all; clc;

%% Parameters
% Tank
a = 24; S = pi*5*5; kv = 1.2;
% Equilibrium
H0 = 16; T0 = 45; Q0 = 48; Tw = 70; Tc = 20;

%% Linearized systems
sysH = ss(-a/(2*S*sqrt(H0)), 1, 1, 0);
sysTheta = ss(-(2*Q0)/(S*H0), 1, 1, 0);
% To get back the control inputs qw and qc
change = inv([1/S 1/S; (Tw - T0)/(S*H0) (Tc - T0)/(S*H0)]);
% Controllers for 4 times faster
kHFast = 3*a/(2*S*sqrt(H0));
gHFast = inv(1*inv(a/(2*S*sqrt(H0)) + 1*kHFast)*1);
sysHCLFast = ss(-a/(2*S*sqrt(H0)) - 1*kHFast, 1*gHFast, 1, 0);

kThetaFast = 3*(2*Q0)/(S*H0);
gThetaFast = inv(1*inv((2*Q0)/(S*H0) + 1*kThetaFast)*1);
sysThetaCLFast = ss(-(2*Q0)/(S*H0) - 1*kThetaFast, 1*gThetaFast, 1, 0);
figure; 
subplot(121); step(sysH); hold on; step(sysHCLFast); legend('Open loop', 'Closed loop', 'FontSize', 14); title('Linear control of h (4 times faster)', 'FontSize', 14); grid on; set(findall(gca, 'Type', 'Line'), 'LineWidth', 2); set(gca, 'FontSize', 14);
subplot(122); step(sysTheta); hold on; step(sysThetaCLFast); legend('Open loop', 'Closed loop', 'FontSize', 14); title('Linear control of \theta (4 times faster)', 'FontSize', 14); grid on; set(findall(gca, 'Type', 'Line'), 'LineWidth', 2); set(gca, 'FontSize', 14);

%% Controllers for 4 times slower
kHSlow = -0.75*a/(2*S*sqrt(H0));
gHSlow = inv(1*inv(a/(2*S*sqrt(H0)) + 1*kHSlow)*1);
sysHCLSlow = ss(-a/(2*S*sqrt(H0)) - 1*kHSlow, 1*gHSlow, 1, 0);

kThetaSlow = -0.75*(2*Q0)/(S*H0);
gThetaSlow = inv(1*inv((2*Q0)/(S*H0) + 1*kThetaSlow)*1);
sysThetaCLSlow = ss(-(2*Q0)/(S*H0) - 1*kThetaSlow, 1*gThetaSlow, 1, 0);
figure; 
subplot(121); step(sysH); hold on; step(sysHCLSlow); legend('Open loop', 'Closed loop', 'FontSize', 14); title('Linear control of h (4 times slower)', 'FontSize', 14); grid on; set(findall(gca, 'Type', 'Line'), 'LineWidth', 2); set(gca, 'FontSize', 14);
subplot(122); step(sysTheta); hold on; step(sysThetaCLSlow); legend('Open loop', 'Closed loop', 'FontSize', 14); title('Linear control of \theta (4 times slower)', 'FontSize', 14); grid on; set(findall(gca, 'Type', 'Line'), 'LineWidth', 2); set(gca, 'FontSize', 14);

%% Controller for nonlinear system
% 4 times faster
kHFast = 4*a/(2*S*sqrt(H0));
gHFast = inv(1*inv(0 + 1*kHFast)*1);

kThetaFast = 4*(2*Q0)/(S*H0);
gThetaFast = inv(1*inv(0 + 1*kThetaFast)*1);

%% Controller for nonlinear system + INTEGRAL
% We use LQR with equal weights for state and integral
Qe = [1 0; 0 1];
Re = 0.01;
Fe = lqr([0 0; -1 0], [1; 0], Qe, Re);
kHFast = Fe(1);
fHFast = Fe(2);
kThetaFast = Fe(1);
fThetaFast = Fe(2);