% ===================================================== %
%    This file contains ALL Qube system parameters.     %
%    Last updated: 02-02-2026                           %
% ===================================================== %

% ----------------------------------------------------- %
%                  QUBE Parameters                      %
% ----------------------------------------------------- %
QUBE_params.m_p = 0.024;                        % (kg) pendulum mass
QUBE_params.l_s = 0.085;                        % (m) rotary servo arm length
QUBE_params.l_p = 0.129;                        % (m) pendulum length
QUBE_params.g = 9.81;                           % (m.s^2) gravitational accel
QUBE_params.R_m = 8.4;                          % (ohm) motor resistance
QUBE_params.L_m = 1.16e-3;                      % (H) motor inductance
QUBE_params.K_m = 0.042;                        % (Nm.A^-1) motor torque constant


% Experimentally derived parameters
QUBE_params.J_s = 3.12e-4;                     % (kg.m^2) rotary servo moment of inertia                              
QUBE_params.J_p = 1.29696e-4;                  % (kg.m^2) pendulum moment of inertia
QUBE_params.D_s = 6e-4;                        % (kg.m^2.s^-1) rotary servo viscous damping coefficient
QUBE_params.D_p = 1.07891e-5;                  % (kg.m^2.s^-1) rotary servo viscous damping coefficient
QUBE_params.K=1.54e-3;                         % (Nm.rad^-1) torsional stiffness of the wire

% Set time before controller takes action
calib_time = 5;

% Sampling period for discrete-time observer/controller pair
delta = 0.002;