# Design-of-stabilizing-near-optimal-state-feedback-controllers-for-discounted-LQR-problems-
This repository contain Matlab's files used to design near-optimal slabilizing state feedback controllers for a discounted LQR problem for a real implementation on a Quanser QUBE 2.

## qube_system_params.m
This file contains the values of the different mechanical and electrical parameters of the Quanser QUBE 2. These values have been obtained via an identification process ensuring a mathematical nonlinear model, which is close to the real behaviour of the system observed.

## discrete_LQR_control.slx
This file contains the Simulink file to use in order to implement the control gain obtained by solving one of the two convex optimization problems in the matlab file 'control_balance_reduced_order.m'.

## Figures_obtained.m
This file contains the Matlab code used to obtain the figures 3,4 and 5 presented in the paper 'Design of stabilizing near-optimal state feedback controllers for discounted LQR problems'.

## control_balance_reduced_order.m
This file contains the code used to derived the linearization and discretization of the nonlinear model of the Quanser QUBE 2 around the equilibrium point corresponding to the pendulum in the upright position. 
The solution of the convex optimization problems presented in the paper 'Design of stabilizing near-optimal state feedback controllers for discounted LQR problems' is also presented (for the one ensuring an upper-bound on the cost with a decay rate of $\alpha=0.98$ and for the one ensuring a control gain close to the optimal one with $\alpha=0.95$.).
