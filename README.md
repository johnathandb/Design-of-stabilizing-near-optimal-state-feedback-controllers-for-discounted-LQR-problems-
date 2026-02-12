# Design-of-stabilizing-near-optimal-state-feedback-controllers-for-discounted-LQR-problems-
This repository contain Matlab's files used to design near-optimal slabilizing state feedback controllers for a discounted LQR problem for a real implementation on a Quanser QUBE 2.

## qube_system_params.m
This file contains the values of the different mechanical and electrical parameters of the Quanser QUBE 2. These values have been obtained via an identification process, which allowed to derive a mathematical nonlinear model close to the real behaviour of the system observed in practise.

## discrete_LQR_control.slx
This file contains the Simulink file to use in order to control the system with a state-feedback law obtained by solving one of the two convex optimization problems in the matlab file 'control_balance_reduced_order.m'.

## Figures_obtained.m
This file contains the Matlab code used to obtain the figures 3,4 and 5 presented in the paper 'Design of stabilizing near-optimal state feedback controllers for discounted LQR problems'.

## control_balance_reduced_order.m
This file contains the code used to derived the linearization and discretization of the nonlinear model of the Quanser QUBE 2 around the equilibrium point corresponding to the pendulum in the upright position. 
The solution of the convex optimization problems presented in the paper 'Design of stabilizing near-optimal state feedback controllers for discounted LQR problems' is also presented (for the one ensuring an upper-bound on the cost with a decay rate of $\alpha=0.98$ and for the one ensuring a control gain close to the optimal one with $\alpha=0.95$.).

## File order_reduced_model 
This file 2 others files names 'Experience_1' and 'Experience_2'. Each of these files contains the detailed setting of the convex-optimization problem solved in a file .txt, the data obtained during the experiment are stored in matlab's files and the video of the experience is presented in a file .mp4.

If you don't want to download the mp4 file of the 2 experiments. these one can also be found on youtube at the following URL

Experiment 1:
[▶️ Regarder la vidéo sur YouTube](https://www.youtube.com/watch?v=0R-1BmAn_OU)

 
Experiment 2:
[▶️ Regarder la vidéo sur YouTube](https://www.youtube.com/watch?v=_ZL3x4MsT28)

 
