% ===================================================== %
%    Quanser QUBE Demonstration: System Modelling       %
%    This script symbolically computes a linearised     %
%    system model for the Quanser QUBE in balance       %
%    control. This script can either be run in sections %
%    or run altogether.                                 %
%    Last updated: 02-02-2026                           %
% ===================================================== %

clear zeros
clc
clearvars
qube_system_params;

% ----------------------------------------------------- %
%            Define equations of motion                 %
% ----------------------------------------------------- %

% System variables/inputs
syms psi psi_d psi_dd phi phi_d phi_dd
syms V_m

% Symbolic parameters
syms m_p l_p l_s g R_m L_m K_m J_s J_p D_s D_p K

% Define states, inputs, outputs and equilibrium of interest
x = [phi;psi; phi_d; psi_d];
u = V_m;
y_pos = psi;

% Equilibrium for position control. Note that equilibrium rotary position does not affect linearisation
x_pos_eqm = [0; 0; 0; 0];        
u_pos_eqm = 0;
y_pos_eqm = 0;

% Define nonlinear dynamics
f_1 = (J_s + J_p*sin(phi)^2)*psi_dd + 2*J_p*sin(phi)*cos(phi)*phi_d*psi_d + ...
      (1/2)*m_p*l_p*l_s*cos(phi)*phi_dd +K*psi - (1/2)*m_p*l_p*l_s*sin(phi)*phi_d^2 + D_s*psi_d - K_m*((-K_m/R_m)*psi_d+(1/R_m)*V_m);
f_2 = J_p*phi_dd + (1/2)*m_p*l_p*l_s*cos(phi)*psi_dd - J_p*sin(phi)*cos(phi)*psi_d^2 + (1/2)*m_p*l_p*g*sin(phi) + D_p*phi_d;


disp("==========================================================================")
disp("We consider the reduced linear system model for balance control")
disp("==========================================================================")

% Define incremental variables for phi
syms delta_phi delta_phi_d delta_phi_dd

% Equilibrium for balance control. 
x_bal_eqm = [pi; 0; 0; 0];        
u_bal_eqm = 0;
y_bal_eqm = pi;

% ----------------------------------------------------- %
%       Perform linearisation for balance control       %
% ----------------------------------------------------- %

f_1_lin = subs(jacobian(f_1, [x; phi_dd; psi_dd; u]), [x;phi_dd;psi_dd; u], [x_bal_eqm; 0; 0; u_bal_eqm])*[delta_phi;psi; delta_phi_d;psi_d; delta_phi_dd;psi_dd; u];
f_2_lin = subs(jacobian(f_2, [x; phi_dd;psi_dd; u]), [x;  phi_dd;psi_dd; u], [x_bal_eqm; 0; 0; u_bal_eqm])*[delta_phi; psi; delta_phi_d;psi_d; delta_phi_dd;psi_dd; u];

disp('The reduced linearised equations of motion for balance control are given by:');
display(string(f_1_lin) + "= 0")
display(string(f_2_lin) + "= 0")

% ----------------------------------------------------- %
%   Generate a state-space model for position control   %
% ----------------------------------------------------- %

% Begin by isolating the highest order derivatives
soln = solve([f_1_lin, f_2_lin] == 0, [delta_phi_dd;psi_dd]);

% Construct full state space model
x_dot = [delta_phi_d; psi_d; soln.delta_phi_dd; soln.psi_dd];

% Define the new state variable 
delta_x = [delta_phi;psi; delta_phi_d;psi_d];
delta_y_bal = delta_phi;

% Now extract state-space matrices
A = simplify(jacobian(x_dot, delta_x));
B = simplify(jacobian(x_dot, u));
C = simplify(jacobian(delta_y_bal, delta_x));

% Don't need feedthrough matrix D, not relevant here
disp('The corresponding reduced state-space matrices are then given by:');
disp('A = '); disp(A);
disp('B = '); disp(B);
disp('C = '); disp(C);

% Generate numeric matrices
A_bal_num = double(subs(A, [J_p, J_s, m_p, l_p, l_s, g, D_s, D_p, K_m, R_m, L_m,K], ...
       [QUBE_params.J_p, QUBE_params.J_s, QUBE_params.m_p, QUBE_params.l_p, QUBE_params.l_s, QUBE_params.g, QUBE_params.D_s, QUBE_params.D_p, QUBE_params.K_m, QUBE_params.R_m, QUBE_params.L_m, QUBE_params.K]));
B_bal_num = double(subs(B, [J_p, J_s, m_p, l_p, l_s, g, D_s, D_p, K_m, R_m, L_m,K], ...
       [QUBE_params.J_p, QUBE_params.J_s, QUBE_params.m_p, QUBE_params.l_p, QUBE_params.l_s, QUBE_params.g, QUBE_params.D_s, QUBE_params.D_p, QUBE_params.K_m, QUBE_params.R_m, QUBE_params.L_m, QUBE_params.K]));
C_bal_num = double(C);

disp('Substituting in the identified system parameters, we have the reduced numeric state-space matrices:');
disp('A = '); disp(A_bal_num);
disp('B = '); disp(B_bal_num);
disp('C = '); disp(C_bal_num);


disp("==========================================================================")
disp("Discretization of the system")
disp("==========================================================================")


delta_t=0.01; % Sample time for discretization
disp("The linear system discretized is given by")
Ad=expm(A_bal_num*delta_t)
rank(A_bal_num);
Bd=inv(A_bal_num)*(expm(A_bal_num*delta_t)-eye(length(A_bal_num)))*B_bal_num;
Cd=C_bal_num;
% Vérification des matrices %
sysc = ss(A_bal_num,B_bal_num,C_bal_num,zeros(size(B_bal_num,2)));    % matrices continues
sysd = c2d(sysc, delta_t, 'zoh');
Ad = sysd.A;
Bd = sysd.B;
Co = ctrb(Ad,Bd);
rank_control=rank(Co)  % Controllability test 
Ob = obsv(Ad,Cd);
rank_obs=rank(Ob) % Observability test


%% Setting of the discounted cost %%
a=100;
b=10;
c=10;
d=1;

Q=[a 0 0 0; 0 b 0 0; 0 0 c 0; 0 0 0 d];
R=10;


%% Evolution of the spectral radius of A+BK_{gamma} in function of gamma %%
list_gamma=[];
list_stab=[];
for gamma=0:0.001:1
list_gamma=[list_gamma, gamma];
[K_gamma,P_gamma]=dlqr(sqrt(gamma)*Ad,sqrt(gamma)*Bd,Q,R);
rho=max(abs(eig(Ad-Bd*K_gamma)));
list_stab=[list_stab, rho];
end
list_RS=ones(size(list_gamma));
plot(list_gamma,list_stab, 'b', 'LineWidth',2)
hold on 
plot(list_gamma, list_RS, 'r--','LineWidth',2)
xlabel('$\gamma$', 'Interpreter', 'latex');
ylabel('$\rho(A+BK_{\gamma})$', 'Interpreter', 'latex')
set(gca, 'FontSize', 14);
set(gca,'TickLabelInterpreter','latex')
grid on

y1=list_stab;
y2=list_RS;
x=list_gamma;
below = y2 <= y1;

% On remplit seulement là où la condition est vraie
fill([x(below) fliplr(x(below))], [0*y1(below) fliplr(y1(below))], ...
     [1, 0.8, 0.6], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
legend('', '', '$A_d+B_dK_{\gamma}$ not Schur','Interpreter', 'latex');
ylim([0.95 1.15])

%% Experiment 1 - upper-bound on the cost (gamma=0.5 and alpha=0.98 -> optimal nonstabilizing)%%
gamma=0.5; alpha=0.98;
sa=sqrt(gamma);
Ag=sa*Ad; Bg=sa*Bd;
[K,Popt,CLP] = dlqr(Ag,Bg,Q,R);
K=-K; % Optimal control gain for gamma=0.95
n=size(Ad,1); 
m=size(Bd,2);
Ce=[sqrtm(Q);zeros(1,4)]; De=[zeros(4,1); sqrt(R)'];
Ce'*De % Need to be equal to 0 to satisfy SA2bis
epsilon=1e-6;

yalmip('clear');
ops = sdpsettings('solver','mosek','verbose',1,'debug',1); 
X = sdpvar(n);
Z = sdpvar(n);
G = sdpvar(n,n,'full');
Y = sdpvar(m,n);
W = sdpvar(n);
LMI1 = [W eye(n);eye(n) X];
LMI2 = [G+G'-X sa*G'*Ad'+sa*Y'*Bd' G'*Ce'+Y'*De';
            sa*Ad*G+sa*Bd*Y X zeros(n,n+m)
           Ce*G+De*Y zeros(n+m,n) eye(n+m)]; %Sous-optimalité
LMI3 = [alpha^2*(G+G'-Z) (Ad*G+Bd*Y)';
            Ad*G+Bd*Y Z]; %Stabilité 
LMI = [LMI1>=0*epsilon*eye(2*n),LMI2>=0*epsilon*eye(3*n+m),LMI3>=1*epsilon*eye(2*n)]; %
Ob = trace(W);
sol = optimize(LMI,Ob,ops);
disp(sol.info)     % Message clair de YALMIP
disp(sol.problem)  % Code numérique YALMIP
check(LMI)
Xv = value(X);
Gv = value(G);
Zv = value(Z);
Yv = value(Y);
Wv = value(W);
Kv = Yv*inv(Gv);
EIG_ABKv=eig(Ad+Bd*Kv)

%% Experiment 2 - Control gain close to the optimal one (gamma=0.5, alpha=0.95) %%
alpha=0.95; epsilon=1e-6;
yalmip('clear');
ops = sdpsettings('solver','mosek','verbose',1,'debug',1); 
S = sdpvar(n,n,'full');
Z = sdpvar(n);
T = sdpvar(m,n);
M = sdpvar(1);
LMI1=M;
LMI2 = [S+S'-M*eye(n) (T-K*S)';T-K*S eye(m)];
LMI3 = [alpha^2*(S+S'-Z) (Ad*S+Bd*T)';
        Ad*S+Bd*T Z];
LMI = [LMI1<=1e10,LMI2>=1*epsilon*eye(m+n), LMI3>=epsilon*eye(2*n)];   % We can add also M>=0
Ob = -M;
sol = optimize(LMI,Ob,ops);
Sv = value(S);
Tv = value(T);
Zv=value(Z);
Mv=value(M);
Ktv = Tv*inv(Sv);
EIG_ABKtv=eig(Ad+Bd*Ktv)

% Set time before controller takes action
calib_time = 5;

% Sampling period for discrete-time observer/controller pair
delta = 0.002;