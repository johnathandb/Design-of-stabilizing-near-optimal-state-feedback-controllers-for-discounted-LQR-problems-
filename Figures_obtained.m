%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Experience 1  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
    T=load("reduced_order_model\Experience_1\qube_bal_control_discrete.mat");

time = T.outputs(1,:)';  % we recover the sample times
phi_ref = T.outputs(2,:)'*(180/pi); % we recover the value of the reference for phi at each time step (in degree)
phi_model = T.outputs(3,:)'*(180/pi); % we recover the real value for phi measured

% ----------------------------------------------------- %
%          Plots   obtained  for Experiment 1           %
% ----------------------------------------------------- %
figure(1)
subplot(2,1,1)
hold on
plot(time, phi_ref, 'r--', 'LineWidth', 2)
plot(time, phi_model, 'b', 'LineWidth', 2)
t_dist=[14.574, 15.426]; % Approximate time of the disturbances that we introduced (see video '')
    for k = 1:length(t_dist)
    xline(t_dist(k),'--','Color',[0.4 0.4 0.4],'LineWidth',1.8)
    end
xlabel('time [s]', 'Interpreter', 'latex');
ylabel('balance error $\delta_{\phi}$ [deg]', 'Interpreter', 'latex')
set(gca, 'FontSize', 14);
set(gca,'TickLabelInterpreter','latex')
grid on
xlim([0 22])
ylim([-180 25])
hold off
legend('reference balance error $\delta_{\phi,ref}$', 'balance error mesured $\delta_{\phi}$', 'manual disturbance', 'Interpreter', 'latex', 'Location', 'southeast');


subplot(2,1,2)
hold on
plot(time, phi_ref, 'r--', 'LineWidth', 2)
plot(time, phi_model, 'b', 'LineWidth', 2)
    for k = 1:length(t_dist)
    xline(t_dist(k),'--','Color',[0.4 0.4 0.4],'LineWidth',1.8)
    end
xlabel('time [s]', 'Interpreter', 'latex');
ylabel('balance error $\delta_{\phi}$ [deg]', 'Interpreter', 'latex')
legend('reference balance error $\delta_{\phi,ref}$', 'balance error mesured $\delta_{\phi}$', 'Interpreter', 'latex', 'Location', 'northwest');
set(gca, 'FontSize', 14);
set(gca,'TickLabelInterpreter','latex')
grid on
xlim([5 22]) % we focus on t>=5 (when the controller is implemented and do the regulation)
ylim([-15 25]) % Zoom on the balance error
hold off
legend('reference balance error $\delta_{\phi,ref}$', 'balance error mesured $\delta_{\phi}$', 'manual disturbance', 'Interpreter', 'latex', 'Location', 'northwest');


%%%%%%%%%%%%%%%%% Experience 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
    T=load("reduced_order_model\Experience_2\qube_bal_control_discrete.mat");

time = T.outputs(1,:)';  % récupère les instants de mesures
phi_ref = T.outputs(2,:)'*(180/pi); % récupère phi reél
phi_model = T.outputs(3,:)'*(180/pi); % récupère phi modèle

% ----------------------------------------------------- %
%        Plots   obtained  for Experiment 2             %
% ----------------------------------------------------- %

figure(2)
subplot(2,1,1)
hold on
plot(time, phi_ref, 'r--', 'LineWidth', 2)
plot(time, phi_model, 'b', 'LineWidth', 2)
t_dist=[13.078, 14.674, 16.352]; % Approximate time of the disturbances that we introduced (see video '')
    for k = 1:length(t_dist)
    xline(t_dist(k),'--','Color',[0.4 0.4 0.4],'LineWidth',1.8)
    end
xlabel('time [s]', 'Interpreter', 'latex');
ylabel('balance error $\delta_{\phi}$ [deg]', 'Interpreter', 'latex')
set(gca, 'FontSize', 14);
set(gca,'TickLabelInterpreter','latex')
grid on
xlim([0 22])
ylim([-180 25])
hold off
legend('reference balance error $\delta_{\phi,ref}$', 'balance error mesured $\delta_{\phi}$', 'manual disturbance', 'Interpreter', 'latex', 'Location', 'southeast');

subplot(2,1,2)
hold on
plot(time, phi_ref, 'r--', 'LineWidth', 2)
plot(time, phi_model, 'b', 'LineWidth', 2)
    for k = 1:length(t_dist)
    xline(t_dist(k),'--','Color',[0.4 0.4 0.4],'LineWidth',1.8)
    end
xlabel('time [s]', 'Interpreter', 'latex');
ylabel('balance error $\delta_{\phi}$ [deg]', 'Interpreter', 'latex')
legend('reference balance error $\delta_{\phi,ref}$', 'balance error mesured $\delta_{\phi}$', 'Interpreter', 'latex', 'Location', 'northwest');
set(gca, 'FontSize', 14);
set(gca,'TickLabelInterpreter','latex')
grid on
xlim([5 22]) % we focus on t>=5 (when the controller is implemented and do the regulation)
ylim([-15 25]) % Zoom on the balance error
hold off
legend('reference balance error $\delta_{\phi,ref}$', 'balance error mesured $\delta_{\phi}$', 'manual disturbance', 'Interpreter', 'latex', 'Location', 'northwest');