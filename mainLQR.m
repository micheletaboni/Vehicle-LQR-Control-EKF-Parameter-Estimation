clear all
close all
clc

global mass Cd mu eps g alpha sigma r xc yc beta

% Parameters Systems:
mass = 1; %kg
Cd = 0.3; 
g = 9.81; %m/s^2
mu = 0.07;
eps = 0.1;

% Parameters Obstacle:
xc = 0.3;
yc = 0.4;
r = 0.15;

%Parameters Penalty Term:
alpha = 50; %amplifica penalità
sigma = 0.01;

% Initial and final time
t0 = 0; 
tf = 5;

%Initial States:
x_i = 0;
y_i = 0;
v_i = 0;
theta_i = 0;


z_i = [x_i; y_i; v_i; theta_i];


%Final States:
x_f = 1;
y_f = 1;
v_f = 0;
theta_f = pi/3;

z_f = [x_f; y_f; v_f; theta_f];

% weight for the control
R = [2, 0; 0, 2]; %per ora inizio così poiche voglio che acceleri piu di quanto giri per evitare mal di macchina

% weight for the velocity
beta =  0.01;

% weight the final state
P = diag([2000, 2000, 1000, 50]);


%% Load Optimal Trajectory

load('Optimal_Trajectory.mat');

u = Optimal_Trajectory.u;
Z = Optimal_Trajectory.Z;
Tu = Optimal_Trajectory.Tu';
Tz = Optimal_Trajectory.Tz';


%% Re-sample the optimal trajectory 

% Time discretization in N = number of time intervals
N = 5000;

Ns = size(Z,2); % number of states
Nu = size(u,2); % number of controls

% Sampled time history
T = linspace(0,Tz(end),N);

% Interpolation/sampling of the state and control 
zk = zeros(N,Ns);
uk = zeros(N,Nu);
for ii = 1:Ns
    zk(:,ii) = interp1(Tz,Z(:,ii),T);
end
for ii = 1:Nu
    uk(:,ii) = interp1(Tu,u(:,ii),T);
end


%% LQR - Elaboration

% Option for ODE solver
VectTol = ones(Ns^2,1)*1e-5;
options = odeset('RelTol', 1e-3, 'AbsTol', 1e-2);

% initial and final time
t0 = T(1);
tf = T(end);

% boundary conditions
p0 = P(1:end)';

% Integration of the matrix riccati equation
[Tp,PP] = ode23(@(t,p) DRE(t,p,R,zk,T,uk,T), flip(T), p0, options);

% From backward to forward dynamics (they are stored in the reversed order)
PP = flipud(PP);
Tp = flipud(Tp);

% Transformation Vector -> Matrix
PP_Matrix = zeros(Ns,Ns);

% Computation of the gain matrix in time, Uncontrolled stability matrix,
% Controlled stability matrix along the trajectory
K = zeros(N,Nu,Ns);
A = zeros(N,Ns,Ns);

Xp = zeros(N,Ns);
for ii = 1:N % Note: try also reshape.m
    % transformation vector -> matrix
    PP_Matrix(1:end) = PP(ii,:)';
    % control matrix G
    B = fu();
    % Uncontrolled state stability matrix
    A(ii,:,:) = fx(zk(ii,:));
    % Gain matrix C
    K(ii,:,:) = R^-1*B'*PP_Matrix; % R has to be non-null
end



%% Extended Kalman Filter:

% weight matrices
% Do I trust my model?
%Qk = diag([0.1, 0.1]); 
Qk = diag([0.1, 0.1, 0.1, 0.1]); 
%Do I trust my measurements?
Rk = diag([10, 10]);

% observation matrix, I can measure only the positions
C = [1 0 0 0;
     0 1 0 0];


%% Extended Kalman Filter - Parameter Estimation:

% weight matrices
% Do I trust my model?
Qk_est = diag([0.01, 0.01, 0.01, 0.01, 0.001]); 

%Do I trust my measurements?
Rk_est = diag([1, 1]);

% observation matrix, I can measure only the positions
C_est = [1 0 0 0 0;
         0 1 0 0 0];

z_i_augmented = [z_i; 0.1];

%P0_est = diag([1 1 25 10 10]);
%P0_est = P0_est(1:end)';
%P0_est = zeros(5*5,1);
P0_est = diag([0 0 0 0 5]);
P0_est = P0_est(1:end)';

%% Simulink Model:

u_opt = timeseries(uk,T);
z_opt = timeseries(zk,T);

%TIME SERIES VUOLE TEMPO ALL'ULTIMA DIMENSIONE SE HO MATRICI 3D
K_correct = permute(K, [2, 3, 1]);
K_values = timeseries(K_correct,T);

amplitude_disturbance = 0.3; %on control

noise_amplitude = 0.05;





%% Graph


%Disturbed state with optimal control: 

figure('Name', 'Optimal Trajectory with Disturbance', 'Color', 'w', 'Position', [100, 100, 800, 600]);
hold on; grid on; box on;
axis equal; 
theta_c = linspace(0, 2*pi, 100);
fill(xc + r*cos(theta_c), yc + r*sin(theta_c), [1 0.4 0.4], ...
    'EdgeColor', 'r', 'LineWidth', 1.5, 'DisplayName', 'Obstacle Area');

plot(zk(:,1), zk(:,2), 'b-', 'LineWidth', 2.5, 'DisplayName', 'Optimal Trajectory');
plot(z_disturbance.data(:,1), z_disturbance.data(:,2), '--b', 'LineWidth', 2.5, 'DisplayName', 'Disturbed Trajectory');

plot(z_disturbance.data(1), z_disturbance.data(2), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 10, 'DisplayName', 'Start');
plot(z_f(1), z_f(2), 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 10, 'DisplayName', 'Target');

xlabel('X Position [m]', 'Interpreter', 'latex', 'FontSize', 16);
ylabel('Y Position [m]', 'Interpreter', 'latex', 'FontSize', 16);
title('\textbf{Trajectory Optimization with Obstacle}', 'Interpreter', 'latex', 'FontSize', 18);
legend('Location', 'best', 'Interpreter', 'latex', 'FontSize', 14);
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
hold off;



figure('Name', 'System Evolution with Disturbance Overview', 'Color', 'w', 'Position', [100, 100, 900, 600]);
hold on; grid on; box on;


p1 = plot(z_disturbance.Time, z_disturbance.data(:,1), '--b',  'LineWidth', 2, 'DisplayName', '$x$ Position DIST');
p2 = plot(z_disturbance.Time, z_disturbance.data(:,2), '--r', 'LineWidth', 2, 'DisplayName', '$y$ Position DIST');
p3 = plot(z_disturbance.Time, z_disturbance.data(:,3), '--g',  'LineWidth', 2, 'DisplayName', 'Velocity $v$ DIST');
p4 = plot(z_disturbance.Time, z_disturbance.data(:,4), '--c', 'LineWidth', 2, 'DisplayName', 'Heading $\phi$ DIST');

p5 = plot(T, zk(:,1), 'b',  'LineWidth', 2, 'DisplayName', '$x$ Position OPT');
p6 = plot(T, zk(:,2), 'r', 'LineWidth', 2, 'DisplayName', '$y$ Position OPT');
p7 = plot(T, zk(:,3), 'g',  'LineWidth', 2, 'DisplayName', 'Velocity $v$ OPT');
p8 = plot(T, zk(:,4), 'c', 'LineWidth', 2, 'DisplayName', 'Heading $\phi$ OPT');


plot(tf*ones(4,1), z_f, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', ...
    'HandleVisibility', 'off'); 
plot(NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Targets ($z_f$)');


xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Value (Units vary)', 'Interpreter', 'latex', 'FontSize', 14);
title('\textbf{Complete System Evolution: States }', 'Interpreter', 'latex', 'FontSize', 16);
legend('Location', 'northeastoutside', 'Interpreter', 'latex', 'FontSize', 11);
set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');
hold off;



figure('Name', 'Control Action Comparison', 'Color', 'w', 'Position', [100, 100, 900, 600]);
hold on; grid on; box on;

p1 = plot(T, uk(:,1), 'r', 'LineWidth', 2, 'DisplayName', '$u1$ OPT');
p2 = plot(T, uk(:,2), 'g',  'LineWidth', 2, 'DisplayName', '$u2$ OPT');
p3 = plot(u_generated.Time, u_generated.data(:,1), '--r', 'LineWidth', 2, 'DisplayName', '$u1$ GENERATED');
p4 = plot(u_generated.Time, u_generated.data(:,2), '--g',  'LineWidth', 2, 'DisplayName', '$u1$ GENERATED');

xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Value (Units vary)', 'Interpreter', 'latex', 'FontSize', 14);
title('\textbf{Control Action Comparison}', 'Interpreter', 'latex', 'FontSize', 16);


legend('Location', 'northeastoutside', 'Interpreter', 'latex', 'FontSize', 11);

set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');
hold off;




%% Graph: Parameter Estimation (mu_r)

figure('Name', 'Parameter Estimation: Friction Coefficient', 'Color', 'w', 'Position', [100, 100, 800, 400]);
hold on; grid on; box on;

t_est = mu_estimated.Time;
mu_est_history = mu_estimated.data; 


plot(t_est, mu_est_history, 'b', 'LineWidth', 2.5, 'DisplayName', 'Estimated $\mu_r$');

yline(mu, 'r--', 'LineWidth', 2, 'DisplayName', 'True Value ($\mu_r = 0.07$)');

plot(t_est(1), z_i_augmented(end), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8, 'DisplayName', 'Initial Guess (0.1)');

xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Friction Coefficient $\mu_r$', 'Interpreter', 'latex', 'FontSize', 14);
title('\textbf{Parameter Estimation: Friction Coefficient} ($\mu_r$)', 'Interpreter', 'latex', 'FontSize', 16);
legend('Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 12);
set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');


ylim([0.05, 0.12]); 

hold off;