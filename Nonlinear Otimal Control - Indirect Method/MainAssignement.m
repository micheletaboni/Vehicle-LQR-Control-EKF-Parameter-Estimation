clear all
close all
clc


% Parameters Vehicle:
mass = 1; %kg
Cd = 0.3; 

% Parameters Obstacle:
xc = 0.3;
yc = 0.4;
r = 0.2;

%Parameters Penalty Term:
alpha = 40; %amplifica penalità
sigma = 0.00001;

% Initial and final time
t0 = 0; 
tf = 5;

%Initial States:
x_i = 0;
y_i = 0;
v_i = 0;
theta_i = 0;
a_i = 0;

z_i = [x_i; y_i; v_i; theta_i; a_i];

%Final States:
x_f = 1;
y_f = 1;
v_f = 0;
theta_f = pi/3;
a_f = 0;

z_f = [x_f; y_f; v_f; theta_f; a_f];


% weight for the control
R = [0.1, 0; 0, 0.1]; %per ora inizio così poiche voglio che acceleri piu di quanto giri per evitare mal di macchina

% weight for the velocity
beta =  0.01;

% weight the final state
P = diag([2000, 2000, 1000, 50, 2000]);


% Time interval
Nsegment = 4000;  % Number of time intervals considered
Tu = linspace(t0, tf, Nsegment)';  % discretize time


%% Iterative Procedure

n = 5; %number of states
m = 2; %number of control actions

options = odeset('RelTol', 1e-3, 'AbsTol',[1e-3 1e-3 1e-3 1e-3 1e-3]);

Nmax = 20000;                        % Maximum number of iterations
%u = 0.1*ones(Nsegment,m);               % guessed initial control  u = 1
u = zeros(Nsegment,m);

step = 0.000001;                       % speed of control adjustment
% step = 0.00005;                       % speed of control adjustment
eps = 1e-2;                         % Exit tollerance condition




for ii = 1:Nmax
   ii
   % 1) start with assumed control u and move forward
   [Tz, Z] = ode45(@(t,z) state_equation(t,z,u,Tu,mass,Cd,n), [t0 tf], z_i, options);
   
   % 2) Move backward to get the adjoint vectos trajectory
   Z_tf = Z(end, :).';
   lambda_tf = P*(Z_tf - z_f);
   
   [Tlmb,lmb] = ode45(@(t,lambda) adjoint_equation(t,lambda,u,Tu,Z,Tz,mass,Cd,n, alpha, sigma,xc, yc, beta, r), [tf t0], lambda_tf, options);

   lmb4 = lmb(:,4);
   lmb4 = interp1(Tlmb,lmb4,Tz);

   lmb5 = lmb(:,5);
   lmb5 = interp1(Tlmb,lmb5,Tz);
   % Important: the adjoint vector is stored in reverse order. The dimension of
   % the adjoint vector may also be different from the dimension of states
   % Use interploate to make sure x and lmb is aligned along the time axis
   
   % 3) Calculate deltaH with x1(t), x2(t), lmb1(t), lmb2(t)
   dH = optimality_condition(lmb4,lmb5,Tz,u,Tu,m,mass, R);
   H_Norm = max(abs(dH(:)));
   
   % 4) Calculate the cost
   part1 = beta*trapz(Tz, Z(:, 3).^3);
   part2 = alpha*trapz(Tz, exp( (r^2 - (Z(:,1) - xc).^2 - (Z(:,2) - yc).^2 ) ./ sigma));
   part3 = 0.5*R(1,1)*trapz(Tu, u(:, 1).^2);
   part4 = 0.5*R(2,2)*trapz(Tu, u(:, 2).^2);
   J(ii,1) = part1 + part2 + part3 + part4 + 0.5*(Z_tf - z_f).'*P*(Z_tf - z_f);


   % clf(FigTag)
   % ax = axes;
   % 
   % h1 = scatter(th,th_p,2,Tu,'LineWidth',1); hold on; box on;
   % colormap jet; h1.LineWidth = 1;
   % caxis([0,Tu(end)]);
   % xlim([-pi,pi]);
   % ylim([-5*pi,5*pi]);
   % 
   % h2 = scatter(x_i(3),x_i(2),3,'LineWidth',1); hold on; box on;
   % h2.MarkerEdgeColor = 'b'; h2.LineWidth = 1;
   % h3 = scatter(x_f(3),x_f(2),3,'LineWidth',1); hold on; box on;
   % h3.MarkerEdgeColor = 'r'; h3.LineWidth = 1;
   % 
   % xlabel('$\theta$ [rad]','Interpreter','LaTex');
   % ylabel('$\dot\theta$ [rad/s]','Interpreter','LaTex');
   % ax.FontSize = 16;
   % ax.TickLabelInterpreter = 'LaTex';
   % 
   % drawnow
   
   % 5) if dH/du < epsilon, exit
   if H_Norm < eps
       % Display final cost
       disp(['Final cost: ',num2str(J(ii,1)),' [-]'])
       break;
   elseif J(ii,1) > 1000000
       break;
   else
       % 6) adjust control for next iteration
       u_old = u;
       u = control_iteraction(dH,Tz,u,Tu,step,m);
   end
end

disp(['Final cost: ',num2str(J(ii,1)),' [-]'])
disp(' ')


%% Graph

figure('Name', 'Optimal Trajectory', 'Color', 'w', 'Position', [100, 100, 800, 600]);
hold on; grid on; box on;
axis equal; 
theta_c = linspace(0, 2*pi, 100);
fill(xc + r*cos(theta_c), yc + r*sin(theta_c), [1 0.4 0.4], ...
    'EdgeColor', 'r', 'LineWidth', 1.5, 'DisplayName', 'Obstacle Area');

plot(Z(:,1), Z(:,2), 'b-', 'LineWidth', 2.5, 'DisplayName', 'Optimal Trajectory');

plot(z_i(1), z_i(2), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 10, 'DisplayName', 'Start');
plot(z_f(1), z_f(2), 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 10, 'DisplayName', 'Target');

xlabel('X Position [m]', 'Interpreter', 'latex', 'FontSize', 16);
ylabel('Y Position [m]', 'Interpreter', 'latex', 'FontSize', 16);
title('\textbf{Trajectory Optimization with Obstacle}', 'Interpreter', 'latex', 'FontSize', 18);
legend('Location', 'best', 'Interpreter', 'latex', 'FontSize', 14);
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
hold off;


figure('Name', 'Cost Convergence', 'Color', 'w', 'Position', [950, 100, 600, 400]);
iter_plot = 1:length(J);
semilogy(iter_plot, J, 'r-', 'LineWidth', 2.5, 'DisplayName', 'Cost J');

hold on; grid on; box on;
grid minor;

xlabel('Iteration', 'Interpreter', 'latex', 'FontSize', 16);
ylabel('Total Cost $\log_{10}(J)$', 'Interpreter', 'latex', 'FontSize', 16);
title('\textbf{Convergence History}', 'Interpreter', 'latex', 'FontSize', 18);
legend('Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 14);
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
hold off;





figure('Name', 'System Evolution Overview', 'Color', 'w', 'Position', [100, 100, 900, 600]);
hold on; grid on; box on;


p1 = plot(Tz, Z(:,1), 'b-',  'LineWidth', 2, 'DisplayName', '$x$ Position');
p2 = plot(Tz, Z(:,2), 'b--', 'LineWidth', 2, 'DisplayName', '$y$ Position');
p3 = plot(Tz, Z(:,3), 'g-',  'LineWidth', 2, 'DisplayName', 'Velocity $v$');
p4 = plot(Tz, Z(:,4), 'g--', 'LineWidth', 2, 'DisplayName', 'Heading $\phi$');
p5 = plot(Tz, Z(:,5), 'k-',  'LineWidth', 2, 'DisplayName', 'Acceleration $a$');


p6 = plot(Tu, u(:,1), 'r-',  'LineWidth', 1.5, 'DisplayName', '$u_1$ (Jerk)');
p7 = plot(Tu, u(:,2), 'm-',  'LineWidth', 1.5, 'DisplayName', '$u_2$ (Steering rate)');


plot(tf*ones(5,1), z_f, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', ...
    'HandleVisibility', 'off'); 
plot(NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Targets ($z_f$)');


xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Value (Units vary)', 'Interpreter', 'latex', 'FontSize', 14);
title('\textbf{Complete System Evolution: States \& Controls}', 'Interpreter', 'latex', 'FontSize', 16);


legend('Location', 'northeastoutside', 'Interpreter', 'latex', 'FontSize', 11);

set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');
hold off;
