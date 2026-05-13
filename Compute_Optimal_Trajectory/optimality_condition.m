function dH = optimality_condition(lambda4,lambda3,tz,u,Tu,m,mass, R)
% interploate the control

u2 = interp1(Tu, u(:, 2), tz);
u1 = interp1(Tu, u(:, 1), tz); % Interploate the control at time tz

dH = zeros(length(tz), m);


dH(:, 1) = R(1,1).*u1 + lambda3./mass; % L_u + lmb^T*B (see the dynamic equation)
dH(:, 2) = R(2,2).*u2 + lambda4;