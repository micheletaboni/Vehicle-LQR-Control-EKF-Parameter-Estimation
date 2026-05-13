function u_new = control_iteraction(dH,tz,u,tu,step,m)
% interploate dH/du
u_new = zeros(length(tu), m);

dH1 = interp1(tz,dH(:,1),tu);
dH2 = interp1(tz,dH(:,2),tu);
u_new(:, 1) = u(:, 1) - step.*dH1;
u_new(:, 2) = u(:, 2) - step.*dH2;