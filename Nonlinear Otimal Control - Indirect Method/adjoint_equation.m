function dlambda = adjoint_equation(t,lambda,u,Tu,Z,Tz,mass,Cd,n, alpha, sigma,xc, yc, beta, r)

dlambda = zeros(n,1);


%interpoliamo le z rispetto al tempo nuovo che ora diventa una variabile
z5 = interp1(Tz,Z(:,5),t);   % Interploate the state varialbes
z4 = interp1(Tz,Z(:,4),t);   % Interploate the state varialbes
z3 = interp1(Tz,Z(:,3),t);   % Interploate the state varialbes
z2 = interp1(Tz,Z(:,2),t);   % Interploate the state varialbes
z1 = interp1(Tz,Z(:,1),t);   % Interploate the state varialbes

%u = interp1(Tu,u,t);     % Interploate the control

dlambda(1) = (2*alpha*(z1 - xc)/sigma)*exp( (r^2 - (z1 - xc)^2 - (z2 - yc)^2 )/sigma );
dlambda(2) =  (2*alpha*(z2 - yc)/sigma)*exp( (r^2 - (z1 - xc)^2 - (z2 - yc)^2 )/sigma );
dlambda(3) =  -3*beta*z3^2 - lambda(1)*cos(z4) - lambda(2)*sin(z4) + lambda(5)*2*Cd*z5/mass;
dlambda(4) =  lambda(1)*z3*sin(z4) - lambda(2)*z3*cos(z4);
dlambda(5) =  -lambda(3) + lambda(5)*2*Cd*z3/mass;