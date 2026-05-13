function dlambda = adjoint_equation(t,lambda,u,Tu,Z,Tz,mass,Cd,n, alpha, sigma,xc, yc, beta, r, mu, g, eps)

dlambda = zeros(n,1);


%interpoliamo le z rispetto al tempo nuovo che ora diventa una variabile
z4 = interp1(Tz,Z(:,4),t);   % Interploate the state varialbes
z3 = interp1(Tz,Z(:,3),t);   % Interploate the state varialbes
z2 = interp1(Tz,Z(:,2),t);   % Interploate the state varialbes
z1 = interp1(Tz,Z(:,1),t);   % Interploate the state varialbes

%u = interp1(Tu,u,t);     % Interploate the control

dlambda(1) = (2*alpha*(z1 - xc)/sigma)*exp( (r^2 - (z1 - xc)^2 - (z2 - yc)^2 )/sigma );
dlambda(2) =  (2*alpha*(z2 - yc)/sigma)*exp( (r^2 - (z1 - xc)^2 - (z2 - yc)^2 )/sigma );
dlambda(3) =  -3*beta*z3^2 - lambda(1)*cos(z4) - lambda(2)*sin(z4) + 2*lambda(3)*Cd*z3/mass + (lambda(3)*mu*g*sech(z3/eps)^2)/eps; 
dlambda(4) =  lambda(1)*z3*sin(z4) - lambda(2)*z3*cos(z4);
