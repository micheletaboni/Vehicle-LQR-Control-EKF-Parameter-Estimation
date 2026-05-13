function dz = state_equation(t,z,u,Tu,mass,Cd,n, mu, g, eps)

dz = zeros(n,1);

u2 = interp1(Tu, u(:,2), t);
u1 = interp1(Tu, u(:,1), t);

%dat che u è mxN, interp1 funziona solo se ho una matrice scritta Nxm quindi devo trasporla,ù
%poi con u = interp1(Tu,u,t); mi tira fuori il valore di u a t e quindi poi
%la trapongo ancora per poter avere il vettore colonna delle control
%actions

dz(1) = z(3)*cos(z(4));
dz(2) = z(3)*sin(z(4));
dz(3) = (u1 - Cd*z(3)^2 - mu*mass*g*tanh(z(3)/eps))/mass;
dz(4) = u2;
