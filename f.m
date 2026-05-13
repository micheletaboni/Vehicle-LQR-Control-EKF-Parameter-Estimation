function dz = f(t,z,u,Tu)

global mass Cd mu eps g alpha sigma r xc yc beta


dz = zeros(5,1);

u2 = interp1(Tu, u(:,2), t);
u1 = interp1(Tu, u(:,1), t);

%dat che u è mxN, interp1 funziona solo se ho una matrice scritta Nxm quindi devo trasporla,ù
%poi con u = interp1(Tu,u,t); mi tira fuori il valore di u a t e quindi poi
%la trapongo ancora per poter avere il vettore colonna delle control
%actions

dz(1) = z(3)*cos(z(4));
dz(2) = z(3)*sin(z(4));
dz(3) = z(5);
dz(4) = u2;
dz(5) = u1/mass - 2*Cd*z(3)*z(5)/mass - (mu*g*z(5)*sech(z(3)/eps)^2)/eps;