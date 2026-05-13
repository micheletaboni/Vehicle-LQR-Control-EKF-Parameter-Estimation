function A = fx(z)

global mass Cd mu eps g alpha sigma r xc yc beta 

A = [0             0            cos(z(4))                                                                                 -z(3)*sin(z(4));
     0             0            sin(z(4))                                                                                  z(3)*cos(z(4));
     0             0            -2*Cd*z(3)/mass - (mu*g*sech(z(3)/eps)^2)/eps                                              0;
     0             0            0                                                                                          0];                       