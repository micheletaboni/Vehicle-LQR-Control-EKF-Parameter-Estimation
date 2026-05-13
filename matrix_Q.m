function Q = matrix_Q(x)

global mass Cd mu eps g alpha sigma r xc yc beta

exponential = exp((r^2 - (x(1) - xc)^2 - (x(2) - yc)^2) / sigma);

Q = [-2*alpha*exponential/sigma + (4*alpha*(x(1) - xc)^2)*exponential/sigma^2,   4*alpha*(x(1) - xc)*(x(2) - yc)*exponential/sigma^2,                      0,           0;
      4*alpha*(x(1) - xc)*(x(2) - yc)*exponential/sigma^2,                       -2*alpha*exponential/sigma + (4*alpha*(x(2) - yc)^2)*exponential/sigma^2, 0,           0;
      0,                                                                         0,                                                                        6*beta*x(3), 0;
      0,                                                                         0,                                                                        0,           0];


end