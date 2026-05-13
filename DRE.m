function [OutVect] = DRE(t,p,R,Z,Tz,u,Tu)

% evaluation of the state at time @t
Nstates = size(Z,2);
zk = zeros(1,Nstates);
for ii = 1:Nstates
    zk(:,ii) = interp1(Tz,Z(:,ii),t);
end

% evaluation of the control action at time @t
Nu = size(u,2);
uk = zeros(1,Nu);
for ii = 1:Nu
    uk(:,ii) = interp1(Tu,u(:,ii),t);
end

% matrices:
A = fx(zk);
B = fu();


%Q = matrix_Q(zk);

Q = diag([10, 10, 10, 1e-0]);

% transformation vector -> matrix
P = zeros(Nstates,Nstates);
P(1:end) = p(1:end);

% DRE
Out = -(A.'*P + P*A - P*B*R^-1*B.'*P + Q);

% transformation matrix -> vector
OutVect = Out(1:end)';