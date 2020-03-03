function [Ad, Bd, G] = DiscreteK(Q)
R = 1; 
Mw = 0.033;
Mb = 1.083;
Ib = 0.00555;
Iw = 133.85 * 10^-7;
l = 0.0615;
k = Mb^2 * l^2 / (Mb * l^2 + Ib);
g = 9.8;
r = 0.03;
a = Mb + 2 * Mw + 2 * Iw / r^2;
A = [0 1 0 0;
0 0 -k*g/(a-k) 0;
0 0 0 1;
0 0 k*g*a/(Mb * l * (a-k)) 0];

B = [0;
(1/r + k/(Mb*l)) / (a-k);
0;
-k/(Mb*l)^2 - (k/(Mb*l)) * (1/r + k/(Mb*l))/(a-k)];

% x, xDot, theta, thetaDot

csys = ss(A, B, zeros(4, 4), zeros(4, 1));
dsys = c2d(csys, 2*1e-3);
[G, X, L] = dlqr(dsys.A,dsys.B, Q, R);
Ad = dsys.A;
Bd = dsys.B;

% disp(G);

% [K, S, P] = lqr(A, B, Q, R);