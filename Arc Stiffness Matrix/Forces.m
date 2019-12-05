clear all
clc

syms z

B = generateB();
C = generateC();
D = generateD();
Ue = generateUe();

q = C*Ue;
eta = B*q;
F = D*eta;

N = fliplr(coeffs(F(1),z, 'All'))
Vy = fliplr(coeffs(F(2),z, 'All'))
Vz = fliplr(coeffs(F(3),z, 'All'))
T = fliplr(coeffs(F(4),z, 'All'))