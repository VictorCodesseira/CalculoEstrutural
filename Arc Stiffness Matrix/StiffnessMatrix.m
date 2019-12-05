clear all
clc

syms L

diary matrix.txt
B = generateB();
C = generateC();
D = generateD();

syms z

integrando = (B.')*(D)*(B);
integral = int(integrando, z, 0, L);
K = (C.')*(integral)*(C);

K = simplify(K)
diary off