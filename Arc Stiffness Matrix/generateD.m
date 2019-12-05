function out = generateD()
  syms D  

  syms E A Iz Iy G J

  D(1,1) = E*A;
  D(2,2) = E*Iy;
  D(3,3) = E*Iz;
  D(4,4) = G*J;

  out = D;
end

