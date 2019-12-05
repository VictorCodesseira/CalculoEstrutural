function out = generateB()
  syms B 
  
  syms z kappa derivZ deriv2Z

  B = generateDel()*generateA();

  for row=1:size(B,1)
    for col=1:size(B,2)
      d1 = diff(B(row,col), derivZ);
      d2 = diff(B(row,col), deriv2Z);
      B(row, col) = B(row, col) - derivZ*d1 - deriv2Z*d2 + diff(d1,z) + diff(d2,z,2);
    end
  end

  out = B;
end

