function out = generateDel()
  syms Del
  
  syms z kappa derivZ deriv2Z

  Del(1,1) = -kappa;
  Del(1,3) = derivZ;

  Del(2,2) = -deriv2Z;
  Del(2,6) = kappa;

  Del(3,1) = deriv2Z;
  Del(3,3) = kappa*derivZ;

  Del(4,2) = kappa*derivZ;
  Del(4,6) = derivZ;

  out = Del;
end

