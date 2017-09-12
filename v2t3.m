function T=v2t3(v)
  T=eye(3);
  T(1:2,1:2)=rotation2D(v(3));
  T(1:2,3)=v(1:2);
end