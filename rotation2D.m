function R=rotation2D(alpha)
  s=sin(alpha);
  c=cos(alpha);
  R=[c -s;
     s  c];
end