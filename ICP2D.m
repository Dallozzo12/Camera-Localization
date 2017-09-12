function [chi, x_new]=ICP2D(X,P,Z)
chi=0;
%cumulative chi2
H=zeros(3,3); %accumulators for H and b
b=zeros(3,1);
for(i=1:size(P,2))
p=P(:,i); z=Z(:,i);
[e,J]=errorAndJacobian2D(X,p,z);
H = H+J'*J;
%assemble H and B
b = b+J'*e;
chi = chi+e'*e;
%update cumulative error
end
dx=(-H\b);
v2t3(dx)
%solve the linear system
X=v2t3(dx)*X;
%apply perturbation
end