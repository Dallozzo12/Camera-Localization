function [e,J]=errorAndJacobian2D(X,p,z)
t=X(1:2,3);
R=X(1:2,1:2);
z_hat=R*p+t;
e=z_hat-z;
J=zeros(2,3);
J(1:2,1:2)=eye(2);
J(1:2,3)=[-z_hat(2),z_hat(1)]';



end