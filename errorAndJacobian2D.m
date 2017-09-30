function [e,J]=errorAndJacobian2D(X,p,z,camera)
% t=X(1:2,3);
% R=X(1:2,1:2);
% z_hat=R*p+t;
% e=z_hat-z;
% J=zeros(2,3);
% J(1:2,1:2)=eye(2);
% J(1:2,3)=[-z_hat(2),z_hat(1)]';

p_camera_frame = inv(X)*p;

% Build the camera matrix
C = [camera(9),0,camera(11); 0,camera(10),camera(12); 0,0,1];

points_cam = C*p_camera_frame(1:end-1);
points_img = [points_cam(1)/points_cam(3); points_cam(2)/points_cam(3)];

z_hat = points_img;
e=z_hat-z;

J_icp=zeros(3,6);
%J_proj=zeros(2,3);

J_proj = [1/points_cam(3) 0 -(points_cam(1)/(points_cam(3))^2); 0 1/points_cam(3) -(points_cam(2)/(points_cam(3))^2)];
J_icp(1:3,1:3)=eye(3);
J_icp(1:3,4:6)=-skew(p_camera_frame);

J = J_proj*C*J_icp;
size(J)

end