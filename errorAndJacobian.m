function [e,J]=errorAndJacobian(X,p,z,camera)

% p_camera_frame = inv(X)*[p;1];
% p_camera_frame = p_camera_frame(1:3);
% 
% % Build the camera matrix
% C = [camera(9),0,camera(11); 0,camera(10),camera(12); 0,0,1];
% 
% points_cam = C*p_camera_frame;
% points_img = [points_cam(1)/points_cam(3); points_cam(2)/points_cam(3)];
% 
% z_hat = points_img;
% e=z_hat-z;
% 
% J_icp=zeros(3,6);
% %J_proj=zeros(2,3);
% 
% J_proj = [1/points_cam(3) 0 -(points_cam(1)/(points_cam(3))^2); 0 1/points_cam(3) -(points_cam(2)/(points_cam(3))^2)];
% J_icp(1:3,1:3)=eye(3);
% J_icp(1:3,4:6)=-skew(p_camera_frame);
% 
% J = J_proj*C*J_icp;

z_hat=X(1:3,1:3)*p+X(1:3,4); %prediction
e=z_hat-z;
J=zeros(3,6);
J(1:3,1:3)=eye(3);
J(1:3,4:6)=-skew(z_hat);

size(J)

end