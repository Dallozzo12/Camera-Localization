function [points_img] = project_land(world_point,camera)
% This function takes as input the pixel coordinates and the depth of a 
% landmark in the image and backprojects it in the world.

points_passed = size(world_point,2);
points_img = [];

% Build the camera matrix
C = [camera(9),0,camera(11); 0,camera(10),camera(12); 0,0,1];

for i=1:points_passed
	points_cam(:,i) = C*world_point(:,i);
	points_img(:,i) = [point(1,i)/point(3,i); point(2,i)/point(3,i)];
end

%point3D_ = [X,Y,Z,1];
%quat = [camera(8),camera(5:7)];
%R = quat2rotm(quat);
%t = camera(2:4)';
%rot_trasl = [R(1,:),t(1);R(2,:),t(2);R(3,:),t(3); 0,0,0,1];
%point3D = inv(rot_trasl)*point3D_';

%point3D = [X,Y,Z];

end