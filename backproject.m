function [points] = backproject(imagepoint_coord, camera)
% This function takes as input the pixel coordinates and the depth of a 
% landmark in the image and backprojects it in the world.

points_passed = size(imagepoint_coord,1);
points = [];

camera_matrix = [camera(9) 0 camera(11); 0 camera(10) camera(12); 0 0 1];

%size(imagepoint_coord)
%size(camera_matrix)

for i=1:points_passed
    points(i,:) = inv(camera_matrix)*imagepoint_coord(i,:)';
end

% for i=1:points_passed
% 	points(i,1) = (imagepoint_coord(i,1)-camera(11))*imagepoint_coord(i,3);
% 	points(i,2) = (imagepoint_coord(i,2)-camera(12))*imagepoint_coord(i,3);
% 	points(i,3) = imagepoint_coord(i,3);
% end

%point3D_ = [X,Y,Z,1];
%quat = [camera(8),camera(5:7)];
%R = quat2rotm(quat);
%t = camera(2:4)';
%rot_trasl = [R(1,:),t(1);R(2,:),t(2);R(3,:),t(3); 0,0,0,1];
%point3D = inv(rot_trasl)*point3D_';

%point3D = [X,Y,Z];

end