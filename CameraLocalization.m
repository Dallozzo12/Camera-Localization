% PROBABILISTIC ROBOTICS PROJECT - 3D CAMERA LOCALIZATION

clear all; close all; clc

%Import the workspace
load('Workspace.mat')

%Robot initial pose
init_pose = eye(4);
%rot_init = quat2rotm([0.01 0 0.10 0.1]);
%tranls_init = [0.1 0.001 0];
%init_pose = [rot_init(1,:), tranls_init(1); rot_init(2,:), tranls_init(2); rot_init(3,:), tranls_init(3); 0 0 0 1];

for i=1:length(RobotPose)
    rot=quat2rotmat([RobotPose(i,5) RobotPose(i,6) RobotPose(i,7) RobotPose(i,8)]);
    Robot_Pose_GroundTruth(:,:,i) = [rot(1,:), RobotPose(i,2); rot(2,:), RobotPose(i,3); rot(3,:), RobotPose(i,4); 0 0 0 1];
end

Robot_Pose_Guess(:,:,1)=init_pose; % robot poses after odometry is applied
Robot_Pose_Adjusted(:,:,1)=init_pose; % robot poses after ICP
transf(:,:,1)=init_pose; % is the homogeneous matrix of the odometry
transf_odom_camera(:,:,1) = init_pose;
cumulative_error = [];

rot_camera = quat2rotm([CameraCalibration(5) CameraCalibration(6) CameraCalibration(7) CameraCalibration(8)]);
transf_camera = [rot_camera(1,:),CameraCalibration(2);rot_camera(2,:),CameraCalibration(3);rot_camera(3,:),CameraCalibration(4); 0 0 0 1];

% ICP 3D
for i=2:length(Start_End_PoseID)
    %land = [];
    odom_trasl = Odometry(i,1:3);
    odom_rot = quat2rotmat([Odometry(i,4) Odometry(i,5) Odometry(i,6) Odometry(i,7)]);
    transf(:,:,i) = [odom_rot(1,:),odom_trasl(1);odom_rot(2,:),odom_trasl(2);odom_rot(3,:),odom_trasl(3); 0 0 0 1]; %*transf(:,:,i); % Odometry transformation
    Robot_Pose_Guess(:,:,i) = Robot_Pose_Guess(:,:,i-1)*transf(:,:,i);
    
    new_id = Start_End_PoseID(i-1,2);
    
    onepose_indeces = find(poseID_obsLandmID_UVD(:,1)==new_id); %get the current pose indeces in the observations vector.
    land_ids = poseID_obsLandmID_UVD(onepose_indeces,2); % get the ids of the landmarks observed from that position ID.
    BackProjectedLandmarks = backproject(poseID_obsLandmID_UVD(onepose_indeces,3:end),CameraCalibration); % backproject landmarks from camera to world in camera reference frame.
    
    BackProjectedLandmarks = [land_ids BackProjectedLandmarks]
    [~,landm_pos] = ismember(land_ids,Landmark_GroundTruth(:,1)); % get the position in the world frame of the observed landmarks.
    Landmarks_InWorld = Landmark_GroundTruth(landm_pos,1:end);

    Image_Landmarks = [(poseID_obsLandmID_UVD(onepose_indeces,3)./poseID_obsLandmID_UVD(onepose_indeces,5)), (poseID_obsLandmID_UVD(onepose_indeces,4)./poseID_obsLandmID_UVD(onepose_indeces,5))]'; % Transform observations from camera to image.
    
    Robot_Pose_Guess(:,:,i);
    
    %[Robot_Pose_Adjusted(:,:,i), cumulative_error(:,i)]=ICP(Robot_Pose_Guess(:,:,i),Landmarks_InWorld(:,2:end)',Image_Landmarks, 3, 0.001, 10e-6, H_odom(4:6,4:6), CameraCalibration);
    [Robot_Pose_Adjusted(:,:,i), cumulative_error(:,i)]=ICP(Robot_Pose_Guess(:,:,i),Landmarks_InWorld(:,2:end)',BackProjectedLandmarks(:,2:end)', 3, 0.001, 10e-6, H_odom(4:6,4:6), CameraCalibration);
    
    % qua fai i plot
    
    Robot_Pose_Guess(:,:,i)
    rot2eul(Robot_Pose_Guess(1:3,1:3,i))
    Robot_Pose_Adjusted(:,:,i)
    rot2eul(Robot_Pose_Adjusted(1:3,1:3,i))
    Robot_Pose_GroundTruth(:,:,i)
    rot2eul(Robot_Pose_GroundTruth(1:3,1:3,i))
    
    rot_groundtruth = rot2eul(Robot_Pose_GroundTruth(1:3,1:3,i));
    rot_adjusted = rot2eul(Robot_Pose_Adjusted(1:3,1:3,i));
    rot_guess = rot2eul(Robot_Pose_Guess(1:3,1:3,i));
    
    Robot_Pose_Guess(:,:,i) = Robot_Pose_Adjusted(:,:,i);
    
    
end

%coord_adj = zeros(3,size(Robot_Pose_Adjusted,3)-1);
%coord_real = zeros(3,size(Robot_Pose_Adjusted,3)-1);

Rz = [0 1 0; -1 0 0; 0 0 -1];

for h=2:size(Robot_Pose_Adjusted,3)
    coord_adj(:,h-1) = Rz*Robot_Pose_Adjusted(1:3,4,h);
    coord_real(:,h-1) = Robot_Pose_GroundTruth(1:3,4,h);
end

figure
plot3(coord_adj(1,:),coord_adj(2,:),coord_adj(3,:),'.','MarkerSize',20)
hold on
plot3(coord_real(1,:),coord_real(2,:),coord_real(3,:),'.','MarkerSize',20)
grid


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% clear all; close all; clc
% 
% % Import the workspace
% load('Workspace.mat')
% 
% % Robot initial pose
% init_pose = eye(4);
% 
% Camera_Pose_Guess(:,:,1)=init_pose; % robot poses after odometry is applied
% Camera_Pose_Adjusted(:,:,1)=init_pose;
% Camera_Pose_GroundTruth = [];
% transf(:,:,1)=init_pose; % is the homogeneous matrix of the odometry
% transf_odom_camera(:,:,1) = init_pose;
% cumulative_error = [];
% land = [];
% 
% rot_camera = quat2rotmat([CameraCalibration(8),CameraCalibration(5),CameraCalibration(6),CameraCalibration(7)]);
% transf_camera = [rot_camera(1,:),CameraCalibration(2);rot_camera(2,:),CameraCalibration(3);rot_camera(3,:),CameraCalibration(4); 0 0 0 1];
% 
% for i=1:length(RobotPose)
%     rot=quat2rotmat([RobotPose(i,5),RobotPose(i,6),RobotPose(i,7) RobotPose(i,8)]);
%     Robot_Pose_GroundTruth(:,:,i) = [rot(1,:), RobotPose(i,2); rot(2,:), RobotPose(i,3); rot(3,:), RobotPose(i,4); 0 0 0 1];
%     Camera_Pose_GroundTruth(:,:,i) = Robot_Pose_GroundTruth(:,:,i)*transf_camera;
% end
% 
% %% ICP 2D
% for i=2:2 %length(Start_End_PoseID)
% 	odom_trasl = Odometry(i,1:3);
% 	odom_rot = quat2rotmat([Odometry(i,4) Odometry(i,5) Odometry(i,6) Odometry(i,7)]);
% 	transf(:,:,i) = [odom_rot(1,:),odom_trasl(1);odom_rot(2,:),odom_trasl(2);odom_rot(3,:),odom_trasl(3); 0 0 0 1]; %*transf(:,:,i); % odometry transformation
% 	transf_odom_camera(:,:,i) = transf(:,:,i)*transf_camera;
% 	Camera_Pose_Guess(:,:,i) = Camera_Pose_Guess(:,:,i-1)*transf_odom_camera(:,:,i);
% 
% new_id = Start_End_PoseID(i-1,2);
% 
% onepose_indeces = find(poseID_obsLandmID_UVD(:,1)==new_id); %get the current pose indeces in the observations vector.
% land_ids = poseID_obsLandmID_UVD(onepose_indeces,2); % get the ids of the landmarks observed from that position ID.
% 
% [~,landm_pos] = ismember(land_ids,Landmark_GroundTruth(:,1)); % get the position in the world frame of the observed landmarks.
% Landmarks = [Landmark_GroundTruth(landm_pos,2:end),ones(length(landm_pos),1)]'; % Observed Landmarks in the world.
% 
% % for j=1:size(Landmarks,2)
% % 	Landmarks_In_Curr_Camera_Frame(:,j) = inv(transf_odom_camera(:,:,i))*Landmarks(:,j);   % transform the landmarks in the current camera frame.
% % end
% 
% % ProjectedLandmarks = project_land(Landmarks_In_Curr_Camera_Frame(1:end-1,:),CameraCalibration); % project landmarks from world to camera.
% % ProjectedLandmarks
% 
% Image_Landmarks = [(poseID_obsLandmID_UVD(onepose_indeces,3)./poseID_obsLandmID_UVD(onepose_indeces,5)), (poseID_obsLandmID_UVD(onepose_indeces,4)./poseID_obsLandmID_UVD(onepose_indeces,5))]'; % Transform observations from camera to image.
% 
% % Camera_Pose_Guess(:,:,i)
% %
% %[error, Camera_Pose_Adjusted(:,:,i)] = ICP(Camera_Pose_Guess(:,:,i), Landmarks, Image_Landmarks, CameraCalibration);
% [Camera_Pose_Adjusted(:,:,i), cumulative_error(:,i)]=ICP2D(Camera_Pose_Guess(:,:,i), Landmarks, Image_Landmarks, CameraCalibration);
% 
% Camera_Pose_Adjusted(:,:,i)
% Camera_Pose_GroundTruth(:,:,i)
% 
% Camera_Pose_Guess(:,:,i)=Camera_Pose_Adjusted(:,:,i);
% 
% end
