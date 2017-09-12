% PROBABILISTIC ROBOTICS PROJECT - 3D CAMERA LOCALIZATION

clear all; close all; clc

%Import the workspace
load('/home/dallo-pc/Desktop/LA SAPIENZA/SECOND YEAR/FIRST SEMESTER/PROBABILISTIC ROBOTICS/Project/Code/Workspace.mat')

%Robot initial pose
init_pose = eye(4);
%rot_init = quat2rotm([0.01 0 0.10 0.1]);
%tranls_init = [0.1 0.001 0];
%init_pose = [rot_init(1,:), tranls_init(1); rot_init(2,:), tranls_init(2); rot_init(3,:), tranls_init(3); 0 0 0 1];

for i=1:length(RobotPose)
    rot=quat2rotm([RobotPose(i,8),RobotPose(i,5),RobotPose(i,6),RobotPose(i,7)]);
    Robot_Pose_GroundTruth(:,:,i) = [rot(1,:), RobotPose(i,2); rot(2,:), RobotPose(i,3); rot(3,:), RobotPose(i,4); 0 0 0 1];
end

Robot_Pose_Guess(:,:,1)=init_pose; % robot poses after odometry is applied
Robot_Pose_Adjusted(:,:,1)=init_pose;
transf(:,:,1)=init_pose; % is the homogeneous matrix of the odometry
transf_odom_camera(:,:,1) = init_pose;
cumulative_error = [];
land = [];

rot_camera = quat2rotm([CameraCalibration(8),CameraCalibration(5),CameraCalibration(6),CameraCalibration(7)]);
transf_camera = [rot_camera(1,:),CameraCalibration(2);rot_camera(2,:),CameraCalibration(3);rot_camera(3,:),CameraCalibration(4); 0 0 0 1];

% ICP 3D
for i=1:1 %length(Start_End_PoseID)
    land = [];
    odom_trasl = Odometry(i,1:3);
    odom_rot = quat2rotm([Odometry(i,7) Odometry(i,4) Odometry(i,5) Odometry(i,6)]);
    transf(:,:,i+1) = [odom_rot(1,:),odom_trasl(1);odom_rot(2,:),odom_trasl(2);odom_rot(3,:),odom_trasl(3); 0 0 0 1]*transf(:,:,i); % Odometry transformation
    Robot_Pose_Guess(:,:,i+1) = Robot_Pose_Guess(:,:,i)*transf(:,:,i+1);
    
    new_id = Start_End_PoseID(i,2);
    
    onepose_indeces = find(poseID_obsLandmID_UVD(:,1)==new_id); %get the current pose indeces in the observations vector.
    land_ids = poseID_obsLandmID_UVD(onepose_indeces,2); % get the ids of the landmarks observed from that position ID.
    BackProjectedLandmarks = backproject(poseID_obsLandmID_UVD(onepose_indeces,3:end),CameraCalibration); % backproject landmarks from camera to world in camera reference frame.
        
    %size(BackProjectedLandmarks)
    %BackProjectedLandmarks
    
    %for j=1:size(BackProjectedLandmarks,2)
    %    land(:,j)=inv(transf(:,:,i+1))*[BackProjectedLandmarks(:,j); 1];
    %    % bring backprojected landmarks from the current camera frame back to the world frame!!!
    %end

    %land = land(1:end-1,:);
    BackProjectedLandmarks = [land_ids BackProjectedLandmarks]
    [~,landm_pos] = ismember(land_ids,Landmark_GroundTruth(:,1)); % get the position in the world frame of the observed landmarks.
    Landmarks_InWorld = Landmark_GroundTruth(landm_pos,1:end)
    
    %Landmarks_InWorld
    %BackProjectedLandmarks
    
    Robot_Pose_Guess(:,:,i+1);
    
    [Robot_Pose_Adjusted(:,:,i+1), cumulative_error(:,i)]=ICP(inv(Robot_Pose_Guess(:,:,i+1)),Landmarks_InWorld(:,2:end)',BackProjectedLandmarks(:,2:end)', 100, 0.01, 10e-9, H_odom(4:6,4:6));
    
    % qua fai i plot
    
    Robot_Pose_Guess(:,:,i+1) = Robot_Pose_Adjusted(:,:,i+1);
    
    Robot_Pose_Adjusted(:,:,i+1)
    Robot_Pose_GroundTruth(:,:,i)
    
end

% clear all; close all; clc
% 
% % Import the workspace
% load('/home/dallo-pc/Desktop/LA SAPIENZA/SECOND YEAR/FIRST SEMESTER/PROBABILISTIC ROBOTICS/Project/Code/Workspace.mat')
% 
% % Robot initial pose
% init_pose = eye(4);
% %rot_init = quat2rotm([0.01 0 0.10 0.1]);
% %tranls_init = [0.1 0.001 0];
% %init_pose = [rot_init(1,:), tranls_init(1); rot_init(2,:), tranls_init(2); rot_init(3,:), tranls_init(3); 0 0 0 1];
% 
% Camera_Pose_Guess(:,:,1)=init_pose; % robot poses after odometry is applied
% Camera_Pose_Adjusted(:,:,1)=init_pose;
% Camera_Pose_GroundTruth = [];
% transf(:,:,1)=init_pose; % is the homogeneous matrix of the odometry
% transf_odom_camera(:,:,1) = init_pose;
% cumulative_error = [];
% land = [];
% 
% rot_camera = quat2rotm([CameraCalibration(8),CameraCalibration(5),CameraCalibration(6),CameraCalibration(7)]);
% transf_camera = [rot_camera(1,:),CameraCalibration(2);rot_camera(2,:),CameraCalibration(3);rot_camera(3,:),CameraCalibration(4); 0 0 0 1];
% 
% for i=1:length(RobotPose)
%     rot=quat2rotm([RobotPose(i,8),RobotPose(i,5),RobotPose(i,6),RobotPose(i,7)]);
%     Robot_Pose_GroundTruth(:,:,i) = [rot(1,:), RobotPose(i,2); rot(2,:), RobotPose(i,3); rot(3,:), RobotPose(i,4); 0 0 0 1];
%     Camera_Pose_GroundTruth(:,:,i) = Robot_Pose_GroundTruth(:,:,i)*transf_camera;
% end
% 
% %% ICP 2D
% for i=1:1 %length(Start_End_PoseID)
% 	odom_trasl = Odometry(i,1:3);
% 	odom_rot = quat2rotm([Odometry(i,7) Odometry(i,4) Odometry(i,5) Odometry(i,6)]);
% 	transf(:,:,i+1) = [odom_rot(1,:),odom_trasl(1);odom_rot(2,:),odom_trasl(2);odom_rot(3,:),odom_trasl(3); 0 0 0 1]*transf(:,:,i); % odometry transformation
% 	transf_odom_camera(:,:,i+1) = transf(:,:,i+1)*transf_camera;
% 	Camera_Pose_Guess(:,:,i+1) = Camera_Pose_Guess(:,:,i)*transf_odom_camera(:,:,i+1);
% 
% new_id = Start_End_PoseID(i,2);
% 
% onepose_indeces = find(poseID_obsLandmID_UVD(:,1)==new_id); %get the current pose indeces in the observations vector.
% land_ids = poseID_obsLandmID_UVD(onepose_indeces,2); % get the ids of the landmarks observed from that position ID.
% 
% [~,landm_pos] = ismember(land_ids,Landmark_GroundTruth(:,1)); % get the position in the world frame of the observed landmarks.
% Landmarks = [Landmark_GroundTruth(landm_pos,2:end),ones(length(landm_pos),1)]'; % Observed Landmarks in the world.
% 
% for j=1:size(Landmarks,2)
% 	Landmarks_In_Curr_Camera_Frame(:,j) = inv(transf_odom_camera(:,:,i+1))*Landmarks(:,j);   % transform the landmarks in the current camera frame.
% end
% 
% ProjectedLandmarks = project_land(Landmarks_In_Curr_Camera_Frame(1:end-1,:),CameraCalibration); % project landmarks from world to camera.
% 
% ProjectedLandmarks
% Image_Landmarks = poseID_obsLandmID_UVD(onepose_indeces,[4,5])'
% 
% Camera_Pose_Guess(:,:,i+1)
% 
% Camera_Pose_Adjusted(:,:,i+1)=ICP2D(Camera_Pose_Guess(:,:,i+1), ProjectedLandmarks, Image_Landmarks)
% 
% %Camera_Pose_Guess(:,:,i+1) = Camera_Pose_Adjusted(:,:,i+1);
% 
% %Camera_Pose_Guess(:,:,i+1)
% %Camera_Pose_Adjusted(:,:,i+1)
% Camera_Pose_GroundTruth(:,:,i+1)
% 
% end



