%% PROBABILISTIC ROBOTICS PROJECT - 3D CAMERA LOCALIZATION

clear all; close all; clc

% Import the workspace
load('/home/dallo-pc/Desktop/LA SAPIENZA/SECOND YEAR/FIRST SEMESTER/PROBABILISTIC ROBOTICS/Project/Code/Workspace.mat')

% Robot initial pose
init_pose = eye(4);




% Define camera poses
curr_cameraPose(:,:,1) = init_pose;  % Will contain the poses predicted via odometry
newcurr_cameraPose(:,:,1) = init_pose;  % Will contain the poses adjusted via ICP

RobotPoses_GroundTruth = []; % Real robot poses (imported from g2o output)
CameraPoses_GroundTruth = []; % Real camera poses (obtained applying a transform to RobotPoses_GroundTruth)
CameraXYZ = [];  % Real camera position (from RobotPoses_GroundTruth)
CameraOrient = [];  % Real camera orientation (from RobotPoses_GroundTruth)
eulers_camera = [];
RobotXYZ = [];  % Real camera position (from RobotPoses_GroundTruth)
RobotOrient = [];  % Real camera orientation (from RobotPoses_GroundTruth)
eulers_robot = [];

% Define structures to store landmarks positions
AllLandmarks_CameraFrame = [];
BackProjectedLandmarks = [];
Land = zeros(1,3);


%% IMPLEMENTATION


% % Build the information matrix related to the odometry from the imported data
% H_odom = zeros(size(Odometry_InfMatrix,2),size(Odometry_InfMatrix,2));
% j=1;
% for i = 1:size(Odometry_InfMatrix,2)
%     if (Odometry_InfMatrix(1,i) ~= 0)
%         H_odom(j,j) = Odometry_InfMatrix(1,i);
%         j=j+1;
%     end
% end
% H_odom = H_odom(1:j-1,1:j-1);
%
% % Build the information matrix related to the observation from the imported data
% H_obs = zeros(size(Observation_InfMatrix,2),size(Observation_InfMatrix,2));
% j=1;
% for i = 1:size(Observation_InfMatrix,2)
%     if (Observation_InfMatrix(1,i) ~= 0)
%         H_obs(j,j) = Observation_InfMatrix(1,i);
%         j=j+1;
%     end
% end
% H_obs = H_obs(1:j-1,1:j-1);


% Import real (noisy??) robot poses and transform them to get the real (noisy??) camera poses.

quat = [CameraCalibration(8),CameraCalibration(5:7)]; % Build quaternion from camera parameters
R = eul2rot(quat2eul(quat)); % Build rotation matrix from quaternion
t = CameraCalibration(2:4)'; % Get camera translation from camera parameters
transformation = [R(1,:),t(1);R(2,:),t(2);R(3,:),t(3); 0,0,0,1]; % Build the 4x4 camera transformation

for i=1:size(NoisyRobotPose) % each row of NoisyRobotPose is a vertex: the noisy odometry of the platform on which the camera is mounted.
    
    % Here I extract the camera ground truth poses, affected by noise.
    quat = [NoisyRobotPose(i,8),NoisyRobotPose(i,5:7)];
    R = eul2rot(quat2eul(quat));
    RobotPoses_GroundTruth(:,:,i) = [R(1,:), NoisyRobotPose(i,2); R(2,:), NoisyRobotPose(i,3); R(3,:), NoisyRobotPose(i,4); 0,0,0,1];
    CameraPoses_GroundTruth(:,:,i) = RobotPoses_GroundTruth(:,:,i) * transformation;
    %Poses_GroundTruth(i,:) = [NoisyRobotPose(i,2:4),euler(3),euler(2),euler(1)];
    
    % Unfold camera and robot poses for plotting.
    CameraXYZ(i,:) = CameraPoses_GroundTruth(1:3,4,i)';
    eulers_camera(i,:) = rotm2eul(CameraPoses_GroundTruth(1:3,1:3,i));
    CameraOrient(i,:) = [eulers_camera(3),eulers_camera(2),eulers_camera(1)];
    
    %     RobotXYZ(i,:) = RobotPoses_GroundTruth(1:3,4,i)';
    %     eulers_robot(i,:) = rotm2eul(RobotPoses_GroundTruth(1:3,1:3,i));
    %     RobotOrient(i,:) = [eulers_robot(3),eulers_robot(2),eulers_robot(1)];
    
    %     plot3(CameraXYZ(i,1),CameraXYZ(i,2),CameraXYZ(i,3),'r.','MarkerSize',20)
    %     hold on
    %     plot3(RobotPoses_GroundTruth(1,4,i),RobotPoses_GroundTruth(2,4,i),RobotPoses_GroundTruth(3,4,i),'k.','MarkerSize',20)
end


% Transform all landmarks from the world frame to the camera frame.
for i=1:size(Landmark_GroundTruth,1)
    AllLandmarks_CameraFrame(i,:) = [transformation * [Landmark_GroundTruth(i,2:4)';1]]';
end
AllLandmarks_CameraFrame = [Landmark_GroundTruth(:,1), AllLandmarks_CameraFrame];


% Iterate over time and at each instant throw ICP
for time=1:1 %size(Start_End_PoseID,1)

    BackProjectedLandmarks = []; % Contains the back-projected landmarks, emptied at every iteration.

    % Get initial pose ID
    init_poseID = Start_End_PoseID(time,1);
    
    quat = [NoisyOdometry(time,7),NoisyOdometry(time,4:6)];  % NoisyOdometry is the odometry from one pose to the next
    R = eul2rot(quat2eul(quat));
    t = NoisyOdometry(time,1:3)';
    rot_trasl = [R(1,:),t(1);R(2,:),t(2);R(3,:),t(3); 0,0,0,1]; % rototraslation from one pose to the next
    curr_cameraPose(:,:,time+1) = curr_cameraPose(:,:,time) * rot_trasl; % new state homogeneous transformation
 
    for i = 1:size(AllLandmarks_CameraFrame,1)  % transform all the landmarks into the current camera frame, applying the inverse transform from the current to the previous pose.
        AllLandmarks_CameraFrame(i,2:end) = [rot_trasl * AllLandmarks_CameraFrame(i,2:end)'];  %inv(rot_trasl) * AllLandmarks_CameraFrame(i,2:end)']';
    end                                     % OPPURE SENZA INV???

    % Get current pose ID
    curr_cameraPoseID = Start_End_PoseID(time,2);

    onepose_indeces = find(poseID_obsLandmID_UVD(:,1)==curr_cameraPoseID); %get the current pose indeces in the observations vector.
    land_ids = poseID_obsLandmID_UVD(onepose_indeces,2); % get the ids of the landmarks observed from that position ID.
    BackProjectedLandmarks = backproject(poseID_obsLandmID_UVD(onepose_indeces,3:end),CameraCalibration); % backproject landmarks from camera to world in camera reference frame.
    BackProjectedLandmarks = [land_ids BackProjectedLandmarks];
    [~,landm_pos] = ismember(land_ids,AllLandmarks_CameraFrame(:,1));    % get the position in the camera frame of the observed landmarks.
    Landmarks_CameraFrame_ICP = AllLandmarks_CameraFrame(landm_pos,1:end-1);

    Landmarks_CameraFrame_ICP(:,2:end)';
    BackProjectedLandmarks(:,2:end)';

    [newcurr_cameraPose(:,:,time+1), chi_stats, num_inliers]=ICP(curr_cameraPose(:,:,time+1), Landmarks_CameraFrame_ICP(:,2:end)', BackProjectedLandmarks(:,2:end)', 100, 0, 10e9);
     
     CameraPoses_GroundTruth(:,:,time+1)
     curr_cameraPose(:,:,time+1)
     newcurr_cameraPose(:,:,time+1)
%     
%     
%     plot3(CameraXYZ(time,1),CameraXYZ(time,2),CameraXYZ(time,3),'r.','MarkerSize',20)
%     hold on
%     plot3(curr_cameraPose(1,4,time+1),curr_cameraPose(2,4,time+1),curr_cameraPose(3,4,time+1),'b.','MarkerSize',20)
%     hold on
%     plot3(newcurr_cameraPose(1,4,time+1),newcurr_cameraPose(2,4,time+1),newcurr_cameraPose(3,4,time+1),'g.','MarkerSize',20)


end
