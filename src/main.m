close all;
clear;
clc;
addpath "./lib";



global K; global T_cam; global img_width; global img_height;
[K, T_cam, z_near, z_far, img_width, img_height] = load_camera("../data/camera.dat");


more off;

disp('loading...');
Traj=load("../data/trajectory.dat");
L=load("../data/world.dat");

h1 = figure(1);
hold on;
% Ground truth trajectory
plot(Traj(:,5),Traj(:,6), 'r-', 'linewidth', 2);
hold on;
% Odometry trajectory
plot(Traj(:,2),Traj(:,3), 'g-', 'linewidth', 2);


%% ----------- test triangulation ------------------------
[_, gt_pose1, _, uv_points1] = read_meas(0);
[_, gt_pose2, _, uv_points2] = read_meas(50);
X1 = v2t(gt_pose1);
X2 = v2t(gt_pose2);

% Test triangulation with random points
% - we create random points in 3D and project them
% - then we triangulate and plot them to see the results

num_points = 20;

points = [];
triangulated_points = [];
rand_points = rand(num_points, 3) * 10 - [5,5,5];
for i=1:num_points,
    p = rand_points(i,:)';
    homogeneous_p = [p; 1];

    u1 = project(homogeneous_p, X1);
    u2 = project(homogeneous_p, X2);
    p_triangulated = triangulate(u1, u2, X1, X2);

    points = [points; p'];
    triangulated_points = [triangulated_points; p_triangulated'];
end

h2 = figure(2);
hold on;
plot3(points(:,1), points(:,2), points(:,3), 'bo', 'linewidth', 2);
plot3(triangulated_points(:,1), triangulated_points(:,2), triangulated_points(:,3), 'rx', 'linewidth', 2);

% waitfor(h2);

%% ------------ test bundle adj -------------------------

XR_true_vec = Traj(:,5:7)';
XL_true = read_world()';
global pose_dim; global landmark_dim; global num_poses; global num_landmarks;
pose_dim = 3;
landmark_dim = 3;
num_poses = size(XR_true_vec, 2);
num_landmarks = size(XL_true, 2);
fprintf('num_poses: %d, num_landmarks: %d\n', num_poses, num_landmarks);

% apply v2t to all odometry poses
XR_guess = zeros(4, 4, num_poses);
for i=1:num_poses,
    XR_guess(:,:,i) = v2t(Traj(i,2:4));
end

XL_guess=XL_true;

% for (pose_num=2:num_poses)
%     xr=rand(3,1)-0.5;
%     dXr=v2t(pert_scale*xr);
%     XR_guess(:,:,pose_num)=dXr*XR_guess(:,:,pose_num);
% endfor;

%apply a perturbation to each landmark
% pert_deviation=1;
% pert_scale=eye(3)*pert_deviation;
% dXl=(rand(landmark_dim, num_landmarks)-0.5)*pert_deviation;
% XL_guess+=dXl;

state.Xr = XR_guess;
state.Xl = XL_guess;

damping = 0;
kernel_threshold_sq = 1e3;
num_iterations = 25;

observations = get_observations();

[state, chi_stats] = bundleAdjustment(state, observations, damping, kernel_threshold_sq, num_iterations);

disp('done');

h3 = figure(3);
subplot(1,2,1);
plot(chi_stats.chi_tot, 'o-');
xlabel('Iteration');
ylabel('Total Chi');
subplot(1,2,2);
plot(chi_stats.num_inliers, 'o-');
xlabel('Iteration');
ylabel('Number of Inliers');    

% plot Xr and then Xl
h4 = figure(4);
subplot(1,2,1);
hold on;
plot(Traj(:,5),Traj(:,6), 'r-', 'linewidth', 2, 'DisplayName', 'Ground Truth Trajectory');
plot(Traj(:,2),Traj(:,3), 'b-', 'linewidth', 2, 'DisplayName', 'Initial Guess Trajectory');
x = reshape(state.Xr(1,4,:), [], 1);
y = reshape(state.Xr(2,4,:), [], 1);
plot(x, y, 'g-', 'linewidth', 2, 'DisplayName', 'Estimated Trajectory');
xlabel('X');
ylabel('Y');
subplot(1,2,2);
hold on;
plot3(XL_true(1,:), XL_true(2,:), XL_true(3,:), 'ro', 'linewidth', 2, 'DisplayName', 'Ground Truth');
plot3(state.Xl(1,:), state.Xl(2,:), state.Xl(3,:), 'gx', 'linewidth', 2, 'DisplayName', 'Estimated');
xlabel('X');
ylabel('Y');
zlabel('Z');

waitforbuttonpress;