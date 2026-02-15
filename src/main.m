close all;
clear;
clc;
addpath "./lib";



[K, T_cam, z_near, z_far, width, height] = load_camera("../data/camera.dat");
global K;
global T_cam;


more off;

disp('loading...');
Traj=load("../data/trajectory.dat");
L=load("../data/world.dat");

h1 = figure(1);
hold on;
plot(Traj(:,5),Traj(:,6), 'r-', 'linewidth', 2);
hold on;
plot(Traj(:,2),Traj(:,3), 'g-', 'linewidth', 2);


[_, gt_pose1, _, uv_points1] = read_meas(0);
[_, gt_pose2, _, uv_points2] = read_meas(50);
X1 = v2t(gt_pose1);
X2 = v2t(gt_pose2);

# Test triangulation with random points
# - we create random points in 3D and project them
# - then we triangulate and plot them to see the results

num_points = 20;

points = [];
triangulated_points = [];
for i=1:num_points,
    rand_points = rand(num_points, 3) * 10 - [5,5,5];
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

waitfor(h2);