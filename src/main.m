close all;
clear;
clc;
addpath "./lib";

USE_OLD_GUESS = false; % set to false to re-run triangulation and get a new guess
filename_initial_guess = '../data/initial_guess.dat'; % load/save based on USE_OLD_GUESS


global K; global T_cam; global img_width; global img_height;
[K, T_cam, z_near, z_far, img_width, img_height] = load_camera("../data/camera.dat");


more off;

disp('loading...');
Traj=load("../data/trajectory.dat");
L=load("../data/world.dat");
XR_true_vec = Traj(:,5:7)';
XL_true = L(:, 2:4)';

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

% ------------ Triangulation-based initialization -------------------------
if USE_OLD_GUESS,
    disp('loading old guess...');
    initial_guess = load(filename_initial_guess);
    XL_guess = zeros(3, num_landmarks);
    guessed = zeros(1, num_landmarks);
    for i=1:size(initial_guess, 1),
        land_index = initial_guess(i, 1);
        guessed(land_index) = 1;
        XL_guess(:, land_index) = initial_guess(i, 2:4)';
    end
else
    disp('computing new guess using triangulation...');

    use_multi_view_triangulation = false; 
    use_only_this_landmark = -1; % set to a landmark index to test only on that landmark

    observations = get_observations();
    [XL_guess, guessed] = initial_guess(observations, XR_guess, use_multi_view_triangulation, use_only_this_landmark);

    % plot all the triangulated points for landmark 1 and ground truth
    h2 = figure(2);
    hold on;


    if use_only_this_landmark < 0,
        % compute avg error for all landmarks
        total_error = 0;
        error_history = zeros(num_landmarks, 1);
        for i=1:num_landmarks,
            err = norm(XL_guess(:, i) - XL_true(:, i));
            total_error = total_error + err;
            error_history(i) = err;
        end
        avg_error = total_error / num_landmarks;
        subplot(1,2,1);
        hold on;
        title(sprintf('Triangulation Errors for All Landmarks (Avg: %.2f m)', avg_error));
        plot(error_history, '-');
        xlabel('Landmark Index');
        ylabel('Triangulation Error (m)');
        subplot(1,2,2);
        hold on;
        title(sprintf('Triangulated Points for All Landmarks, Guessed: %d / %d', sum(guessed), num_landmarks));

        plot3(XL_true(1,:), XL_true(2,:), XL_true(3,:), 'ro', 'linewidth', 2, 'DisplayName', 'Ground Truth');
        plot3(XL_guess(1,:), XL_guess(2,:), XL_guess(3,:), 'gx', 'linewidth', 2, 'DisplayName', 'Initial Guess');
    else
        % plot all the triangulated points for the chosen landmark and ground truth
        err = norm(XL_guess(:, use_only_this_landmark) - XL_true(:, use_only_this_landmark));
        title(sprintf('Triangulated Points for Landmark %d (Error: %.2f)', use_only_this_landmark, err));
        plot3(points_to_plot(:,1), points_to_plot(:,2), points_to_plot(:,3), 'bo', 'linewidth', 2, 'DisplayName', 'Triangulated Points');
        plot3(XL_true(1,use_only_this_landmark), XL_true(2,use_only_this_landmark), XL_true(3,use_only_this_landmark), 'ro', 'linewidth', 2, 'DisplayName', 'Ground Truth');
        plot3(XL_guess(1,use_only_this_landmark), XL_guess(2,use_only_this_landmark), XL_guess(3,use_only_this_landmark), 'gx', 'linewidth', 2, 'DisplayName', 'Initial Guess');
    end
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    legend();
    saveas(h2, '../imgs/triangulation.png');

    % dump the initial guess to a file for debugging
    initial_guess_file = fopen(filename_initial_guess, 'w');
    for i=1:num_landmarks,
        if ~guessed(i),
            continue;
        end
        Xl = XL_guess(:, i);
        fprintf(initial_guess_file, '%d %f %f %f\n', i, Xl(1), Xl(2), Xl(3));
    end
    fclose(initial_guess_file);
end


%% ------------ bundle adj -------------------------

% apply perturbation to XL
% perturbation = 1;
% XL_guess = XL_true + randn(size(XL_guess)) * perturbation;

state.Xr = XR_guess;
state.Xl = XL_guess;

damping = 1;
kernel_threshold_sq = 1e1;
num_iterations = 20;

observations = get_observations();

[state, chi_stats] = bundleAdjustment(state, observations, damping, kernel_threshold_sq, num_iterations);

disp('done');

h3 = figure(3);
subplot(1,2,1);
hold on;
title('Bundle Adjustment Chi Statistics');
plot(chi_stats.chi_tot, 'o-');
xlabel('Iteration');
ylabel('Total Chi');
subplot(1,2,2);
hold on;
title('Bundle Adjustment Inliers');
plot(chi_stats.num_inliers, 'o-');
xlabel('Iteration');
ylabel('Number of Inliers');
saveas(h3, '../imgs/chi_stats.png');

% plot Xr and then Xl
h4 = figure(4);
subplot(1,2,1);
hold on;
title('Robot Trajectory');
plot(Traj(:,5),Traj(:,6), 'r-', 'linewidth', 2, 'DisplayName', 'Ground Truth Trajectory');
plot(Traj(:,2),Traj(:,3), 'b-', 'linewidth', 2, 'DisplayName', 'Initial Guess Trajectory');
x = reshape(state.Xr(1,4,:), [], 1);
y = reshape(state.Xr(2,4,:), [], 1);
plot(x, y, 'g-', 'linewidth', 2, 'DisplayName', 'Estimated Trajectory');
xlabel('X');
ylabel('Y');
legend();
subplot(1,2,2);
hold on;
title('Landmark Positions');
% plot seen ground truth landmarks in red and unseen in magenta
guessed_XL_true = XL_true(:, guessed);
unguessed_XL_true = XL_true(:, ~guessed);
plot3(guessed_XL_true(1,:), guessed_XL_true(2,:), guessed_XL_true(3,:), 'ro', 'linewidth', 2, 'DisplayName', 'Guessed Ground Truth');
plot3(unguessed_XL_true(1,:), unguessed_XL_true(2,:), unguessed_XL_true(3,:), 'mo', 'linewidth', 2, 'DisplayName', 'Unguessed Ground Truth');
plot3(state.Xl(1,:), state.Xl(2,:), state.Xl(3,:), 'gx', 'linewidth', 2, 'DisplayName', 'Estimated');
xlabel('X');
ylabel('Y');
zlabel('Z');
legend();
saveas(h4, '../imgs/ba_results.png');



% ------------------ Evaluation ------------------

ev_err.trasl = [];
ev_err.rot = [];

for i=1:num_poses-1,
    T_i = state.Xr(:, :, i);
    T_j = state.Xr(:, :, i+1);
    GT_i = v2t(XR_true_vec(:, i));
    GT_j = v2t(XR_true_vec(:, i+1));

    rel_T = invHomo(T_i) * T_j;
    rel_GT = invHomo(GT_i) * GT_j;
    error_T = invHomo(rel_T) * rel_GT;

    rot_error = atan2(error_T(2, 1), error_T(1, 1));
    transl_error = norm(error_T(1:3, 4));

    ev_err.rot = [ev_err.rot; rot_error];
    ev_err.trasl = [ev_err.trasl; transl_error];
end

% compute the whole RMSE (sum or mean?)
vn = vecnorm(state.Xl - XL_true).^2;
map_error_avg = sqrt(mean(vn));
map_error_sum = sqrt(sum(vn));
fprintf('Map avg RMSE: %.2f m\n', map_error_avg);
fprintf('Map sum RMSE: %.2f m\n', map_error_sum);

h5 = figure(5);
subplot(2,2,1);
hold on;
title(sprintf('Pose Translation Errors (Avg: %.2f m)', mean(ev_err.trasl)));
plot(ev_err.trasl, 'o-');
xlabel('Pose Index');
ylabel('Translation Error (m)');

subplot(2,2,2);
hold on;
title(sprintf('Pose Rotation Errors (Avg: %.2f rad)', mean(ev_err.rot)));
plot(ev_err.rot, 'o-');
xlabel('Pose Index');
ylabel('Rotation Error (rad)');

subplot(2,2,3:4);
axis off;
text(0.1, 0.5, sprintf('Landmark Map avg RMSE: %.2f m\nLandmark Map sum RMSE: %.2f m\n', map_error_avg, map_error_sum), 'FontSize', 12);
saveas(h5, '../imgs/pose_errors.png');


waitforbuttonpress;