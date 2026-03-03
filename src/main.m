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
XR_true_vec = Traj(:,5:7)';
XL_true = L(:, 2:4)';

global pose_dim; global landmark_dim; global num_poses; global num_landmarks;
pose_dim = 3;
landmark_dim = 3;
num_poses = size(XR_true_vec, 2);
num_landmarks = size(XL_true, 2);
fprintf('num_poses: %d, num_landmarks: %d\n', num_poses, num_landmarks);

% ------------ Triangulation-based initialization -------------------------

% apply v2t to all odometry poses
XR_guess = zeros(4, 4, num_poses);
for i=1:num_poses,
    XR_guess(:,:,i) = v2t(Traj(i,2:4));
end

observations = get_observations();
XL_guess = zeros(3, num_landmarks);
avg_count = zeros(1, num_landmarks); % to maintain a moving avg

points_to_plot = [];
choosen_land = -1; % to test only on one landmark

for i=1:size(observations, 1),
    obs1 = observations(i);
    pose_index1 = obs1.pose_index;
    landmark_indexes1 = obs1.landmark_indexes;

    for j=i+1:size(observations, 1),
        obs2 = observations(j);
        pose_index2 = obs2.pose_index;
        landmark_indexes2 = obs2.landmark_indexes;

        if choosen_land < 0,
            common_landmarks = intersect(landmark_indexes1, landmark_indexes2);
            if isempty(common_landmarks),
                continue;
            end

            X1 = XR_guess(:, :, pose_index1);
            X2 = XR_guess(:, :, pose_index2);

            for k=1:length(common_landmarks),
                landmark_index = common_landmarks(k);
                uv1 = obs1.uvs(landmark_indexes1 == landmark_index, :)';
                uv2 = obs2.uvs(landmark_indexes2 == landmark_index, :)';

                p_triangulated = triangulate(uv1, uv2, X1, X2);

                % exclude points that are too far away from the XY origin (outliers) or z < 0 (under the ground)
                if norm(p_triangulated(1:2)) > 50,
                    continue;
                end

                XL_guess(:, landmark_index) = (XL_guess(:, landmark_index) * avg_count(landmark_index) + p_triangulated) / (avg_count(landmark_index) + 1);
                avg_count(landmark_index) += 1;
            end

        else

            % --------------------------------------------
            % for testing, let's do it just for one landmark

            X1 = XR_guess(:, :, pose_index1);
            X2 = XR_guess(:, :, pose_index2);
            uv1 = obs1.uvs(landmark_indexes1 == choosen_land, :)';
            uv2 = obs2.uvs(landmark_indexes2 == choosen_land, :)';

            if isempty(uv1) || isempty(uv2),
                continue;
            end

            p_triangulated = triangulate(uv1, uv2, X1, X2);
            % exclude points that are too far away from the XY origin (outliers) or z < 0 (under the ground)
            if norm(p_triangulated(1:2)) > 50,
                continue;
            end
            points_to_plot = [points_to_plot; p_triangulated'];
            XL_guess(:, choosen_land) = (XL_guess(:, choosen_land) * avg_count(choosen_land) + p_triangulated) / (avg_count(choosen_land) + 1);
            avg_count(choosen_land) += 1;
        end
    end
end

% plot all the triangulated points for landmark 1 and ground truth
h2 = figure(2);
hold on;

if choosen_land < 0,
    % compute avg error for all landmarks
    total_error = 0;
    count = 0;
    for i=1:num_landmarks,
        if avg_count(i) > 0,
            err = norm(XL_guess(:, i) - XL_true(:, i));
            total_error = total_error + err;
            count = count + 1;
        end
    end
    avg_error = total_error / count;
    title(sprintf('Triangulated Points for All Landmarks (Avg Error: %.2f)', avg_error));

    plot3(XL_true(1,:), XL_true(2,:), XL_true(3,:), 'ro', 'linewidth', 2, 'DisplayName', 'Ground Truth');
    plot3(XL_guess(1,:), XL_guess(2,:), XL_guess(3,:), 'gx', 'linewidth', 2, 'DisplayName', 'Initial Guess');
else
    err = norm(XL_guess(:, choosen_land) - XL_true(:, choosen_land));
    title(sprintf('Triangulated Points for Landmark %d (Error: %.2f)', choosen_land, err));
    plot3(points_to_plot(:,1), points_to_plot(:,2), points_to_plot(:,3), 'bo', 'linewidth', 2, 'DisplayName', 'Triangulated Points');
    plot3(XL_true(1,choosen_land), XL_true(2,choosen_land), XL_true(3,choosen_land), 'ro', 'linewidth', 2, 'DisplayName', 'Ground Truth');
    plot3(XL_guess(1,choosen_land), XL_guess(2,choosen_land), XL_guess(3,choosen_land), 'gx', 'linewidth', 2, 'DisplayName', 'Initial Guess');
end
xlabel('X');
ylabel('Y');
zlabel('Z');
legend();
saveas(h2, '../imgs/triangulation.png');


%% ------------ bundle adj -------------------------

state.Xr = XR_guess;
state.Xl = XL_guess;

damping = 0;
kernel_threshold_sq = 1e3;
num_iterations = 10;

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
plot3(XL_true(1,:), XL_true(2,:), XL_true(3,:), 'ro', 'linewidth', 2, 'DisplayName', 'Ground Truth');
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
    GT_i = v2t(Traj(i, 2:4));
    GT_j = v2t(Traj(i+1, 2:4));

    rel_T = inv(T_i) * T_j;
    rel_GT = inv(GT_i) * GT_j;
    error_T = inv(rel_T) * rel_GT;

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
title('Pose Translation Errors');
plot(ev_err.trasl, 'o-');
xlabel('Pose Index');
ylabel('Translation Error (m)');
yline(mean(ev_err.trasl), 'r--', sprintf('Avg: %.2f m', mean(ev_err.trasl)));

subplot(2,2,2);
hold on;
title('Pose Rotation Errors');
plot(ev_err.rot, 'o-');
xlabel('Pose Index');
ylabel('Rotation Error (rad)');
yline(mean(ev_err.rot), 'r--', sprintf('Avg: %.2f rad', mean(ev_err.rot)));

subplot(2,2,3:4);
axis off;
text(0.1, 0.5, sprintf('Map avg RMSE: %.2f m\nMap sum RMSE: %.2f m\n', map_error_avg, map_error_sum), 'FontSize', 12);
saveas(h5, '../imgs/pose_errors.png');


waitforbuttonpress;