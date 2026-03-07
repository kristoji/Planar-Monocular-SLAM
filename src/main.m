close all;
clear;
clc;
addpath "./lib";

USE_OLD_GUESS = false; % set to false to re-run triangulation and get a new guess
filename_initial_guess = '../data/initial_guess-3.dat'; % load/save based on USE_OLD_GUESS

#derivative of rotation matrix w.r.t rotation around x, in 0
global  Rx0=[0 0 0; 0 0 -1; 0 1 0];

#derivative of rotation matrix w.r.t rotation around y, in 0
global  Ry0=[0 0 1; 0 0 0; -1 0 0];

#derivative of rotation matrix w.r.t rotation around z, in 0
global  Rz0=[0 -1 0; 1  0 0; 0  0 0];


global K; global T_cam; global img_width; global img_height; global z_near; global z_far;
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
[landmark_observations, pose_observations] = get_observations(XR_guess);


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

    [XL_guess, guessed] = compute_initial_guess(landmark_observations, XR_guess, use_multi_view_triangulation);
    plot_initial_guess(XL_guess, XL_true, guessed);

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
state.guessed = guessed;

damping = 1e-6;
kernel_threshold_sq = 1e1;
num_iterations = 10;

[landmark_observations, pose_observations] = get_observations(XR_guess);

[state, chi_stats] = bundleAdjustment(state, landmark_observations, pose_observations, damping, kernel_threshold_sq, num_iterations);

disp('done');

plot_chi_stats(chi_stats);

guessed_XL_true = XL_true(:, guessed>0);
guessed_XL_est = state.Xl(:, guessed>0);
plot_ba_estimate(state, Traj, XL_true, guessed_XL_true, guessed_XL_est, guessed);


% ------------------ Evaluation ------------------

%rmse on the guessed landmarks only
vn = vecnorm(guessed_XL_est - guessed_XL_true).^2;
map_error = sqrt(mean(vn));
fprintf('Landmark Map RMSE (guessed landmarks only): %.2f m\n', map_error);

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

plot_eval(ev_err, map_error, guessed);

waitforbuttonpress;