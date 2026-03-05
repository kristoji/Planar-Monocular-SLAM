function [state, chi_stats] = bundleAdjustment(state, observations, damping, kernel_threashold_sq, num_iters)

    % pose_dim = 3; % dimension of the pose (x, y, theta)
    % landmark_dim = 3; % dimension of the landmark (x, y, z)
    % num_poses = size(state.Xr, 3);
    % num_landmarks = size(state.Xl, 2);

    global pose_dim; global landmark_dim; global num_poses; global num_landmarks;
    system_size = pose_dim * num_poses + landmark_dim * num_landmarks;

    chi_stats.chi_tot = [];
    chi_stats.num_inliers = [];

    for iter = 1:num_iters
        H = zeros(system_size, system_size);
        b = zeros(system_size, 1);

        % chi_stat = struct('chi_tot', 0, 'num_inliers', 0);
        chi_tot = 0;
        num_inliers = 0;
        invalid_meas_count = 0;
        tot_meas_count = 0;


        % fprintf('size of observations: %d\n', size(observations, 1));
        for meas_index = 1:size(observations, 1)
            obs = observations(meas_index);
            pose_index = obs.pose_index;

            % fprintf('size of obs.uvs: %d\n', size(obs.uvs, 1));
            % for each pose, there are a lot of observations
            for obs_index = 1:size(obs.uvs, 1)
                tot_meas_count = tot_meas_count + 1;
                landmark_index = obs.landmark_indexes(obs_index);
                u_obs = obs.uvs(obs_index, :)';

                Xr = state.Xr(:, :, pose_index);
                % if meas_index == 8
                %     disp(landmark_index);
                %     disp(size(state.Xl));
                % end
                Xl = state.Xl(:, landmark_index);

                [is_valid, err, J_pose, J_landmark] = compute_error_and_jacobian(Xr, Xl, u_obs);

                if ~is_valid,
                    invalid_meas_count += 1;
                    continue; % skip invalid measurements
                end

                chi = err' * err;
                if chi < kernel_threashold_sq,
                    num_inliers = num_inliers + 1;
                else 
                    weight = sqrt(kernel_threashold_sq / chi); % apply robust kernel
                    J_pose *= weight;
                    J_landmark *= weight;
                    err *= weight;
                    chi = kernel_threashold_sq;
                end
                chi_tot += chi;

                % Fill in the Hessian and gradient
                pose_start = (pose_index - 1) * pose_dim + 1;
                landmark_start = num_poses * pose_dim + (landmark_index - 1) * landmark_dim + 1;

                H(pose_start:pose_start+pose_dim-1, pose_start:pose_start+pose_dim-1) += J_pose' * J_pose;

                H(landmark_start:landmark_start+landmark_dim-1, landmark_start:landmark_start+landmark_dim-1) += J_landmark' * J_landmark;

                H(pose_start:pose_start+pose_dim-1, landmark_start:landmark_start+landmark_dim-1) += J_pose' * J_landmark;
                H(landmark_start:landmark_start+landmark_dim-1, pose_start:pose_start+pose_dim-1) += J_landmark' * J_pose;

                b(pose_start:pose_start+pose_dim-1) += J_pose' * err;
                b(landmark_start:landmark_start+landmark_dim-1) += J_landmark' * err;

            end
        end

        fprintf('Iteration %d: Total Chi = %.2f | Inliers = %d, Invalid Measurements = %d / Total Measurements = %d\n', iter, chi_tot, num_inliers, invalid_meas_count, tot_meas_count);

        chi_stats.chi_tot = [chi_stats.chi_tot, chi_tot];
        chi_stats.num_inliers = [chi_stats.num_inliers, num_inliers];

        % Add damping to the Hessian
        H += damping * eye(system_size);

        % check if H is symmetric positive definite
        if ~isequal(H, H') || any(eig(H) <= 0),
            fprintf('Hessian is not positive definite at iteration %d\n', iter);
        end

        % Solve for the update fixing the first pose to zero 
        dx = zeros(system_size, 1);
        dx(pose_dim+1:end) = -H(pose_dim+1:end, pose_dim+1:end) \ b(pose_dim+1:end);

        % Update the state
        state = boxPlus(state, dx);

        if norm(dx) < 1e-6,
            fprintf('Converged at iteration %d\n', iter);
            break;
        end
    end

end