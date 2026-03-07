function [state, chi_stats] = bundleAdjustment(state, landmark_observations, pose_observations, damping, kernel_threashold_sq, num_iters)

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

        % ---------------- proj ba ----------------

        % fprintf('size of landmark_observations: %d\n', size(landmark_observations, 1));
        for meas_index = 1:size(landmark_observations, 1)
            obs = landmark_observations(meas_index);
            pose_index = obs.pose_index;

            % for each pose, there are a lot of landmark_observations
            for obs_index = 1:size(obs.uvs, 1)
                tot_meas_count = tot_meas_count + 1;
                landmark_index = obs.landmark_indexes(obs_index);
                
                if state.guessed(landmark_index) == 0,
                    continue; % skip unguessed landmarks
                end
                
                u_obs = obs.uvs(obs_index, :)';

                Xr = state.Xr(:, :, pose_index);
                Xl = state.Xl(:, landmark_index);

                [is_valid, err, J_pose, J_landmark] = compute_proj_error_and_jacobian(Xr, Xl, u_obs);

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

        % ----------------- pose ba -----------------

        for meas_index = 1:size(pose_observations, 1)
            obs = pose_observations(meas_index);
            pose_i_index = obs.pose_i_index;
            pose_j_index = obs.pose_j_index;
            Z = obs.Z;

            Xi = state.Xr(:, :, pose_i_index);
            Xj = state.Xr(:, :, pose_j_index);

            [err, Ji, Jj] = compute_pose_error_and_jacobian(Xi, Xj, Z);

            if ~is_valid,
                invalid_meas_count += 1;
                continue; % skip invalid measurements
            end

            Omega = eye(12);
            Omega(1:9, 1:9) *= 1e3; % grisetti said we need to pimp the rotation part a little
            
            chi = err' * Omega * err;
            if chi < kernel_threashold_sq,
                num_inliers = num_inliers + 1;
            else 
                weight = sqrt(kernel_threashold_sq / chi); % apply robust kernel
                Ji *= weight;
                Jj *= weight;
                err *= weight;
                chi = kernel_threashold_sq;
            end
            chi_tot += chi;

            pose_i_matrix_index = (pose_i_index - 1) * pose_dim + 1;
            pose_j_matrix_index = (pose_j_index - 1) * pose_dim + 1;

            H(pose_i_matrix_index:pose_i_matrix_index+pose_dim-1, pose_i_matrix_index:pose_i_matrix_index+pose_dim-1) += Ji' * Omega * Ji;

            H(pose_i_matrix_index:pose_i_matrix_index+pose_dim-1, pose_j_matrix_index:pose_j_matrix_index+pose_dim-1) += Ji' * Omega * Jj;

            H(pose_j_matrix_index:pose_j_matrix_index+pose_dim-1, pose_i_matrix_index:pose_i_matrix_index+pose_dim-1) += Jj' * Omega * Ji;

            H(pose_j_matrix_index:pose_j_matrix_index+pose_dim-1, pose_j_matrix_index:pose_j_matrix_index+pose_dim-1) += Jj' * Omega * Jj;

            b(pose_i_matrix_index:pose_i_matrix_index+pose_dim-1) += Ji' * Omega * err;
            b(pose_j_matrix_index:pose_j_matrix_index+pose_dim-1) += Jj' * Omega * err;
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

    h = figure();
    subplot(1,2,1);
    hold on;
    title('Sparsity Pattern of Hessian');
    spy(H);
    xlabel('Column Index');
    ylabel('Row Index');
    subplot(1,2,2);
    hold on;
    title('Sparsity Pattern of Hessian (Zoomed)');
    spy(H(1:100, 1:100));
    xlabel('Column Index');
    ylabel('Row Index');
    saveas(h, '../imgs/hessian_sparsity.png');

end