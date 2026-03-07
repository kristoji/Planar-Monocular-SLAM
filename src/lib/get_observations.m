function [landmark_observations, pose_observations] = get_observations(XR_guess)
    global num_poses;
    global num_landmarks;

    landmark_observations = [];
    pose_observations = [];

    for i = 1:num_poses,
        [_, _, _, uv] = read_meas(i-1); % read_meas uses 0-based indexing
        obs.pose_index = i; % 1-based indexing
        obs.landmark_indexes = 1+uv(:, 2); % landmark_id is 0-based in the data, convert to 1-based
        obs.uvs = uv(:, 3:4); % u, v
        landmark_observations = [landmark_observations; obs];

        if i < num_poses,
            pose_obs.pose_i_index = i;
            pose_obs.pose_j_index = i + 1;
            Xi = XR_guess(:,:,i);
            Xj = XR_guess(:,:,i+1);
            rel_X = invHomo(Xi) * Xj;
            pose_obs.Z = rel_X;
            pose_observations = [pose_observations; pose_obs];
        end
    end
end