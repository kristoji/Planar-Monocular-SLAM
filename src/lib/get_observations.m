function observations = get_observations()
    global num_poses;
    global num_landmarks;
    observations = [];
    for i = 0:num_poses-1,
        [_, _, _, uv] = read_meas(i);
        obs.pose_index = i + 1; % 1-based indexing
        obs.landmark_indexes = 1+uv(:, 2); % landmark_id is 0-based in the data, convert to 1-based for MATLAB indexing
        obs.uvs = uv(:, 3:4); % u, v
        observations = [observations; obs];
    end
end