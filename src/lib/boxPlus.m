function state = boxPlus(state, dx)

    global pose_dim; global landmark_dim; global num_poses; global num_landmarks;

    % Update poses
    for i=1:num_poses,
        idx_start = (i-1) * pose_dim + 1;
        idx_end = idx_start + pose_dim - 1;
        delta_pose = dx(idx_start:idx_end);
        state.Xr(:,:,i) = v2t(delta_pose) * state.Xr(:,:,i);
    end

    % Update landmarks
    for j=1:num_landmarks,
        idx_start = num_poses * pose_dim + (j-1) * landmark_dim + 1;
        idx_end = idx_start + landmark_dim - 1;
        delta_landmark = dx(idx_start:idx_end);
        state.Xl(:,j) = state.Xl(:,j) + delta_landmark;
    end

end