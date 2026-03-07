function [XL_guess, guessed] = compute_initial_guess(observations, XR_guess, use_multi_view_triangulation)
    global num_landmarks;

    if nargin < 3
        use_multi_view_triangulation = false;
    end

    XL_guess = zeros(3, num_landmarks);
    avg_count = zeros(1, num_landmarks); % to maintain a moving avg

    if ~use_multi_view_triangulation,
        points_to_plot = [];

        for i=1:size(observations, 1),
            obs1 = observations(i);
            pose_index1 = obs1.pose_index;
            landmark_indexes1 = obs1.landmark_indexes;

            for j=i+1:size(observations, 1),
                obs2 = observations(j);
                pose_index2 = obs2.pose_index;
                landmark_indexes2 = obs2.landmark_indexes;

                common_landmarks = intersect(landmark_indexes1, landmark_indexes2);
                if isempty(common_landmarks),
                    continue;
                end

                X1 = XR_guess(:, :, pose_index1);
                X2 = XR_guess(:, :, pose_index2);

                % % compute baseline and skip if too small (triangulation won't work well)
                % t1 = X1(1:3,4);
                % t2 = X2(1:3,4);

                % baseline = norm(t1 - t2);

                % if baseline < 0.5
                %     continue;
                % end

                for k=1:length(common_landmarks),
                    landmark_index = common_landmarks(k);
                    uv1 = obs1.uvs(landmark_indexes1 == landmark_index, :)';
                    uv2 = obs2.uvs(landmark_indexes2 == landmark_index, :)';

                    [p_triangulated, valid] = triangulate(uv1, uv2, X1, X2);

                    % exclude points that are too far away from the XY origin (outliers) or z < 0 (under the ground)
                    if norm(p_triangulated(1:2)) > 50 || ~valid,
                        continue;
                    end

                    XL_guess(:, landmark_index) = (XL_guess(:, landmark_index) * avg_count(landmark_index) + p_triangulated) / (avg_count(landmark_index) + 1);
                    avg_count(landmark_index) += 1;
                end
            end
        end
    else
        % ---------------------- multi-view triangulation ----------------------
        % for each landmark, find all the poses that see it, and triangulate using all those poses together
        for landmark_index=1:num_landmarks,
            poses_that_see_landmark = [];
            uvs_for_landmark = [];
            for i=1:size(observations, 1),
                obs = observations(i);
                pose_index = obs.pose_index;
                landmark_indexes = obs.landmark_indexes;

                uv = obs.uvs(landmark_indexes == landmark_index, :);
                if ~isempty(uv),
                    uvs_for_landmark = [uvs_for_landmark; uv];
                    poses_that_see_landmark = [poses_that_see_landmark; pose_index];
                end

            end

            if length(poses_that_see_landmark) < 2,
                continue; % need at least 2 views to triangulate
            end

            Xs = zeros(4, 4, length(poses_that_see_landmark));
            for j=1:length(poses_that_see_landmark),
                Xs(:,:,j) = XR_guess(:, :, poses_that_see_landmark(j));
            end

            p_triangulated = multi_triangulation(uvs_for_landmark, Xs);

            % exclude points that are too far away from the XY origin (outliers) or z < 0 (under the ground)
            if norm(p_triangulated(1:2)) > 50,
                continue;
            end

            XL_guess(:, landmark_index) = p_triangulated;
            avg_count(landmark_index) = 1;
        end
    end

    guessed = avg_count > 0;

end