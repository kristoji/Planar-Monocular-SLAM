

function [seq, gt_pose, odom_pose, uv_points] = read_meas(i, verbose=false)

    % uv_point = [point_id, landmark_id, u, v]

    filename = sprintf("../data/meas-%05d.dat", i);
    if verbose,
        printf('loading file: %s\n', filename);
    end

    file_id = fopen(filename, 'r');
    if file_id == -1,
        printf('Error opening file: %s\n', filename);
        return;
    end

    line = fgetl(file_id);
    seq = sscanf(line, 'seq: %d');

    line = fgetl(file_id);
    gt_pose = sscanf(line, 'gt_pose: %f %f %f');

    line = fgetl(file_id);
    odom_pose = sscanf(line, 'odom_pose: %f %f %f');

    uv_points = [];
    while ~feof(file_id),
        line = fgetl(file_id);
        if isempty(line),
            continue;
        end
        point = sscanf(line, 'point %d %d %f %f');
        uv_points = [uv_points; point'];
    end
    fclose(file_id);

    if verbose,
        printf('seq: %d\n', seq);
        printf('gt_pose: %f %f %f\n', gt_pose(1), gt_pose(2), gt_pose(3));
        printf('odom_pose: %f %f %f\n', odom_pose(1), odom_pose(2), odom_pose(3));
        printf('uv_points: %d x %d\n', size(uv_points,1), size(uv_points,2));
    end
return;