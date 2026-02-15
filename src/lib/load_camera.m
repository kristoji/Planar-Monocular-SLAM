function [K, T_cam, z_near, z_far, width, height] = load_camera(filename, verbose=false)

    file_id = fopen(filename, 'r');
    if file_id == -1,
        printf('Error opening file: %s\n', filename);
        return;
    end
    
    # camera matrix
    line = fgetl(file_id);
    K = zeros(3,3);
    for i=1:3,
        line = fgetl(file_id);
        K(i,:) = sscanf(line, '%f %f %f');
    end

    # camera pose
    line = fgetl(file_id);
    T_cam = zeros(4,4);
    for i=1:4,
        line = fgetl(file_id);
        T_cam(i,:) = sscanf(line, '%f %f %f %f');
    end

    line = fgetl(file_id);
    z_near = sscanf(line, 'z_near: %f');
    line = fgetl(file_id);
    z_far = sscanf(line, 'z_far: %f');
    line = fgetl(file_id);
    width = sscanf(line, 'width: %d');
    line = fgetl(file_id);
    height = sscanf(line, 'height: %d');
    fclose(file_id);

    if verbose,
        printf('K:\n');
        disp(K);
        printf('T_cam:\n');
        disp(T_cam);
        printf('z_near: %f\n', z_near);
        printf('z_far: %f\n', z_far);
        printf('width: %d\n', width);
        printf('height: %d\n', height);
    end
return;
