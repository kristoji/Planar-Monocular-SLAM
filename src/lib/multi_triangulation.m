function p = multi_triangulation(uvs, poses)
    % triangulate 3D points from many 2D correspondences
    % us: 2D points in images (cell array of size N, each cell is 2xM)
    % Xs: 4x4 homogeneous poses of the cameras (cell array of size N, each cell is 4x4)
    % p: 3D point in world coordinates

    global K; global T_cam;

    N = size(uvs, 1);
    
    % Initialize variables for the linear system M * p = b
    M = zeros(3, 3);
    b = zeros(3, 1);
    I = eye(3); % 3x3 Identity matrix
    
    invK = inv(K);
    
    for i = 1:N
        u = uvs(i, 1);
        v = uvs(i, 2);
        
        % The pose of the camera in the world frame (Camera-to-World)
        T_wc = poses(:,:,i) * T_cam;
        
        % 1. Ray Origin (o_i): The camera's position in the world
        o_i = T_wc(1:3, 4);
        
        % 2. Ray Direction in Camera Frame: Convert pixel to normalized camera coordinates
        p_cam = invK * [u; v; 1];
        dir_cam = p_cam / norm(p_cam); % Normalize to a unit vector
        
        % 3. Ray Direction in World Frame (d_i): Rotate the direction by the camera's rotation
        R_wc = T_wc(1:3, 1:3);
        d_i = R_wc * dir_cam;
        
        Projection_ortho = I - (d_i * d_i');
        
        M = M + Projection_ortho;
        b = b + Projection_ortho * o_i;
    end
    
    p = M \ b;
return;