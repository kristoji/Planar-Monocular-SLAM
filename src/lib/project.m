function u = project(p, X)
    % project a 3D point p into the image plane of a camera with pose X
    global K;
    global T_cam;

    P = [K, zeros(3,1)] * inv(X * T_cam);
    p_homogeneous = P * p;
    u = p_homogeneous(1:2) / p_homogeneous(3);
end