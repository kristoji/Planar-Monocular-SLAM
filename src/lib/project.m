function [u, is_valid] = project(p, X)
    % project a 3D point p into the image plane of a camera with pose X
    global K;
    global T_cam;
    global img_height;
    global img_width;

    P = [K, zeros(3,1)] * invHomo(X * T_cam);
    p_homogeneous = P * p;
    u = p_homogeneous(1:2) / p_homogeneous(3);

    is_valid = false;
    if u(1) > 0 && u(1) < img_width && u(2) > 0 && u(2) < img_height,
        is_valid = true;
    end
end