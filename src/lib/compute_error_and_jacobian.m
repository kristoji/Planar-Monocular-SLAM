function [is_valid, err, J_pose, J_landmark] = compute_error_and_jacobian(Xr, Xl, u_obs)

    global K; global img_width; global img_height; global T_cam;

    is_valid = false;

    err = [0;0];
    J_pose = zeros(2, 3);
    J_landmark = zeros(2, 3);


    % iR=Xr(1:3,1:3)';
    % it=-iR*Xr(1:3,4);
    invXr = invHomo(Xr);
    iR = invXr(1:3,1:3);
    it = invXr(1:3,4);

    p_in_r = iR*Xl + it;    % == inv(Xr) * Xl

    % iR_cam = T_cam(1:3,1:3)';
    % it_cam=-iR_cam*T_cam(1:3,4);
    invT_cam = invHomo(T_cam);
    iR_cam = invT_cam(1:3,1:3);
    it_cam = invT_cam(1:3,4);

    p_in_cam = iR_cam * p_in_r + it_cam; % == inv(T_cam) * p_in_r
    
    if (p_in_cam(3) < 0),
        return; % invalid measurement
    end

    p_cam = K * p_in_cam;
    iz = 1/p_cam(3);
    iz2 = iz^2;

    z_hat = p_cam(1:2) * iz;
    if (z_hat(1) < 0 || z_hat(1) > img_width || z_hat(2) < 0 || z_hat(2) > img_height),
        return; % invalid measurement
    end

    Jp = [iz, 0, -p_cam(1)*iz2;
          0, iz, -p_cam(2)*iz2];


    % J_wr = de( R' * (R'(dtheta) * (Xl + dxl) - dt) - t) / de([dx, dy, dtheta]) on [dx, dy, dtheta]=0
    iR_cam_r = iR_cam * iR;
    
    J_wr = zeros(3,3);
    J_wr(:,1:2) = -iR_cam_r(: , 1:2);
    % de(R'(dtheta) * (Xl) = [Xl(2); -Xl(1); 0]; when dtheta=0
    J_wr(:,3) = iR_cam_r * [Xl(2); -Xl(1); 0];

    J_wl = iR_cam_r;


    J_pose = Jp * K * J_wr;
    J_landmark = Jp * K * J_wl;
    
    err = z_hat - u_obs;
    is_valid = true;
end