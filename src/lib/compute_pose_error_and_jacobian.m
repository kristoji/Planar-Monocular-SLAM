function [err, Ji, Jj] = compute_pose_error_and_jacobian(Xi, Xj, Z)
    global Rx0;
    global Ry0;
    global Rz0;

    err = zeros(12,1);
    Ji = zeros(12, 3);
    Jj = zeros(12, 3);

    Ri = Xi(1:3,1:3);
    Rj = Xj(1:3,1:3);
    ti = Xi(1:3,4);
    tj = Xj(1:3,4);
    tij = tj - ti;
    Ri_transpose = Ri';

    dR_dax = Ri_transpose * Rx0 * Rj;
    dR_day = Ri_transpose * Ry0 * Rj;
    dR_daz = Ri_transpose * Rz0 * Rj;

    % Jj(1:9,4) = reshape(dR_dax, 9, 1);
    % Jj(1:9,5) = reshape(dR_day, 9, 1);
    % Jj(1:9,6) = reshape(dR_daz, 9, 1);
    % Jj(10:12,1:3) = Ri_transpose;
    % Jj(10:12,4:6) = -Ri_transpose * skew(tj);

    Jj(10:12,1:2) = Ri_transpose(:,1:2);
    Jj(1:9,3) = reshape(dR_daz, 9, 1);
    Jj(10:12,3) = (-Ri_transpose * skew(tij))(:,3);

    Ji = -Jj;

    % e = flattenIsometryByColumns(Z_hat-Z);
    Z_hat = invHomo(Xi) * Xj;
    err(1:9) = reshape(Z_hat(1:3,1:3) - Z(1:3,1:3), 9, 1);
    err(10:12) = Z_hat(1:3,4) - Z(1:3,4);


end