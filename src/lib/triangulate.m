function p = triangulate(u1, u2, X1, X2)
    % triangulate 3D points from 2D correspondences
    % u1: 2D points in image 1
    % u2: 2D points in image 2
    % X1: 4x4 homogeneous  of the camera for image 1
    % X2: 4x4 pose of the camera for image 2
    % p: 3D point in world coordinates
    global K;
    global T_cam;

    P1 = [K, zeros(3,1)] * invHomo(X1 * T_cam);
    P2 = [K, zeros(3,1)] * invHomo(X2 * T_cam);

    A = [u1(1)*P1(3,:) - P1(1,:);
        u1(2)*P1(3,:) - P1(2,:);
        u2(1)*P2(3,:) - P2(1,:);
        u2(2)*P2(3,:) - P2(2,:)];
    
    
    [~, ~, V] = svd(A);
    p_hom = V(:,end);
    p = p_hom(1:3)/p_hom(end);
    % printf('A: %d x %d\n', size(A,1), size(A,2));
    % printf('triangulated point: %d x %d\n', size(p,1), size(p,2));
return;
