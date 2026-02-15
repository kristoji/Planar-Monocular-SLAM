function T = invHomo(A)
    % computes the inverse of a homogeneous transformation matrix A
    R = A(1:3, 1:3);
    t = A(1:3, 4);
    T = [R', -R'*t; 0, 0, 0, 1];
end