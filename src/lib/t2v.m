
% computes the pose 2d pose vector v from an homogeneous transform A
% A:[ R t ] 4x4 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector

# ASSUMPTION: rotation is only around z-axis
function v=t2v(A)
	v(1:2, 1)=A(1:2,4);
	v(3,1)=atan2(A(2,1),A(1,1));
end