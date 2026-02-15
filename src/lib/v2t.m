
% computes the homogeneous transform matrix A of the pose vector v
% A:[ R t ] 4x4 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector

# ASSUMPTION: rotation is only around z-axis
function A=v2t(v)
  	c=cos(v(3));
  	s=sin(v(3));
	A=[c, -s, 0, v(1) ;
	   s,  c, 0, v(2) ;
	   0,  0, 1, 0 ;
	   0,  0, 0, 1 ];
end