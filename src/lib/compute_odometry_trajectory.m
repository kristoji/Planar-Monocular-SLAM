#computes the trajectory of the robot by chaining up
#the incremental movements of the odometry vector
#U:	a Nx3 matrix, each row contains the odoemtry ux, uy utheta
#T:	a Nx3 matrix, each row contains the robot position (starting from 0,0,0)

function T=compute_odometry_trajectory(U)
	T=zeros(size(U,1),3);
	current_T=v2t(zeros(1,3));
	for i=1:size(U,1)
		u=U(i,1:3)';
		# this is P in the slides (thank bart)

		# HINT : current_T is the absoulute pose obtained by concatenating 
		# all the relative transformation until the current timestamp i
		current_T *= v2t(u);
		T(i,1:3)=t2v(current_T)';
	end
end