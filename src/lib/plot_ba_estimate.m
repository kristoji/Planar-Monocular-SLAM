function plot_ba_estimate(state, Traj, XL_true, guessed_XL_true, guessed_XL_est, guessed)
    % plot Xr and then Xl
    h4 = figure(4);
    subplot(1,2,1);
    hold on;
    title('Robot Trajectory');
    plot(Traj(:,5),Traj(:,6), 'r-', 'linewidth', 2, 'DisplayName', 'Ground Truth Trajectory');
    plot(Traj(:,2),Traj(:,3), 'b-', 'linewidth', 2, 'DisplayName', 'Initial Guess Trajectory');
    x = reshape(state.Xr(1,4,:), [], 1);
    y = reshape(state.Xr(2,4,:), [], 1);
    plot(x, y, 'g-', 'linewidth', 2, 'DisplayName', 'Estimated Trajectory');
    xlabel('X');
    ylabel('Y');
    legend();
    subplot(1,2,2);
    hold on;
    title('Landmark Positions');
    % plot seen ground truth landmarks in red and unseen in magenta
    unguessed_XL_true = XL_true(:, guessed==0);
    plot3(guessed_XL_true(1,:), guessed_XL_true(2,:), guessed_XL_true(3,:), 'ro', 'linewidth', 2, 'DisplayName', 'Guessed Ground Truth');
    plot3(unguessed_XL_true(1,:), unguessed_XL_true(2,:), unguessed_XL_true(3,:), 'mo', 'linewidth', 2, 'DisplayName', 'Unguessed Ground Truth');
    plot3(guessed_XL_est(1,:), guessed_XL_est(2,:), guessed_XL_est(3,:), 'gx', 'linewidth', 2, 'DisplayName', 'Estimated');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    legend();
    saveas(h4, '../imgs/ba_results.png');

end
