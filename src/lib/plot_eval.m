function plot_eval(ev_err, map_error, guessed)
    global num_landmarks;

    h5 = figure(5);
    subplot(2,2,1);
    hold on;
    title(sprintf('Pose Translation Errors (Avg: %.2f m)', mean(ev_err.trasl)));
    plot(ev_err.trasl, 'o-');
    xlabel('Pose Index');
    ylabel('Translation Error (m)');

    subplot(2,2,2);
    hold on;
    title(sprintf('Pose Rotation Errors (Avg: %.2f rad)', mean(ev_err.rot)));
    plot(ev_err.rot, 'o-');
    xlabel('Pose Index');
    ylabel('Rotation Error (rad)');

    subplot(2,2,3:4);
    axis off;
    text(0.1, 0.5, sprintf('Landmark Map avg RMSE: %.2f m\n', map_error), 'FontSize', 12);
    text(0.1, 0.3, sprintf('Number of Guessed Landmarks: %d / %d (%.2f%%)\n', sum(guessed), num_landmarks, sum(guessed) / num_landmarks * 100), 'FontSize', 12);
    saveas(h5, '../imgs/pose_errors.png');

end