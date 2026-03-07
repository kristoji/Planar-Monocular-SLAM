function plot_initial_guess(XL_guess, XL_true, guessed)
    global num_landmarks;

    % plot all the triangulated points and ground truth
    h2 = figure(2);
    hold on;

    % compute avg error for all landmarks
    total_error = 0;
    error_history = zeros(num_landmarks, 1);
    for i=1:num_landmarks,
        if ~guessed(i),
            error_history(i) = NaN; % mark unguessed landmarks as NaN
            continue;
        end
        err = norm(XL_guess(:, i) - XL_true(:, i));
        total_error = total_error + err;
        error_history(i) = err;
    end
    avg_error = total_error / sum(guessed);
    subplot(2,1,1);
    hold on;
    title(sprintf('Triangulation Errors for All Landmarks (Avg: %.2f m)', avg_error));
    % plot(error_history, '-');
    % mark unguessed landmarks with red x
    for i=1:num_landmarks,
        if ~guessed(i),
            plot(i, 0, 'rx', 'linewidth', 2, 'DisplayName', 'Unguessed Landmark');
        else
            plot(i, error_history(i), 'bo', 'linewidth', 2, 'DisplayName', 'Guessed Landmark');
        end
    end
    xlabel('Landmark Index');
    ylabel('Triangulation Error (m)');
    subplot(2,1,2);
    hold on;
    title(sprintf('Triangulated Points for All Landmarks, Guessed: %d / %d', sum(guessed), num_landmarks));

    plot3(XL_true(1,:), XL_true(2,:), XL_true(3,:), 'ro', 'linewidth', 2, 'DisplayName', 'Ground Truth');
    XL_guess_guessed = XL_guess(:, guessed>0);
    plot3(XL_guess_guessed(1,:), XL_guess_guessed(2,:), XL_guess_guessed(3,:), 'gx', 'linewidth', 2, 'DisplayName', 'Initial Guess');
    
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    legend();
    saveas(h2, '../imgs/triangulation_initial-guess.png');

    % print indexes of guessed landmarks and their errors on file
    initial_guess_file = fopen('../data/initial_guess_errors.dat', 'w');
    for i=1:num_landmarks,
        if guessed(i),
            fprintf(initial_guess_file, '%d %f %f %f\n', i, XL_guess(1,i), XL_guess(2,i), XL_guess(3,i));
        end
    end
    fclose(initial_guess_file);

end