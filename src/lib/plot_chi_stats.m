function plot_chi_stats(chi_stats)

    h3 = figure(3);
    subplot(1,2,1);
    hold on;
    title('Bundle Adjustment Chi Statistics');
    plot(chi_stats.chi_tot, 'o-');
    xlabel('Iteration');
    ylabel('Total Chi');
    subplot(1,2,2);
    hold on;
    title('Bundle Adjustment Inliers');
    plot(chi_stats.num_inliers, 'o-');
    xlabel('Iteration');
    ylabel('Number of Inliers');
    saveas(h3, '../imgs/chi_stats.png');

end