function plot_trajectories(positions)
    % Extract the number of agents (n) and time steps (T)
    [~, n, ~] = size(positions);

    % Define a set of colors for each agent
    colors = lines(n);

    % Create a figure
    figure;

    % Plot the trajectories for each agent
    for agent = 1:n
        plot(positions(:, agent, 1), positions(:, agent, 2), 'Color', colors(agent, :), 'LineWidth', 2);
        hold on;
    end

    % Set labels and title
    xlabel('X-axis');
    ylabel('Y-axis');
    title('Agent Trajectories (2D)');

    % Add a legend
    % legend(cellstr(num2str((1:n)', 'Agent %d')));

    % Plot a small cross at the start point
    plot(positions(1, :, 1), positions(1, :, 2), 'x', 'Color', 'k', 'MarkerSize', 8);

    % Hold off to stop overlaying subsequent plots
    hold off;
end