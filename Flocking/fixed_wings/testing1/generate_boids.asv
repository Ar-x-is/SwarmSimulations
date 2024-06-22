function [positions, velocities] = generate_boids(num_boids, range, min_d)
    % Check if the range is too small for the specified number of boids
    if range / num_boids < min_d
        error('The specified range is too small to accommodate the specified number of boids with the given minimum distance.');
    end
    
    % Initialize the boids' positions and velocities
    positions = zeros(num_boids, 2);
    velocities = zeros(num_boids, 2);
    
    % Generate boids with minimum distance constraint
    for i = 1:num_boids
        % Generate a position until it satisfies the minimum distance constraint
        while true
            position = range*rand(1, 2); % Random position between [0, range]
            % Check if the new position satisfies the minimum distance constraint with existing positions
            if all(pdist2(position, positions) >= min_d)
                positions(i, :) = position;
                break; % Exit the loop if the condition is satisfied
            end
        end
        % Assign random velocities [-range/2, range/2]
        velocities(i, :) = range*rand(1, 2) - range/2;
    end
end
