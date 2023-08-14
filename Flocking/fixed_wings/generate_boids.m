function [positions, velocities] = generate_boids(num_boids, range)
% Initialize the boids' positions and velocities
positions = range*rand(num_boids, 2); % Random positions between [0, range]
velocities = range*rand(num_boids, 2) - range/2; % Random velocities [-range/2, range/2]
end