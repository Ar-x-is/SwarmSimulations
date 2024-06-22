function [positions, velocities] = generate_agents(n, bounds)

    d_min = bounds(1);
    d_max = bounds(2);
    v_min = bounds(3);
    v_max = bounds(4);
    
    % Generate random positions within the specified bounds
    positions = (d_max - d_min) * rand(n, 2) + d_min;

    % Generate random velocities with magnitudes within the specified bounds
    angles = 2 * pi * rand(n, 1);
    magnitudes = (v_max - v_min) * rand(n, 1) + v_min;
    velocities = magnitudes .* [cos(angles), sin(angles)];

    positions = positions;
    velocities = velocities;

end