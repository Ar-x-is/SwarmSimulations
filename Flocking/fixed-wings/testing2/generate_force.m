function [acceleration, omega] = generate_force(q, p, sys_params)

    [n, ~] = size(q);

    acceleration = zeros(n,2);
    omega = zeros(n,1);

    %---------------------------PRELIMINARIES-------------------------
    % This is developed in Olfati's work on general flocking
    function out = sigmaEpsilon(z, epsilon)
        out = z ./ sqrt(1 + epsilon*(vecnorm(z, 2, 2)));
    end

    function rho = bumpFunction(z, h)
        rho = zeros(size(z));
        rho(z >= 0 & z < h) = 1;
        rho(z >= h & z <= 1) = (1/2) * (1 + cos(pi * ((z(z >= h & z <= 1) - h)/(1 - h))));
    end

    function out = phi(z, a, b) 
        % merely scaling and shifting the sigmaEpsilon function
        c = abs(a-b)/sqrt(4*a*b);
        out = 0.5 * ((a+b) * sigmaEpsilon(z+c, 1) + (a-b));
    end

    function out  = phi_alpha(z, r_alpha, d_alpha, a, b)
        out = bumpFunction(z/r_alpha, 0.2) .* phi(z-d_alpha, a, b);
    end

    function out = sigmaNorm(z, epsilon)
        if epsilon == 0
            out = 1/2 * vecnorm(z, 2, 2);
        else
            out = (1/epsilon) * (sqrt(1 + epsilon * vecnorm(z, 2, 2).^2) - 1);
        end
    end

    % find the angle between two vectors, taking x along the first
    function out = angle(v1, v2)
        cross = v1(2)*v2(1) - v1(1)*v2(2);
        dot = sum(v1 .* v2);
        out = atan2(cross, dot);
        if cross == 0 && dot < 0
            out = pi;
        end
    end
    %------------------------------------------------------------------
    %-------------------------SYSTEM PARAMTERS-------------------------
    r = sys_params(1);
    d = sys_params(2);
    epsilon = sys_params(3);
    %------------------------------------------------------------------
    %-------------------------EVALUATE FORCE---------------------------
    % The force is given by the turning and linear accelerations
    % Olfati - alpha lattice formation force
    directions = p ./ vecnorm(p,2,2);
    distances = pdist2(q, q);
    adjacency_matrix = bumpFunction(distances/r, epsilon) - eye(size(q,1));
    target_heading_vector = sum(directions);
    target_heading_vector = target_heading_vector / vecnorm(target_heading_vector,2,2);
    target_heading_vector = target_heading_vector + [0,1];

    for i = 1:n
        alpha_force = 0;
        for j = 1:n
            i;
            j;
            displacement = q(i,:) - q(j,:);
            distance = vecnorm(displacement, 2, 2);
            n_ij = sigmaEpsilon(displacement, epsilon);
            alpha_force = alpha_force + phi_alpha(sigmaNorm(displacement, ...
                epsilon), r, d, 5, 10) .* n_ij;
        end
        alpha_acceleration = sum(alpha_force .* directions(i,:));
        alpha_turning = vecnorm(alpha_force - alpha_acceleration * ...
            directions(i,:), 2, 2)/vecnorm(p(i,:),2,2);
        turning = 10/pi * angle(p(i,:), target_heading_vector);
        if target_heading_vector == 0
            turning = 10;
        end

        acceleration(i,:) = alpha_acceleration * directions(i,:);
        omega(i) = alpha_turning + turning;
    end
    %------------------------------------------------------------------

end