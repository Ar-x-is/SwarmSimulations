function out = generate_force(q, p, sys_params)

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
        out = (1/epsilon) * (sqrt(1 + epsilon * vecnorm(z, 2, 2).^2) - 1);
    end
    %------------------------------------------------------------------
    %-------------------------SYSTEM PARAMTERS-------------------------

    %------------------------------------------------------------------
    %-------------------------EVALUATE FORCE---------------------------

    %------------------------------------------------------------------

end