function y = update_agents(x, delta_t, L)
   
    y = (x' - L*delta_t*x')';

end