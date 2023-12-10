function [r,d] = flock_params(t,d1,d2,k,t0,tspan)
% Author : Vignesh Anand, vignesh_aero@iitb.ac.in
% flock_params : returns the command neighbour distance 'd' and cutoff
% distance 'r'. Based on linear variation of d from d1 to d2
% INPUTS
% Curent time (scalar) t
% Initial neighbour distance (scalar>0) d1
% Final neighbour distance (scalar>0) d2
% Flocking ratio (2>scalar>1) k
% Time at which to start variation (scalar>0) t0
% Duration of variation (scalar>0) tspan
% OUTPUTS
% Cutoff distance (scalar) r
% Neighbour distance (scalar) d

if t<t0
d=d1;
r=d*k;
elseif t<t0+tspan
    d=d1-(t-t0)*(d2-d1)/tspan;
    r=d*1.4;
else
    d=d2;
    r=k*d;
end
end