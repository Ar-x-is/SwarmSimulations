function [u,phi]=feedback_linearization2(ux,uy,vi,theta,v_lim,phi_max,a_lim,tv)
%u=0;
%w=0;
%phi=0;
g=9.81;
kp_airspeed=1;
if vi<v_lim(1) || vi>v_lim(1)
    u=tv*kp_airspeed*(u_min-vi)+vi;
    phi=0;
    return
else
    aw=[cos(theta),sin(theta);
        -1*sin(theta)/vi, cos(theta)/vi]*[ux;uy];
    a=aw(1);
    w=aw(2);
end
a=max(min(a,a_lim(2)),a_lim(1));
u=vi+a*tv;
phi=-1*arctan(w*vi/g);
phi=max(min(phi,phi_max),-1*phi_max);
end
