function [u,phi] = virtual_agent(xi,yi,xvi,yvi,vi,theta,v_lim,phi_max,a_lim)
u=0;
w=0;
phi=0;
g=9.81;
kp_airspeed=1;
if vi<v_lim(1) || vi>v_lim(1)
    a=kp_airspeed*(u_min-vi);
    phi=0;
    u=a;

    return
else
    x=xvi-xi;
    y=yvi-yi;
    rho=sqrt(x^2+y^2);
    alpha=atan2(y,x)-theta;
    beta=-1*alpha-theta;
    a=alpha+beta+rho;

end
u=max(min(a,a_lim(2)),a_lim(1));
phi=-1*arctan(w*vi/g);
phi=max(min(phi,phi_max),-1*phi_max);
end