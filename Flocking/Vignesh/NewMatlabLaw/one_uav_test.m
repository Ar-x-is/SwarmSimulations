% State= [x,y,,v,theta,phi,a]
y0=[0,0,20,-3*pi/4,0,0];
[t,y]=ode45(@(t,y) odefcn(t,y,v_lim,v_lim_ult,a_lim,phi_max,g,tp,ta),tspan,y0);
plot(y(:,1),y(:,2))
axis('equal')
function dydt=odefcn(t,y,v_lim,v_lim_ult,a_lim,phi_max,g,tp,ta)
ref=reference_trajectory(t);
xi=y(1);
yi=y(2);
vi=y(3);
theta=y(4);
phi=y(5);
ai=y(6);
xs=[xi;yi];
vs=[vi*cos(theta);vi*sin(theta)];
kp=0.1;
kd=0.6324;
u=kp*(ref(1:2)-xs)+kd*(ref(3:4)-vs);
[a_c,phi_c]=feedback_linearization(u(1),u(2),vi,theta,v_lim,phi_max,a_lim,g);
[dxi,dyi,dvi,dtheta,dphi,dai] = UAV(vi,theta,phi,ai,phi_c,a_c,v_lim_ult,a_lim,phi_max,tp,ta);
dydt=[dxi;dyi;dvi;dtheta;dphi;dai];
end