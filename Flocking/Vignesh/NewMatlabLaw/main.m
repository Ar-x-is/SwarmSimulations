%% Initalise State and Params
simparams
[p0,v_0,theta_0,phi0,a0]=initialise_agents(N,x0_lim,y0_lim,v0_lim,theta0_lim,phi0_lim,a0_lim,d_0);
v_0;
theta_0*180/pi;

% State of each agent [x,y,v,theta,phi,a]'
state_0=zeros(6*N,1);
i=0;
%p0=[0;0;90;0;180;0;0;90;90;90;180;90;0;180;90;180;180;180];
%theta_0=[pi,0];
state_0(1:6:end)=p0(1:2:end);
state_0(2:6:end)=p0(2:2:end);
state_0(3:6:end)=v_0;
state_0(4:6:end)=theta_0;
state_0(5:6:end)=phi0;
state_0(6:6:end)=a0;
%state_0
options=odeset('RelTol',1e-3,'AbsTol',1e-2);
%% Test Function
%dydt=odefcn(10,state_0,N,d1,d2,k,t0,t_change,d_min,h,eps,a,b,c1,c2,v_lim,v_lim_ult,a_lim,g,phi_max,ta,tp,wps,v_nav,loiter_radius,wind)
%% solve ODE45
[t,y]=ode45(@(t,y) odefcn(t,y,N,d1,d2,k,t0,t_change,d_min,h,eps,a,b,c1,c2,v_lim,v_lim_ult,a_lim,g,phi_max,ta,tp,wps,v_nav,loiter_radius,wind),tspan,state_0,options);
%% ode45 function
function dydt=odefcn(t,y,N,d1,d2,k,t0,t_change,d_min,h,eps,a,b,c1,c2,v_lim,v_lim_ult,a_lim,g,phi_max,ta,tp,wps,v_nav,r_loiter,wind)
%tic
t
%y
% if t>500
%     d=d/2;
%     r=r/2;
% end
dydt=zeros(size(y));
p0=zeros(2*N,1);
p0(1:2:end)=y(1:6:end);

p0(2:2:end)=y(2:6:end);
vs=y(3:6:end);
thetas=y(4:6:end);
q0=zeros(2*N,1);
q0(1:2:end)=vs.*cos(thetas);
q0(2:2:end)=vs.*sin(thetas);
i=0;
%process_t=toc
while i<N
    %tic
    xi=y(6*i+1);
    yi=y(6*i+2);
    vi=y(6*i+3);
    thetai=y(6*i+4);
    phii=y(6*i+5);
    ai=y(6*i+6);
    state=[xi;yi;vi*cos(thetai);vi*sin(thetai)];
    [ref,~]=reference_trajectory(t,wps,v_nav,r_loiter);
    p=zeros(2*(N-1),1);
    q=zeros(2*(N-1),1);
    if i==0
        p(2*i+1:end)=p0(2*(i+1)+1:end);
        q(2*i+1:end)=q0(2*(i+1)+1:end);
    elseif i==N-1
        p(1:2*i)=p0(1:2*i);
        q(1:2*i)=q0(1:2*i);
    else
        p(1:2*i)=p0(1:2*i);
        p(2*i+1:end)=p0(2*(i+1)+1:end);
        q(1:2*i)=q0(1:2*i);
        q(2*i+1:end)=q0(2*(i+1)+1:end);
    end
    [r,d]=flock_params(t,d1,d2,k,t0,t_change);
    %process2_t=toc
    %tic
    [ux,uy]=FlockController(state,p,q,ref,r,d,d_min,h,eps,a,b,c1,c2);
    %flock_t=toc
    %tic
    %norm([ux;uy])
    [a_c,phi_c]=feedback_linearization(ux,uy,vi,thetai,v_lim,phi_max,a_lim,g);
    %fl_t=toc
    %tic
    [dxi,dyi,dvi,dtheta,dphi,dai] = UAV(vi,thetai,phii,ai,phi_c,a_c,v_lim_ult,a_lim,phi_max,tp,ta,wind);
    %uav_t=toc
    %tic
    dydt(6*i+1)=dxi;
    dydt(6*i+2)=dyi;
    dydt(6*i+3)=dvi;
    dydt(6*i+4)=dtheta;
    dydt(6*i+5)=dphi;
    dydt(6*i+6)=dai;
    i=i+1;
    %end_t=toc
end
end
%% 
% function dydt=odefcn2(t,y,N,r,d,d_min,h,eps,a,b,c1,c2,v_lim,v_lim_ult,a_lim,g,phi_max,ta,tp)
% t
% %y
% dydt=zeros(size(y));
% p0=zeros(2*N,1);
% p0(1:2:end)=y(1:6:end);
% p0(2:2:end)=y(2:6:end);
% %p0
% q0=zeros(2*N,1);
% q0(1:2:end)=y(3:6:end);
% q0(2:2:end)=y(4:6:end);
% ref=reference_trajectory(t);
% %q0
% i=0;
% while i<N
%     %i
%     xi=y(6*i+1);
%     yi=y(6*i+2);
%     vxi=y(6*i+3);
%     vyi=y(6*i+4);
%     phii=y(6*i+5);
%     ai=y(6*i+6);
%     state=[xi;yi;vxi;vyi];
%     p=zeros(2*(N-1),1);
%     q=zeros(2*(N-1),1);
%     if i==0
%         p(2*i+1:end)=p0(2*(i+1)+1:end);
%         q(2*i+1:end)=q0(2*(i+1)+1:end);
%     elseif i==N-1
%         p(1:2*i)=p0(1:2*i);
%         q(1:2*i)=q0(1:2*i);
%     else
%         p(1:2*i)=p0(1:2*i);
%         p(2*i+1:end)=p0(2*(i+1)+1:end);
%         q(1:2*i)=q0(1:2*i);
%         q(2*i+1:end)=q0(2*(i+1)+1:end);
%     end
% %     %p0
% %     p
% %     %q0
% %     q
%     [ux,uy]=FlockController(state,p,q,ref,r,d,d_min,h,eps,a,b,c1,c2);
%     dydt(6*i+1)=vxi;
%     dydt(6*i+2)=vyi;
%     dydt(6*i+3)=ux;
%     dydt(6*i+4)=uy;
%     dydt(6*i+5)=0;
%     dydt(6*i+6)=0;
%     i=i+1;
% 
% end
% end