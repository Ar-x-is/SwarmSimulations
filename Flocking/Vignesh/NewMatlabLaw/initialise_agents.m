function [p,v,theta,phi,a]=initialise_agents(N,x_lim,y_lim,v_lim,theta_lim,phi0_lim,a0_lim,d_0)
% Function initalise_agents: randomly initialises postions and velocities
% of N agents withing given limits
% INPUTS
% Number of agents N (scalar)
% X positon bounds x_lim (2X1)
% Y positon bounds y_lim (2X1)
% X velocity bounds vx_lim (2X1)
% Y velocity bounds vy_lim (2X1)
% OUTPUTS
% Agent Positions (2Nx1) p=[x1,y1,x2,y2.....xn,yn]
% Agent Velocities (2Nx1) q=[vx1,vy1,vx2,vy2.....vxn,vyn]
px=rand(N,1)*(x_lim(2)-x_lim(1))+x_lim(1);
py=rand(N,1)*(y_lim(2)-y_lim(1))+y_lim(1);
dist=ones(N*(N-1)/2,1);
sums=1;
j=1;
while j<N
    k=j+1;
    while k<=N
        dist(sums)=sqrt((px(j)-px(k))^2+(py(j)-py(k))^2);
        k=k+1;
        sums=sums+1;
    end
    j=j+1;
end
dmin=min(dist);
while dmin<d_0
    px=rand(N,1)*(x_lim(2)-x_lim(1))+x_lim(1);
    py=rand(N,1)*(y_lim(2)-y_lim(1))+y_lim(1);
    dist=ones(N*(N-1)/2,1);
    sums=1;
    j=1;
    while j<N
        k=j+1;
        while k<=N
            dist(sums)=sqrt((px(j)-px(k))^2+(py(j)-py(k))^2);
            k=k+1;
            sums=sums+1;
        end
        j=j+1;
    end
    dmin=min(dist)
end
v=rand(N,1)*(v_lim(2)-v_lim(1))+v_lim(1);
theta=rand(N,1)*(theta_lim(2)-theta_lim(1))+theta_lim(1);
phi=rand(N,1)*(phi0_lim(2)-phi0_lim(1))+phi0_lim(1);
a=rand(N,1)*(a0_lim(2)-a0_lim(1))+a0_lim(1);
p=ones(2*N,1);
p(1:2:end,1)=px;
p(2:2:end,1)=py;


end