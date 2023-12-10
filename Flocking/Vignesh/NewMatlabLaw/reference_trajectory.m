function [ref,ts] = reference_trajectory(t,wps,v_nav,r)
%reference_trajectory returns a reference position and reference velocity
%for the flock
%INPUT
% Time t (scalar)
%OUTPUT
%Reference position ref (4X1)= [x;y;vx;vy]
% if t<100
%     ref=[20*t;0;20;0];
% else
%     ref=[2000-19*(t-100);(t-100);-19;1];
%w0=[0;0];
%w1=[10000;1000];
%w2=[0;2000];
% cpk=[9800;1000];
% c1=[9489.40796537;747.94328411];
% c2=[9489.40796537;1252.05671589];
% p1=[9469.50722156;946.95072216];
% p2=[9644.70398268;873.97164206];
% p3=[9644.70398268;1126.02835794];
% p4=[9469.50722156;1053.04927784];
% tp=[380.6694790800825;388.5793124842333;427.9370220813949;435.8468554855459;816.5163345656283];
% omega=v_nav/r;
% if t<tp(1)
%     ref=p2p(t,0,w0,p1,v_nav);
% elseif t<tp(2)
%     ref=circ(t,tp(1),c1,p1,r,v_nav,1);
% elseif t<tp(3)
%     ref=circ(t,tp(2),cpk,p2,r,v_nav,-1);
% elseif t<tp(4)
%     ref=circ(t,tp(3),c2,p3,r,v_nav,1);
% else
%     ref=p2p(t,tp(4),p4,w2,v_nav);
np=length(wps(1,:));
i=2;
ts=zeros(np,1);
while i<=np
    ts(i)=norm(wps(:,i)-wps(:,i-1))/v_nav+ts(i-1);
    i=i+1;
end
i=1;
while i<np
    if t<ts(i+1)
        ref=p2p(t,ts(i),wps(:,i),wps(:,i+1),v_nav);
        break
    end
    i=i+1;
end
if i==np
    ref=circ(t,ts(np),wps(:,np),2*wps(:,np)-wps(:,np-1),r,v_nav,-1);
end
end
function ref=p2p(t,t0,p1,p2,v_nav)
    % Return a point to point trajectory flown at a given speed
    % INPUT
    % Current time, starting time t,t0 (scalar)
    % Initial and final points (2x1) p1,p2
    % Navigation velocity (scalar) v_nav
    % OUTPUT
    % Reference position,velocity (4x1) ref=[x,y,vx,vy]'
    tet=atan2(p2(2)-p1(2),p2(1)-p1(1)); % Heading
    vx=v_nav*cos(tet); % x,y componetens of velocity
    vy=v_nav*sin(tet);
    t=t-t0;  % Elapsed time in trajecotry
    ref=[p1(1)+vx*t;p1(2)+vy*t;vx;vy];
end

function ref=circ(t,t0,c,p0,r,v_nav,clock)
    % Returns circular trajectory about a given point flown at a given speed
    % INPUT
    % Current time, starting time t,t0 (scalar)
    % Circle centre and Initil point (2x1) c,p0
    % Circle radius (scalar) r
    % Navigation velocity (scalar) v_nav
    % Trajectory direction +1=clockwise, -1=anti-clockwise (scalar) clock
    % OUTPUT
    % Reference position,velocity (4x1) ref=[x,y,vx,vy]'
    t=t-t0;
    clock=sign(clock);
    p0=p0-c;
    tet0=atan2(p0(2),p0(1));
    omega=v_nav/r;
    pos=[c(1)+r*cos(tet0-clock*omega*t);c(2)+r*sin(tet0-clock*omega*t)];
    vel=[r*clock*omega*sin(tet0-clock*omega*t);-1*r*clock*omega*cos(tet0-clock*omega*t)];
    ref=[pos(1);pos(2);vel(1);vel(2)];
end