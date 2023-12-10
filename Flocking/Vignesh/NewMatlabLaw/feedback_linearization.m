function [a,phi]=feedback_linearization(ux,uy,vi,theta,v_lim,phi_max,a_lim,g)
% feedback_linearization Produces bank and acceleration commands for a
% fixed wing UAV given cartesian coordinates and performance limits,
% enforces stall limits 
%a=0;
%w=0;
%phi=0;
%kp_airspeed=1;
aw=[cos(theta),sin(theta);
        -1*sin(theta)/vi, cos(theta)/vi]*[ux;uy];
a=aw(1);
w=aw(2);
phi=atan(w*vi/g);
% if vi<v_lim(1)
%     a=0.5*(v_lim(1)-vi);
%     %a=0;
% elseif vi>v_lim(2)
%     a=0.5*(v_lim(2)-vi);
%     %a=0;
% end
% if abs(mod(atan2(uy,ux)-theta+2*pi,2*pi)-pi)<(20*pi/180) && norm([ux;uy])>abs(a_lim(1))
%     if mod(atan2(uy,ux)-theta+2*pi,2*pi)>=pi
%         phi=-1*phi_max;
%     else
%         phi=phi_max;
%     end
% end
% a=max(min(a,a_lim(2)),a_lim(1));
% phi=max(min(phi,phi_max),-1*phi_max);
end
