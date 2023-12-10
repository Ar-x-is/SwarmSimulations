function [dxi,dyi,dvi,dtheta,dphi,dai] = UAV(vi,theta,phi,ai,phi_c,a_c,v_lim,a_lim,phi_max,tp,ta,wind)
%UAV Models the Planar Kinematics of a fixed wing UAV with first order
%dynamics on acceleration and roll angle
% TTD: V^2 bank angle limit, better stall recovery
    g=9.800665;
    dxi=vi*cos(theta)+wind(1);
    dyi=vi*sin(theta)+wind(2);
    dvi=ai;
    w=yaw_rate(phi,vi);
    dtheta=w;
%     if vi<v_lim(1)
% %         warning('Stall Speed Limit')
% %         disp(vi)
%         a_c=a_lim(2);
%     elseif vi>v_lim(2)
% %         warning('Max Speed Limit')
% %         vi
%         a_c=a_lim(1);
%     end
%     phi_c=constraint(phi_c,[-phi_max,phi_max]);
%     a_c=constraint(a_c,a_lim);
    dphi=(phi_c-phi)/tp;
    dai=(a_c-ai)/ta;
    


    function z_new=constraint(z,z_lim)
        z_new=max(min(z_lim(2),z),z_lim(1));
    end
    function w_r=yaw_rate(phi,v)
        w_r=g*tan(phi)/v;
    end
end