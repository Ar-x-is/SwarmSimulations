function [ux,uy]=FlockController(state,p,q,ref,r,d,d_min,h,eps,a,b,c1,c2)
% Flock Controller generates cartesian coordinates for olfati flocking
% Reference:R. Olfati-Saber, "Flocking for multi-agent dynamic systems: algorithms and theory," 
% in IEEE Transactions on Automatic Control, vol. 51, no. 3, pp. 401-420, March 2006,
% doi: 10.1109/TAC.2005.864190. Link:https://ieeexplore.ieee.org/document/1605401
% INPUTS:
% given current agent state (4x1) state=[x;y;vx;vy]
% Neighbour Positions (2(N-1)x1) p=[x1,y1,x2,y2.....xn,yn]
% Neighbour Velocities (2(N-1)x1) q=[vx1,vy1,vx2,vy2.....vxn,vyn]
% Sensing Range r (scalar>d)
% Flocking Distance d (scalar>0)
% Bump function Parameter h (0<scalar<1)
% Sigma Norm Parameter eps (0<scalar)
% Phi function parameters a,b (0<a<b)
% Distance error navigation gain c1 (0<c1)
% Velocity error navigation gain c2 (0<c2)
% OUTPUTS:
% Cartesian Acclerations [ux,uy]

%     h=0.5;    % bump function paramater
%     eps=0.1;  % sigma norm parameter
%     a=5;      % phi function parameters
%     b=5;
%     c1=0.1;   % navigation controller gains c1-> position error gain, 
%     c2=0.6324; %c2-> velocity error gain
    U_con_sat=0.1;
    u_nav_sat=0.05;
    c=abs(a-b)/sqrt(4*a*b);
    i=1;
    u=navigation();  % Cartesian accelerations from PD Navigation controller
    while i<=length(p)/2    % Iterate through neighbour states
        pj=p(2*(i-1)+1:2*i);  % Slice for neighbour position and velocity
        qj=q(2*(i-1)+1:2*i);
%         i
        u_con=consensus(pj,qj);
        u_grad=gradient(pj);
        u=u+2*u_con+u_grad;  % Add contribution from consensus and gradient terms
        if norm(state(1:2)-pj)<d_min
            %warning('Near Collison')
            norm(state(1:2)-pj)
        end
        i=i+1;
    end
    ux=u(1);
    uy=u(2);
    function u_nav=navigation()
        % Navigation Controller
        % Uses a PD controller based on position and velocity error
        %Output Cartesian accelerates u_nav (2x1)=[ux;uy]
        u_nav=min(0.2,max(-0.2,-1*(c1*(state(1:2)-ref(1:2))+c2*(state(3:4)-ref(3:4)))));
    end
    function u_con=consensus(pj,qj)
        % Velocity Consensus Controller
        % Acceleration given by adjacency times the velocity error
        % INPUTS
        % Neighbour Position pj (2X1)
        % Neighbour Velocity qj (2X1)
        % OUTPUTS
        % Cartesian Accelrations u_con (2X1)
        u_con=0.2*adjacency(pj)*tanh(0.3*(qj-state(3:4)));
    end
    function u_grad=gradient(pj)
        % Gradient based controller
        % Acceleration given by gradient of potential function
        % INPUTS
        % Neighbour Position pj (2X1)
        % OUTPUTS
        % Cartesian Accelrations u_grad (2X1)
        pa=phi_alpha(sigma(norm(state(1:2)-pj)));
        sd=sigmad(pj-state(1:2));
        u_grad=pa*sd;
    end
    function adj=adjacency(pj)
        % Adjacency function is a smooth approximation of of the adjacency graph
        % INPUT
        % Neighbour Position pj (2X1)
        % OUTPUT
        % Adjacency function Adj (scalar)
        % adj=1 for d<<r , adj=0 d>r
        adj=p_h(sigma(norm(pj-state(1:2)))/sigma(r));
    end
    function ph=p_h(z)
        % Bump Function
        if z>=0 && z<h
            ph=1;
        elseif z>=h && z<1
            ph=(1+cos(pi*(z-h)/(1-h)))/2;
        else
            ph=0;
        end
    end
    function sig=sigma(z)
        % Sigma Norm
        sig=(sqrt(1+eps*norm(z))-1)/eps;
    end
    function sigd=sigmad(z)
        % Derivative of sigma norm
        sigd=z/(1+eps*sigma(z));
    end
    function phia=phi_alpha(z)
        % phi_alpha function
        phia=p_h(z/sigma(r))*phi_function(z-sigma(d));
        function phi=phi_function(x)
            phi=0.5*((a+b)*sigmad(x+c)+a-b);
            %change from python in eps, og=1
        end
    end



end