%% File Name: ALOS3D_pred.m
% Author: Sebastian Zieglmeier
% Date last updated: 30.10.2025
% Description:
% Predictive variant of the Adaptive Line-of-Sight (ALOS) guidance law for
% 3-D waypoint tracking of autonomous underwater vehicles (AUVs).
% ALOS3D_pred computes the desired heading and pitch reference angles for a
% straight-line segment between consecutive waypoints, without using
% persistent variables. The internal adaptation parameters and active
% waypoint index are passed explicitly as inputs and returned as outputs,
% enabling multi-step prediction (e.g., within PALOS or DeePC frameworks).
%
% Sources:
% [1] - Sebastian Zieglmeier, et al., "Data-Enabled Predictive Control and 
%       Guidance for Autonomous Underwater Vehicles",
%       https://doi.org/10.48550/arXiv.2510.25309
% [2] - T. I. Fossen and P. Aguiar (2024).
%      "A Uniform Semiglobal Exponential Stable Adaptive Line-of-Sight (ALOS)
%      Guidance Law for 3-D Path Following." Automatica, 163, 111556.
%      doi:10.1016/j.automatica.2024.111556
%
% Usage:
%   [psi_ref, theta_ref, y_e, z_e, alpha_c_hat, beta_c_hat, ...
%       alpha_hat, beta_hat, k] = ...
%       ALOS3D_pred(x, y, z, Delta_h, Delta_v, gamma_h, gamma_v, ...
%                   M_theta, h, R_switch, wpt, alpha_hat, beta_hat, k)
%
% Inputs:
%   x, y, z       : Current AUV positions in the NED frame (m)
%   Delta_h       : Positive look-ahead distance, horizontal plane (m)
%   Delta_v       : Positive look-ahead distance, vertical plane (m)
%   gamma_h       : Adaptation gain, horizontal plane
%   gamma_v       : Adaptation gain, vertical plane
%   M_theta       : Bound on adaptation parameters
%                   (|alpha_c| < M_theta, |beta_c| < M_theta)
%   h             : Sampling time (s)
%   R_switch      : Switching radius for waypoint transition (m)
%   wpt           : Struct with waypoint arrays (wpt.pos.x, wpt.pos.y, wpt.pos.z)
%   alpha_hat     : Current vertical-plane adaptation parameter estimate
%   beta_hat      : Current horizontal-plane adaptation parameter estimate
%   k             : Current active waypoint index
%
% Outputs:
%   psi_ref       : Desired LOS heading angle (rad)
%   theta_ref     : Desired LOS pitch angle (rad)
%   y_e           : Cross-track error (m)
%   z_e           : Vertical-track error (m)
%   alpha_c_hat   : Current estimate of alpha_c (rad)
%   beta_c_hat    : Current estimate of beta_c (rad)
%   alpha_hat     : Updated alpha_hat parameter after one step
%   beta_hat      : Updated beta_hat parameter after one step
%   k             : Updated active waypoint index after distance check
%
% Notes:
% - ALOS3D_pred computes the same guidance law as ALOS3D, but all internal
%   states are handled externally. It is therefore fully deterministic and
%   suitable for predictive simulations.
% - The function is used by PALOS.m to generate predictive reference
%   trajectories consistent with the DeePC/MPC prediction horizon.
%



function [psi_ref, theta_ref, y_e, z_e, alpha_c_hat, beta_c_hat, alpha_hat, beta_hat, k] = ...
    ALOS3D_pred(x,y,z,Delta_h,Delta_v,gamma_h,gamma_v,M_theta,h,R_switch,...
    wpt, alpha_hat, beta_hat, k)




%% Initialization of (xk,yk,zk) and (xk_next,yk_next,zk_next), and integral state 
if k == 0  
  
    % Check if R_switch is smaller than the min. distance between the waypoints
    if R_switch > min( sqrt( diff(wpt.pos.x).^2 + diff(wpt.pos.y).^2 ...
            + diff(wpt.pos.z).^2) )
        error("The distances between the waypoints must be larger than R_switch");
    end
    
    % Check input parameters
    if (R_switch < 0)
        error("R_switch must be larger than zero");
    end
    if (Delta_h < 0 || Delta_v < 0)
        error("Delta_h and Delta_v must be larger than zero");
    end         
    
    alpha_hat = 0;        % initial states 
    beta_hat = 0;               

    k = 1;                % set first waypoint as the active waypoint
    xk = wpt.pos.x(k); 
    yk = wpt.pos.y(k);  
    zk = wpt.pos.z(k);     
    fprintf('Active waypoints:\n')
    % fprintf('  (x%1.0f,y%1.0f,z%1.0f) = (%.1f,%.1f,%.1f) \n',k,k,k,xk,yk,zk);

end

%% Read next waypoint (xk_next, yk_next, zk_next) from wpt.pos 
n = length(wpt.pos.x);
if k < n                        % if there are more waypoints, read next one 
    xk_next = wpt.pos.x(k+1);  
    yk_next = wpt.pos.y(k+1);   
    zk_next = wpt.pos.z(k+1); 
    xk = wpt.pos.x(k);
    yk = wpt.pos.y(k);
    zk = wpt.pos.z(k);
else                            % else, continue with last bearing and depth
    bearing = atan2((wpt.pos.y(n)-wpt.pos.y(n-1)), (wpt.pos.x(n)-wpt.pos.x(n-1)));
    R = 1e10;
    xk_next = wpt.pos.x(n) + R * cos(bearing);
    yk_next = wpt.pos.y(n) + R * sin(bearing);    
    zk_next = wpt.pos.z(n);
    xk = wpt.pos.x(k);
    yk = wpt.pos.y(k);
    zk = wpt.pos.z(k);
end

%% Compute azimuth ane elevation angles for straigh-line path
pi_h = atan2(  (yk_next-yk) , (xk_next-xk) );  
pi_v = atan2( -(zk_next-zk) , sqrt( (xk_next-xk)^2 + (yk_next-yk)^2 ) );  

% Along-track, cross-track and vertical-track errors (x_e, y_e, z_e) 
Rz = [ cos(pi_h) -sin(pi_h) 0
       sin(pi_h)  cos(pi_h) 0
       0 0 1 ];
Ry = [  cos(pi_v) 0 sin(pi_v)
        0 1 0
       -sin(pi_v) 0 cos(pi_v) ];

e = Ry' * Rz' * [x-xk, y-yk, z-zk]';
y_e = e(2);
z_e = e(3);

% If the next waypoint satisfy the switching criterion, k = k + 1
d = sqrt( (xk_next-x)^2 + (yk_next-y)^2 + (zk_next-z)^2 );
if (d < R_switch) && (k < n)
    k = k + 1;  
end

% ALOS guidance laws
psi_ref   = pi_h - beta_hat  - atan( y_e / Delta_h );
theta_ref = pi_v + alpha_hat + atan( z_e / Delta_v );

% ALOS parameter estimates with projection: alpha_hat[k+1], beta_hat[k+1]
alpha_c_hat = alpha_hat;
beta_c_hat  = beta_hat;
alpha_hat = alpha_hat + h * gamma_v * ...
    Delta_v / sqrt( Delta_v^2 + z_e^2 ) * proj(alpha_hat, z_e, M_theta);
beta_hat = beta_hat + h * gamma_h * ...
    Delta_h / sqrt( Delta_h^2 + y_e^2 ) * proj(beta_hat, y_e, M_theta);

end

% *************************************************************************
% Projection algorithm
% *************************************************************************
function y = proj(theta_hat, tau, M_theta)
    
eps = 0.001;
M_theta_hat = M_theta + eps;

if ( abs(theta_hat) > M_theta ) && ( theta_hat * tau > 0 )
    c = min(1, ( M_theta_hat^2 - M_theta^2) / (M_theta_hat^2 - M_theta^2) );
    y = (1 - c) * tau;
else
    y = tau;
end

end


