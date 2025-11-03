%% File Name: PALOS.m
% Author: Sebastian Zieglmeier
% Date last updated: 30.10.2025
% Description:
% Predictive Adaptive Line-of-Sight (PALOS) guidance law for 3-D waypoint 
% tracking of autonomous underwater vehicles (AUVs). This function predicts 
% future desired heading and pitch angles over a finite horizon, integrating 
% the classical ALOS algorithm with a predictive extension for use in 
% Data-Enabled Predictive Control (DeePC) and model predictive control frameworks.
%
% Sources:
% [1] - Sebastian Zieglmeier, et al., "Data-Enabled Predictive Control and 
%       Guidance for Autonomous Underwater Vehicles",
%       https://doi.org/10.48550/arXiv.2510.25309
%
% Usage:
%   [psi_d_vec, theta_d_vec, y_e, z_e, alpha_c_hat, beta_c_hat, k2] = ...
%       PALOS(x_pos, y_pos, z_pos, Delta_h, Delta_v, gamma_h, gamma_v, ...
%              M_theta, h, R_switch, wpt, x, T_fut, K_f)
%
% Inputs:
%   x_pos, y_pos, z_pos : Current AUV position in NED frame (m)
%   Delta_h             : Positive look-ahead distance, horizontal plane (m)
%   Delta_v             : Positive look-ahead distance, vertical plane (m)
%   gamma_h             : Positive adaptation gain, horizontal plane
%   gamma_v             : Positive adaptation gain, vertical plane
%   M_theta             : Max parameter bound: |alpha_c| < M_theta, |beta_c| < M_theta
%   h                   : Sampling time (s)
%   R_switch            : Switch to next waypoint inside circle of radius R_switch (m)
%   wpt                 : Waypoint struct with NED arrays
%   x                   : Full vehicle state vector (used here for velocities and attitude)
%   T_fut               : Prediction horizon length of predictive controller (integer ≥ 1)
%   K_f                 : LOS observer gain used in LOSobserver(·)
%
% Outputs:
%   psi_d_vec           : Predicted desired heading angles over horizon [1×T_fut] (rad)
%   theta_d_vec         : Predicted desired pitch angles over horizon  [1×T_fut] (rad)
%   y_e1                : Cross-track error (m) at the first predicted step
%   z_e1                : Vertical-track error (m) at the first predicted step
%   alpha_c_hat         : Estimate of alpha_c (rad) at the first predicted step
%   beta_c_hat          : Estimate of beta_c (rad) at the first predicted step
%   k2                  : Active waypoint index AFTER the first predicted step
%
% Internal (persistent) states:
%   alpha_hat1, beta_hat1 : Internal parameter estimates carried between calls
%   k1                    : Persistent active waypoint index
%   psi_d1, theta_d1      : Persistent filtered desired angles
%   r_d1,  q_d1           : Persistent observer states for yaw- and pitch-rate
%
%
% Notes:
% - PALOS calls ALOS3D_pred(...) each step to compute (psi_ref, theta_ref, y_e, z_e,
%   alpha_c_hat, beta_c_hat, alpha_hat, beta_hat, k) for the current or estimated position,
%   then filters references via:
%       [theta_d, q_d] = LOSobserver(theta_d, q_d, theta_ref, h, K_f);
%       [psi_d,   r_d] = LOSobserver(psi_d,   r_d,   psi_ref,   h, K_f);
% - The predicted position is advanced using current attitude/velocity to align guidance
%   with the predictive control horizon.
% - Persistent variables ensure smooth evolution across control steps. Use
%   clear PALOS to reinitialize (e.g., before a new mission)


function [psi_d_vec, theta_d_vec, y_e1, z_e1, alpha_c_hat1, beta_c_hat1, k2] = PALOS(x_pos, y_pos, z_pos, ...
    Delta_h, Delta_v, gamma_h, gamma_v, M_theta, h, R_switch, ...
    wpt, x, T_fut, K_f)

    % --- Persistent states to maintain continuity across calls ---
    persistent alpha_hat1 
    persistent beta_hat1 
    persistent k1
    persistent psi_d1
    persistent r_d1 
    persistent theta_d1
    persistent q_d1

    % --- One-time initialization and sanity checks ---
    if isempty(k1)   
  
        % Ensure R_switch is smaller than the minimum inter-waypoint distance
        if R_switch > min( sqrt( diff(wpt.pos.x).^2 + diff(wpt.pos.y).^2 ...
                + diff(wpt.pos.z).^2) )
            error("The distances between the waypoints must be larger than R_switch");
        end
        
        % Basic parameter validation
        if (R_switch < 0)
            error("R_switch must be larger than zero");
        end
        if (Delta_h < 0 || Delta_v < 0)
            error("Delta_h and Delta_v must be larger than zero");
        end         
        
        % Initialize persistent guidance/observer states
        k1 = 0; 
        alpha_hat1 = 0;        % initial states 
        beta_hat1 = 0;
        psi_d1 = x(12);
        r_d1 = 0;
        theta_d1 = 0;
        q_d1 = 0;
    
    end

    % --- Bring persistent states into local working variables ---
    alpha_hat = alpha_hat1;
    beta_hat = beta_hat1;
    k = k1;
    psi_d = psi_d1;
    r_d = r_d1;
    theta_d = theta_d1;
    q_d = q_d1;

    % --- Extract velocities/attitude used for prediction ---
    u = x(1);
    v = x(2);
    w = x(3);
    phi = x(10);

    % --- Preallocate reference sequences and predicted positions ---
    psi_d_vec = zeros(1, T_fut);
    theta_d_vec = zeros(1, T_fut);
    x_pred = x_pos;
    y_pred = y_pos;
    z_pred = z_pos;

    % --- Predictive loop over the horizon ---
    for i = 1:1:T_fut

        % Compute ALOS references at the current predicted position
         [psi_ref, theta_ref, y_e, z_e, alpha_c_hat, beta_c_hat, alpha_hat, beta_hat, k] = ...
            ALOS3D_pred(x_pred,y_pred,z_pred, ...
            Delta_h,Delta_v,gamma_h,gamma_v,M_theta,h,R_switch,...
            wpt, alpha_hat, beta_hat, k);
         
        % Filter references with LOS observers (smooths switching/steps)
        [theta_d, q_d] = LOSobserver(theta_d, q_d, theta_ref, h, K_f);
        [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, h, K_f);

        % Propagate actual position using current filtered references
        % (approximate forward Euler with current body velocities)
        R_rot = Rzyx(phi,theta_d,psi_d);
        pos = [x_pos;y_pos;z_pos] + h * R_rot * [u; v; w];
        x_pos = pos(1);
        y_pos = pos(2);
        z_pos = pos(3);

        % Predict forward position for NEXT ALOS evaluation (simple kinematics)
        x_pred = x_pred + v * h * cos(psi_d)*cos(theta_d); 
        y_pred = y_pred + v * h * sin(psi_d)*cos(theta_d);
        z_pred = z_pred + v * h * sin(theta_d);

        % Store horizon references
        psi_d_vec(i) = psi_d;
        theta_d_vec(i) = theta_d;

        % After first step: update persistent states and log outputs
        if i == 1
            if k ~= k1
                fprintf('  (x%1.0f,y%1.0f,z%1.0f) = (%.1f,%.1f,%.1f) \n',k,k,k,wpt.pos.x(k),wpt.pos.y(k),wpt.pos.z(k));
            end
            alpha_hat1 = alpha_hat;
            beta_hat1 = beta_hat;
            k1 = k;
            psi_d1 = psi_d;
            r_d1 = r_d;
            theta_d1 = theta_d;
            q_d1 = q_d;
            k2 = k;
            y_e1 = y_e;
            z_e1 = z_e;
            alpha_c_hat1 = alpha_c_hat;
            beta_c_hat1 = beta_c_hat;
         end

    end

end
