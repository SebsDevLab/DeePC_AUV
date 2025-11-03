%% File Name: SIMremus100_with_DeePC.m
% Author: Sebastian Zieglmeier 
% Date last updated: 30.10.2025
%
% Description:
% Comprehensive simulation framework for the REMUS 100 Autonomous
% Underwater Vehicle (AUV) combining classical and data-driven control from
% Fossens MSS toolbox.
% This script extends the original MSS Remus100 simulator with hierarchical
% Data-Enabled Predictive Control (DeePC) for depth and heading regulation,
% additionally integrated with the Predictive Adaptive Line-of-Sight (PALOS)
% guidance law for 3-D waypoint tracking.
%
%
% New Dependencies:
%   remus100_Seb.m            - Vehicle dynamics model (6-DOF)
%   DeePC_fast.m              - Fast Data-Enabled Predictive Control class
%   make_page_MIMO.m/make_hankel_MIMO - Construction of page matrices (Hankel)
%   ALOS3D_pred.m / PALOS.m   - Predictive ALOS guidance algorithms
%
%
%
% Reference Papers:
%   - S. Zieglmeier et al. (2025). "Semi-Data-Driven Model Predictive
%     Control for Autonomous Underwater Vehicles," ArXiv preprint.
%
% Notes:
%   - Designed for research on DeePC for AUVs.
%   - The script collects PRBS/chirp-based identification data to build
%     Page matrices for DeePC.
%
% -------------------------------------------------------------------------
% Simulation starts below
% -------------------------------------------------------------------------

clearvars;                          % Clear all variables from memory
clear ALOS3D;                       % Clear persistent states in controllers
clear PALOS;
close all;                          % Close all open figure windows
%% SIMULATOR CONFIGURATION
h = 0.05;                           % Sampling time (s)
T_final = 1500;	                    % Final simulation time (s)
% Time vector initialization
t = 0:h:T_final;                % Time vector from 0 to T_final          
nTimeSteps = length(t);         % Number of time steps

% Choose Control approach via GUI
[ControlFlag, DisturbanceFlag, Load_from_File] = controlMethod_with_DeePC();

% Define waypoints for 3D path following
wpt.pos.x = [0  -20 -100   0  200, 200  400];
wpt.pos.y = [0  200  600 950 1300 1800 2200];
wpt.pos.z = [0   10  100 100   50   50   50];

% Initialize position and orientation
xn = 0; yn = 0; zn = 0;             % Initial North-East-Down positions (m)
phi = 0; theta = 0; psi = 0;        % Initial Euler angles (radians)
U = 1;                              % Initial speed (m/s)

% First state-vector:
x = [U; zeros(5,1); xn; yn; zn; phi; theta; psi];

% Initialize ocean current parameters
Vc = 0;                         % Horizontal speed (m/s)
betaVc = deg2rad(0);            % Horizontal direction (radians)
wc = 0;                         % Vertical speed (m/s)

% Initialize propeller dynamics
n_max = 1525;                  % Maximum propeller speed (RPM)
n = 1000;                      % Initial propeller speed (RPM)

%% CONTROL SYSTEM CONFIGURATION
delta_max = deg2rad(20);       % Maximum rudder and stern angle (rad)

Kp_theta = 5.0;                % Proportional gain for pitch control
Kd_theta = 2.0;                % Derivative gain for pitch control
Ki_theta = 0.3;                % Integral gain for pitch control

Kp_z = 0.1;                    % Proportional gain for depth
T_z = 100;                     % Time constant for integral action in depth control

Kp_psi = -7.5;                 % Proportional gain for heading control
Ki_psi = -0.75;                % Integral gain for heading control
Kd_psi = -15;                  % Derivative gain for heading control

%% INPUT RANGE CHECK
rangeCheck(n, 0, n_max); 
rangeCheck(U, 0, 5);

%% ------------------------------------------------------------------------
%% ------------------------------------------------------------------------
%% Initialize Variables and control hyperparameters for Heading control
T_fut_heading = 10;        % Prediction horizon
T_d_heading = 100;         % T_d is the number of data points used to build the Hankel matrix
T_ini_heading = 6;         % Number of past values of DeePC
T_c_heading = T_d_heading*(T_fut_heading+T_ini_heading); % Number of data points collected for big Hankel matrix

% Cost function parameters of the hybrid predictive controller
lambda_ini_heading = 1e7;
lambda_g_heading = 1e3;
R_heading = 1e-1;
Q_heading = 1e4;

sys_heading.nu = 1;
sys_heading.ny = 1;

%% Initialize Variables and control hyperparameters for Inner Depth control

T_fut_pitch = 6;         % Prediction horizon
T_d_pitch = 100;         % T_d is the number of data points used to build the Hankel matrix
T_ini_pitch = 5;         % Number of past values of DeePC
T_c_pitch = T_d_pitch*(T_fut_pitch+T_ini_pitch); % Number of data points collected for big Hankel matrix

% Cost function parameters of the hybrid predictive controller
lambda_ini_pitch = 1e7;
lambda_g_pitch = 1e2;
% R_pitch = 5e2*ones(1, 1);  % Pure DeePC
% Q_pitch = 1e4*ones(1, 1);
R_pitch = 1e2*ones(1, 1);    % For the use of DeePC+PALOS
Q_pitch = 1e2*ones(1, 1);

sys_pitch.nu = 1;
sys_pitch.ny = 1;

%% DATA COLLECTION LOOP for inner Depth Loop and Heading control
if Load_from_File == 1
    cacheFile = 'Inner_loop_sequences.mat';
    load(cacheFile, 'u_data_delta_r', 'y_data_psi', 'u_data_delta_s', 'y_data_theta');
    disp(['Loaded cached data from ', cacheFile]);
else
    T_c_inner_loop = max(T_c_pitch, T_c_heading); % Number of data points collected for data matrix of inner loops
    f0 = 0.001;
    f1 = 0.05;
    u_data_ripple_pitch = 0.3*delta_max * idinput([T_c_inner_loop, sys_pitch.nu]);  % PRBS for delta_s
    u_data_chirp_pitch  = 0.7*delta_max * chirp(h:h:T_c_inner_loop*h, f0 ,T_c_inner_loop*h, f1);
    u_data_pitch = sat(u_data_chirp_pitch' + u_data_ripple_pitch, delta_max);
    
    u_data_ripple_heading = 0.3*delta_max*idinput([T_c_inner_loop, sys_heading.nu]);  % PRBS for delta_s
    u_data_chirp_heading = 0.7*delta_max*[linspace(1,1, T_c_inner_loop/5), linspace(1,-1, T_c_inner_loop*2/5), linspace(-1,-1, T_c_inner_loop*2/5)];
    u_data_heading = sat(u_data_chirp_heading' + u_data_ripple_heading, delta_max);
    
    collectData_inner = zeros(T_c_inner_loop, length(x) + 3); % Preallocate table for simulation data
    
    for i = 1:T_c_inner_loop    
        % Depth inner loop
        delta_s = u_data_pitch(i);
        delta_s = sat(delta_s, delta_max);
    
        % Heading autopilot using the tail rudder
        delta_r = u_data_heading(i);
        delta_r = sat(delta_r, delta_max);
       
        % Propeller speed
        n = 1000;
        ui = [delta_r delta_s n]';            
        
        % Store collected data in a table for the controller 
        collectData_inner(i,:) = [ui' x'];
        
        % Simulate the remus model to collect the data
        x = rk4(@remus100_updated_OC, h, x, ui, Vc, betaVc, wc);  % RK4 method x(k+1)
        
    end
    % Euler angles x = [ u v w p q r x y z phi theta psi ]'
    u_data_delta_s = collectData_inner(:,2);
    y_data_theta = collectData_inner(:, 3+11);
    
    u_data_delta_r = collectData_inner(:,1);
    y_data_psi = collectData_inner(:, 3+12);

    % cacheFile = 'Inner_loop_sequences.mat';
    % save(cacheFile, 'u_data_delta_r', 'y_data_psi', 'u_data_delta_s', 'y_data_theta');
    % disp(['Saved data on', cacheFile]);
end
u_data_pitch = u_data_delta_s;
y_data_pitch = y_data_theta;

u_data_heading = u_data_delta_r;
y_data_heading = y_data_psi;

% figure
% subplot(2,1,1)
% plot(h:h:T_c_inner_loop*h,rad2deg(u_data_pitch))
% xlabel("t in s")
% ylabel("delta_s in deg")
% title("Stern angle")
% subplot(2,1,2)
% plot(h:h:T_c_inner_loop*h, rad2deg(y_data_pitch))
% xlabel("t in s")
% ylabel("theta in deg")
% title("Pitch angle")
% 
% figure
% subplot(2,1,1)
% plot(h:h:T_c_inner_loop*h, rad2deg((u_data_heading)));
% xlabel("t in s")
% ylabel("delta_r in deg")
% title("Rudder angle")
% subplot(2,1,2)
% plot(h:h:T_c_inner_loop*h, rad2deg((y_data_heading)));
% xlabel("t in s")
% ylabel("psi in deg")
% title("Yaw angle")


%% Build Hankel matrizes for Heading Control
L_heading = T_ini_heading + T_fut_heading; % Lag for right size of Hankel matrizes
num_hankel_cols_heading = T_d_heading - L_heading + 1;

H_u_heading = make_page_MIMO(u_data_heading', num_hankel_cols_heading, L_heading);
H_y_heading = make_page_MIMO(y_data_heading', num_hankel_cols_heading, L_heading);

sys_heading.constraints.u_min = (-delta_max);
sys_heading.constraints.u_max = (delta_max);
sys_heading.constraints.y_min = -pi;
sys_heading.constraints.y_max = pi;

%% Build Hankel matrizes for inner Loop - Depth Control
L_pitch = T_ini_pitch + T_fut_pitch; % Lag for right size of Hankel matrizes
num_hankel_cols_pitch = T_d_pitch - L_pitch + 1;

H_u_pitch = make_page_MIMO(u_data_pitch', num_hankel_cols_pitch, L_pitch);
H_y_pitch = make_page_MIMO(y_data_pitch', num_hankel_cols_pitch, L_pitch);

sys_pitch.constraints.u_min = (-delta_max);
sys_pitch.constraints.u_max = (delta_max);
sys_pitch.constraints.y_min = deg2rad(-30);
sys_pitch.constraints.y_max = deg2rad(30);

%% ------------------------------------------------------------------------
%% Initialize everything to zero again:
% Initialize position and orientation
xn = 0; yn = 0; zn = 0;             % Initial North-East-Down positions (m)
phi = 0; theta = 0; psi = 0;        % Initial Euler angles (radians)
U = 1;                              % Initial speed (m/s)

x = [U; zeros(5,1); xn; yn; zn; phi; theta; psi];

% Initialize ocean current parameters
Vc = 0;                      % Horizontal speed (m/s)
betaVc = deg2rad(0);          % Horizontal direction (radians)
wc = 0;                      % Vertical speed (m/s)

% Initialize propeller dynamics
n = 1000;                      % Initial propeller speed (RPM)

% Setup for heading control
psi_step = deg2rad(0);       % Step change in heading angle (rad)

% INPUT RANGE CHECK
rangeCheck(n, 0, n_max); 
rangeCheck(U, 0, 5);
%% ------------------------------------------------------------------------
% Initialize heading and pitch control
control_heading = DeePC_fast(T_d_heading, T_ini_heading, T_fut_heading, R_heading, Q_heading, (lambda_ini_heading), lambda_g_heading, H_u_heading, H_y_heading, sys_heading);
control_pitch = DeePC_fast(T_d_pitch, T_ini_pitch, T_fut_pitch, R_pitch, Q_pitch, (lambda_ini_pitch), lambda_g_pitch, H_u_pitch, H_y_pitch, sys_pitch);
%% Initialize Variables and control hyperparameters
T_fut = 7;         % Prediction horizon
T_d = 200;         % T_d is the number of data points used to build the Hankel matrix
T_ini = 7;         % Number of past values of DeePC
r_f = 10;     % decimation ratio between outer (depth) and inner (pitch) control loops
T_c = T_d*(T_fut+T_ini)*r_f; % Number of data points collected for big Hankel matrix

% Cost function parameters of the hybrid predictive controller
lambda_ini = 1e5;
lambda_g = 1e2;
R = 1e3*ones(1, 1);
Q = 1e2*ones(1, 1);

sys.nu = 1; 
sys.ny = 1; 
%% DATA COLLECTION LOOP for outer Loop - Depth Control
if Load_from_File == 1
    cacheFile = 'Depth_outer_loop_sequences.mat';
    load(cacheFile, 'u_data_theta_d', 'y_data_theta', 'y_data_z', 'u_data_delta_s');
    disp(['Loaded cached data from ', cacheFile]);
else
    f0 = 0.0001;
    f1 = 0.01;
    u_data_ripple_theta = 0.15*delta_max*idinput([T_c, sys.nu]);  % PRBS for delta_s
    u_data_chirp_theta = 0.7*delta_max * chirp(h:h:T_c*h, f0 ,T_c*h, f1);
    u_data_theta_d = u_data_ripple_theta - u_data_chirp_theta';
    psi_d = 0;
    theta_d = 0;
    collectData = zeros(T_c, length(x) + 4); % Preallocate table for simulation data
    
    u_past_sim_heading = zeros(T_ini_heading, sys_heading.nu);
    y_past_sim_heading = zeros(T_ini_heading, sys_heading.ny);
    u_past_sim_pitch = zeros(T_ini_pitch, sys_pitch.nu);
    y_past_sim_pitch = zeros(T_ini_pitch, sys_pitch.ny);
    for i = 1:T_c
        % Kinematic representation
    
        % Depth autopilot using the stern planes
        if mod(i,r_f) == 1
            theta_d_1 = u_data_theta_d(i);
            theta_d_vec = interp1([1 r_f+1], [theta_d; theta_d_1], linspace(1,r_f+1+T_fut_pitch,r_f+1+T_fut_pitch), 'linear', 'extrap');
            theta_runner = 1;
        end
        theta_runner = theta_runner + 1;
        theta_d = theta_d_vec(theta_runner);
        y_reference_pitch = theta_d_vec(theta_runner:theta_runner+T_fut_pitch-1);
        [u_fut_pitch, y_fut_pitch, g_fut_pitch, sigma_fut_pitch, opt_error_flag] = control_pitch.step(u_past_sim_pitch, y_past_sim_pitch, y_reference_pitch', H_u_pitch, H_y_pitch); 
        delta_s = u_fut_pitch(1);
        delta_s = sat(delta_s, delta_max);
    
        % Heading autopilot using the tail rudder
        y_reference_heading = psi_d * ones(1,T_fut_heading);
        [u_fut_heading, y_fut_heading, g_fut_heading, sigma_fut_heading, opt_error_flag] = control_heading.step(u_past_sim_heading, y_past_sim_heading, y_reference_heading', H_u_heading, H_y_heading); 
        delta_r = sat(u_fut_heading(1), delta_max);
        
        n = 1000;
    
        % Control input vector
        ui = [delta_r delta_s n]';            
        
        % Store collected data in a table for the controller 
        collectData(i,:) = [theta_d ui' x'];
        
        % Simulate the remus model to collect the data
        x = rk4(@remus100_updated_OC, h, x, ui, Vc, betaVc, wc);  % RK4 method x(k+1)
    
      
        
       phi = x(10); theta = x(11); psi = x(12); % Euler angles
       u_past_sim_heading = [u_past_sim_heading(2:end,:); [delta_r]];
       y_past_sim_heading = [y_past_sim_heading(2:end,:); psi];
    
       u_past_sim_pitch = [u_past_sim_pitch(2:end,:); [delta_s]];
       y_past_sim_pitch = [y_past_sim_pitch(2:end,:); theta];
       
    end
    u_data_theta_d = collectData(:,1);
    y_data_theta = collectData(:, 4+11);
    y_data_z = collectData(:, 4+9);
    u_data_delta_s = collectData(:, 3);

    % cacheFile = 'Depth_outer_loop_sequences.mat';
    % save(cacheFile, 'u_data_theta_d', 'y_data_theta', 'y_data_z', 'u_data_delta_s');
    % disp(['Saved data on ', cacheFile]);
end
y_data = y_data_z(r_f:r_f:end);
u_data = y_data_theta(r_f:r_f:end);
u_data2 = u_data_theta_d(r_f:r_f:end);
% t2 = h:h:T_c*h;
% t_data = t2(r_f:r_f:end);
% figure
% subplot(2,1,1)
% plot(t_data, rad2deg(u_data))
% hold on;
% plot(t_data, rad2deg(u_data2))
% xlabel("t in s")
% ylabel("theta_d in deg")
% title("Desired pitch angle")
% subplot(2,1,2)
% plot(t_data, y_data)
% xlabel("t in s")
% ylabel("z in m")
% title("Heave position")

%% Build Hankel matrizes for outer Loop - Depth Control
L = T_ini + T_fut; % Lag for right size of Hankel matrizes
num_hankel_cols = T_d - L + 1;

H_u = make_page_MIMO(u_data',num_hankel_cols, L);
H_y = make_page_MIMO(y_data',num_hankel_cols, L);

sys.constraints.u_min = (-deg2rad(30));
sys.constraints.u_max = (deg2rad(30));
sys.constraints.y_min = 0;
sys.constraints.y_max = 200;

%% ------------------------------------------------------------------------
%% ------------------------------------------------------------------------
%% Initialize everything to zero again:

% Initialize position and orientation
xn = 0; yn = 0; zn = 0;             % Initial North-East-Down positions (m)
phi = 0; theta = 0;                 % Initial Euler angles (radians)
if ControlFlag == 3 || ControlFlag == 5
    psi = atan2(wpt.pos.y(2) - wpt.pos.y(1), ...
        wpt.pos.x(2) - wpt.pos.x(1));   % Yaw angle towards next waypoint
else
    psi = 0;
end
U = 1;                              % Initial speed (m/s)

% Initial control and state setup
theta_d = 0; q_d = 0;               % Initial pitch references
psi_d = psi; r_d = 0; a_d = 0;      % Initial yaw references

% Integral states for autopilots
z_int = 0;                     % Integral state for depth control
theta_int = 0;                 % Integral state for pitch control
psi_int = 0;                   % Integral state for yaw control

% Initial low-pass filter state
xf_z_d = zn;                        
xf_x_d = xn;
xf_y_d = yn;

x = [U; zeros(5,1); xn; yn; zn; phi; theta; psi];

% Initialize ocean current parameters
Vc = 0;                      % Horizontal speed (m/s)
betaVc = deg2rad(0);          % Horizontal direction (radians)
wc = 0;                      % Vertical speed (m/s)

% Initialize propeller dynamics
n = 1000;                      % Initial propeller speed (RPM)


%% CONTROL SYSTEM CONFIGURATION

% Setup for depth and heading control
psi_step = -60;       % Step change in heading angle (rad)
z_step = 30;                   % Step change in depth, max 100 m

% Depth controller (suceessive-loop closure)
z_d = zn;                      % Initial depth target (m)
wn_d_z = 0.02;                 % Natural frequency for depth control

% Heading reference model
zeta_d_psi = 1.0;              % Desired damping ratio for yaw control
wn_d_psi = 0.1;                % Natural frequency for yaw control
r_max = deg2rad(5.0);          % Maximum allowable rate of turn (rad/s)

%% ALOS AND PALOS PARAMETERS
Delta_h = 20;               % Horizontal look-ahead distance (m)
Delta_v = 20;               % Vertical look-ahead distance (m)
gamma_h = 0.001;            % Adaptive gain, horizontal plane
gamma_v = 0.001;            % Adaptive gain, vertical plane
M_theta = deg2rad(20);      % Maximum value of estimates, alpha_c, beta_c

% Additional parameter for straigh-line path following
R_switch = 5;               % Radius of switching circle
K_f = 0.5;                  % LOS observer gain

%% INPUT RANGE CHECK
rangeCheck(n, 0, n_max); 
rangeCheck(U, 0, 5);

%% ------------------------------------------------------------------------
%% ------------------------------------------------------------------------
%% MAIN Control LOOP
simData = zeros(nTimeSteps, length(x) + 10); % Preallocate table for simulation data
alosData = zeros(nTimeSteps, 4); % Preallocate table for ALOS guidance data
control = DeePC_fast(T_d, T_ini, T_fut, R, Q, (lambda_ini), lambda_g, H_u, H_y, sys);

u_past_sim = zeros(T_ini, sys.nu);
y_past_sim = zn*ones(T_ini, sys.ny);
u_past_sim_heading = zeros(T_ini_heading, sys_heading.nu);
y_past_sim_heading = zeros(T_ini_heading, sys_heading.ny);
u_past_sim_pitch = zeros(T_ini_pitch, sys_pitch.nu);
y_past_sim_pitch = zeros(T_ini_pitch, sys_pitch.ny);

% reference generation
z_reference = zeros(1, nTimeSteps+T_fut);
psi_reference = zeros(1, nTimeSteps+T_fut);
z_ref2 = zeros(1, T_fut);
for i = 1:nTimeSteps+T_fut*r_f
    if i*h > 100
        z_reference(i) = z_step+10*sin(i*0.005);
        psi_reference(i) = deg2rad(60)+deg2rad(20*sin(i*0.01));
    elseif i*h > 10
        z_reference(i) = z_step;
        psi_reference(i) = deg2rad(60);
    else
        z_reference(i) = 0;
        psi_reference(i) = deg2rad(0);
    end
end

% Time counter for control loops
elapsedTime = 0;
elapsedTime_heading = 0;
elapsedTime_pitch = 0;
% Nulling for data-driven control
for i = 1:nTimeSteps

    % Measurements 
    q = x(5);                  % Pitch rate (rad/s)
    r = x(6);                  % Yaw rate (rad/s)
    xn = x(7);                 % North position (m)
    yn = x(8);                 % East position (m)
    zn = x(9);                 % Down position (m), depth
    theta = x(11); psi = x(12); % Euler angles
    % Control systems 
    switch ControlFlag
        case {1} % Depth command, z_ref, and heading angle command, psi_ref
            z_ref = z_reference(i);
            psi_ref = psi_reference(i);

            % LP filtering the depth command
            [xf_z_d, z_d] = lowPassFilter(xf_z_d, z_ref, wn_d_z, h);
            % Third-order reference model for the heading angle
            [psi_d, r_d, a_d] = refModel(psi_d, r_d, a_d, psi_ref, r_max,...
                zeta_d_psi, wn_d_psi, h, 1);

            % Depth autopilot using the stern planes 
            theta_d = Kp_z * ( (zn - z_d) + (1/T_z) * z_int );     % PI
            delta_s = Kp_theta * ssa( theta - theta_d )...        % PID
                + Kd_theta * q + Ki_theta * theta_int;
            delta_s = sat(delta_s, delta_max);
            
            % Heading autopilot using the tail rudder
            delta_r = Kp_psi * ssa( psi - psi_d )...        % PID
                + Ki_psi * psi_int + Kd_psi * r;
            delta_r = sat(delta_r, delta_max);

            % Ocean current dynamics
            if DisturbanceFlag == 1
                if t(i) > 10
                    Vc = 0.5;
                    wc = 0.05;
                    betaVc = deg2rad(150);
                else
                    Vc = 0;
                    wc = 0;
                    betaVc = deg2rad(150);
                end
                betaVc = betaVc + randn / 1000;
                Vc = Vc + 0.002 * randn;
                wc = wc + 0.001 * randn;
            end
            % Euler's integration method (k+1)
            z_int = z_int + h * ( zn - z_d );
            theta_int = theta_int + h * ssa( theta - theta_d );
            psi_int = psi_int + h * ssa( psi - psi_d );
        case {4}
            % Outer loop
            if mod(i,r_f) == 1
                z_ref2 = z_reference(i:r_f:i+T_fut*r_f);
                % predictive LP filtering of the depth command
                for j = 1:1:T_fut
                    z_ref = z_ref2(j);
                    [xf_z_d, z_d] = lowPassFilter(xf_z_d, z_ref, wn_d_z, h*r_f);
                    if j == 1
                        xf_z_d_next_it = xf_z_d;
                    end
                    y_reference(j) = z_d; 
                end
                z_d = y_reference(1); % for saving later
                xf_z_d = xf_z_d_next_it;
                % Outer DeePC control
                tic;
                [u_fut, y_fut, g_fut, sigma_fut, opt_error_flag] = control.step(u_past_sim, y_past_sim, y_reference', H_u, H_y); 
                elapsedTime = [elapsedTime, toc]; 
                if opt_error_flag == 1
                    disp(["Warning: Opt_error, Timestep: " + string(i)]);
                end
                % Extrapolating over the predicted value: 
                theta_d_vec = interp1([1 r_f+1], [theta_d; u_fut(1)], linspace(1,r_f+1+T_fut_pitch,r_f+1+T_fut_pitch), 'linear', 'extrap');
                theta_runner = 1;
            end
            theta_runner = theta_runner + 1;
            theta_d = theta_d_vec(theta_runner);
            y_reference_pitch = theta_d_vec(theta_runner:theta_runner+T_fut_pitch-1);

            % Inner DeePC control
            tic;
            [u_fut_pitch, y_fut_pitch, g_fut_pitch, sigma_fut_pitch, opt_error_flag] = control_pitch.step(u_past_sim_pitch, y_past_sim_pitch, y_reference_pitch', H_u_pitch, H_y_pitch); 
            elapsedTime_pitch = [elapsedTime_pitch, toc];
            delta_s = sat(u_fut_pitch(1), delta_max);
            if opt_error_flag == 1
                    disp(["Warning: Opt_error, Timestep: " + string(i)]);
            end

            % Third-order reference model for the heading angle
            for f=1:1:T_fut_heading
                [psi_d, r_d, a_d] = refModel(psi_d, r_d, a_d, psi_reference(i+f-1), r_max,...
                    zeta_d_psi, wn_d_psi, h, 1);
                y_reference_heading(f) = psi_d;
                r_reference_heading(f) = r_d;
                a_reference_heading(f) = a_d;
            end
            psi_d = y_reference_heading(1);
            r_d = r_reference_heading(1);
            a_d = a_reference_heading(1);
            tic;
            [u_fut_heading, y_fut_heading, g_fut_heading, sigma_fut_heading, opt_error_flag] = control_heading.step(u_past_sim_heading, y_past_sim_heading, y_reference_heading', H_u_heading, H_y_heading); 
            elapsedTime_heading = [elapsedTime_heading, toc];
            delta_r = sat(u_fut_heading(1), delta_max);
            if opt_error_flag == 1
                disp(["Warning: Opt_error, Timestep: " + string(i)]);
            end

            % Ocean current dynamics
            if DisturbanceFlag == 1
                if t(i) > 10
                    Vc = 0.5;
                    wc = 0.05;
                    betaVc = deg2rad(150);
                else
                    Vc = 0;
                    wc = 0;
                    betaVc = deg2rad(150);
                end
                betaVc = betaVc + randn / 1000;
                Vc = Vc + 0.002 * randn;
                wc = wc + 0.001 * randn;
            end
            
           if mod(i,r_f) == 0
               u_past_sim = [u_past_sim(2:end,:); theta];
           end
           if mod(i,r_f) == 0
               y_past_sim = [y_past_sim(2:end,:); zn];
           end
           u_past_sim_heading = [u_past_sim_heading(2:end,:); [delta_r]];
           y_past_sim_heading = [y_past_sim_heading(2:end,:); psi];
           u_past_sim_pitch = [u_past_sim_pitch(2:end,:); [delta_s]];
           y_past_sim_pitch = [y_past_sim_pitch(2:end,:); theta];
           
        case {5} % ALOS path-following with DeePC
            [psi_d_vec, theta_d_vec, y_e, z_e, alpha_c_hat, beta_c_hat, k1] = PALOS(xn, yn, zn, ...
            Delta_h, Delta_v, gamma_h, gamma_v, M_theta, h, R_switch, ...
            wpt, x, max(T_fut_heading, T_fut_pitch), K_f);

            y_reference_heading = psi_d_vec(1:1:T_fut_heading);

            tic;
            [u_fut_heading, y_fut_heading, g_fut_heading, sigma_fut_heading, opt_error_flag] = control_heading.step(u_past_sim_heading, y_past_sim_heading, y_reference_heading', H_u_heading, H_y_heading); 
            if opt_error_flag == 1
                disp(["Warning: Opt_error, Timestep: " + string(i)]);
            end
            elapsedTime_heading = [elapsedTime_heading, toc];
            delta_r = sat(u_fut_heading(1), delta_max);

            
            y_reference_pitch = theta_d_vec(1:1:T_fut_pitch);
            tic;
            [u_fut_pitch, y_fut_pitch, g_fut_pitch, sigma_fut_pitch, opt_error_flag] = control_pitch.step(u_past_sim_pitch, y_past_sim_pitch, y_reference_pitch', H_u_pitch, H_y_pitch); 
            if opt_error_flag == 1
                disp(["Warning: Opt_error, Timestep: " + string(i)]);
            end
            elapsedTime_pitch = [elapsedTime_pitch, toc];
            delta_s = sat(u_fut_pitch(1), delta_max);

            theta_d = theta_d_vec(1);
            psi_d = psi_d_vec(1);
            z_d = wpt.pos.z(min(k1+1, length(wpt.pos.z)));

            
            % Ocean current dynamics
            if DisturbanceFlag == 1
                if t(i) > 10
                    Vc = 0.5;
                    wc = 0.05;
                    betaVc = deg2rad(150);
                else
                    Vc = 0;
                    wc = 0;
                    betaVc = deg2rad(150);
                end
                betaVc = betaVc + randn / 1000;
                Vc = Vc + 0.002 * randn;
                wc = wc + 0.001 * randn;
            end

            % Store ALOS data in table
            alosData(i,:) = [y_e z_e alpha_c_hat beta_c_hat];
               if mod(i,r_f) == 0
                   u_past_sim = [u_past_sim(2:end,:); theta];
               end
               if mod(i,r_f) == 0
                   y_past_sim = [y_past_sim(2:end,:); zn];
               end
               u_past_sim_heading = [u_past_sim_heading(2:end,:); [delta_r]];
               y_past_sim_heading = [y_past_sim_heading(2:end,:); psi];
               u_past_sim_pitch = [u_past_sim_pitch(2:end,:); [delta_s]];
               y_past_sim_pitch = [y_past_sim_pitch(2:end,:); theta];
   

            otherwise % ALOS path-following
            % ALOS guidance law
            [psi_ref, theta_ref, y_e, z_e, alpha_c_hat, beta_c_hat, k1] = ...
                ALOS3D(xn, yn, zn, Delta_h, Delta_v, gamma_h, gamma_v,...
                M_theta, h, R_switch, wpt);
            
            z_d = wpt.pos.z(min(k1+1, length(wpt.pos.z)));
            % ALOS observer
            [theta_d, q_d] = LOSobserver(theta_d, q_d, theta_ref, h, K_f);
            [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, h, K_f);
            r_d = sat(r_d, r_max);

            delta_r = Kp_psi * ssa( psi - psi_d )...        % PID
                + Ki_psi * psi_int + Kd_psi * r;
            delta_r = sat(delta_r, delta_max);
            
            % Depth autopilot using the stern planes (PID)
            delta_s = Kp_theta * ssa( theta - theta_d )...
                + Kd_theta * q + Ki_theta * theta_int;
            delta_s = sat(delta_s, delta_max);

            % Ocean current dynamics
            if DisturbanceFlag == 1
                if t(i) > 10
                    Vc = 0.5;
                    wc = 0.05;
                    betaVc = deg2rad(150);
                else
                    Vc = 0;
                    wc = 0;
                    betaVc = deg2rad(150);
                end
                betaVc = betaVc + randn / 1000;
                Vc = Vc + 0.002 * randn;
                wc = wc + 0.001 * randn;
            end
            alosData(i,:) = [y_e z_e alpha_c_hat beta_c_hat];
            % Euler's integration method (k+1)
            z_int = z_int + h * ( zn - z_d );
            theta_int = theta_int + h * ssa( theta - theta_d );
            psi_int = psi_int + h * ssa( psi - psi_d );
   end
   n = 1000;
   % Control input vector
   ui = [delta_r delta_s n]';            

   % Store simulation data in a table
   simData(i,:) = [z_d theta_d psi_d r_d Vc betaVc wc ui' x'];
   x = rk4(@remus100_updated_OC, h, x, ui, Vc, betaVc, wc);  % RK4 method x(k+1)
end

close all;
disp("Average control times: ");
disp(mean(elapsedTime));
disp(mean(elapsedTime_heading));
disp(mean(elapsedTime_pitch));

% Data allocation
scrSz = get(0, 'ScreenSize'); % Returns [left bottom width height]
legendLocation = 'best'; legendSize = 12;
if isoctave; legendLocation = 'northeast'; end

% simData = [z_d theta_d psi_d r_d Vc betaVc wc ui' x']
z_d     = simData(:,1);
theta_d = simData(:,2);
psi_d   = simData(:,3);
r_d     = simData(:,4);
Vc      = simData(:,5);
betaVc  = simData(:,6);
wc      = simData(:,7);
ui       = simData(:,8:10);
nu      = simData(:,11:16);
u = nu(:,1); v = nu(:,2); w = nu(:,3);
p = nu(:,4); q = nu(:,5); r = nu(:,6);
eta = simData(:,17:22);

% alosData = [y_e z_e alpha_c_hat beta_c_hat]
y_e = alosData(:,1);
z_e = alosData(:,2);
alpha_c_hat = alosData(:,3);
beta_c_hat = alosData(:,4);

% Ocean current velocities 
uc = Vc .* cos(betaVc);
vc = Vc .* sin(betaVc);

% Crab angles, AOA and SSA
U = sqrt(u.^2+v.^2+w.^2); % Speed
gamma = asin( (u.*sin(theta)-(v.*sin(phi)+w.*cos(phi)).*cos(theta)) ./ U );
alpha_c = theta - gamma; % Horizontal crab angle
beta_c = atan2(v.*cos(phi)-w.*sin(phi), ...
    u.*cos(theta)+(v.*sin(phi)+w.*cos(phi)).*sin(theta)); % Vertical crab angle
alpha = asin( (w-wc) ./ (u-uc) ); % AOA
beta  = atan2( (v-vc), (u-uc) ); % SSA

% RMSEs
disp("Heading control RMSE:")
disp(rmse(rad2deg((eta(:,6))), rad2deg((psi_d))))
disp("Depth control RMSE - z:")
disp(rmse(((eta(:,3))), ((z_d))))
disp("Depth control RMSE - theta:")
disp(rmse(rad2deg((eta(:,5))), rad2deg((theta_d))))

%% PLOTS
% Generalized velocity
figure(1);
if ~isoctave; set(gcf,'Position',[1, 1, scrSz(3)/3, scrSz(4)]); end
subplot(611),plot(t,u)
xlabel('Time (s)'),title('Surge velocity (m/s)'),grid
subplot(612),plot(t,v)
xlabel('Time (s)'),title('Sway velocity (m/s)'),grid
subplot(613),plot(t,w)
xlabel('Time (s)'),title('Heave velocity (m/s)'),grid
subplot(614),plot(t,(180/pi)*p)
xlabel('Time (s)'),title('Roll rate (deg/s)'),grid
subplot(615),plot(t,(180/pi)*q)
xlabel('Time (s)'),title('Pitch rate (deg/s)'),grid
subplot(616),plot(t,(180/pi)*r)
xlabel('Time (s)'),title('Yaw rate (deg/s)'),grid
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(findall(gcf,'type','legend'),'FontSize',legendSize)

% Heave position and Euler angles
figure(2);
if ~isoctave; set(gcf,'Position',[scrSz(3)/3, 1, scrSz(3)/3, scrSz(4)]); end
subplot(411),plot(t,eta(:,3),t,z_d)
xlabel('Time (s)'),title('Heave position (m)'),grid
legend('True','Desired')
subplot(412),plot(t,rad2deg(eta(:,4)))
xlabel('Time (s)'),title('Roll angle (deg)'),grid
subplot(413),plot(t,rad2deg(ssa(eta(:,5))),t,rad2deg(ssa(theta_d)))
xlabel('Time (s)'),title('Pitch angle (deg)'),grid
legend('True','Desired')
subplot(414),plot(t,rad2deg(ssa(eta(:,6))),t,rad2deg(ssa(psi_d)))
xlabel('Time (s)'),title('Yaw angle (deg)'),grid
legend('True','Desired')
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(findall(gcf,'type','legend'),'FontSize',legendSize)


% Control signals
figure(3);
if ~isoctave; set(gcf,'Position',[2*scrSz(3)/3,scrSz(4)/2,scrSz(3)/3,scrSz(4)/2]);end
subplot(311),plot(t,rad2deg(ui(:,1)))
xlabel('Time (s)'),title('Rudder command \delta_r (deg)'),grid
subplot(312),plot(t,rad2deg(ui(:,2)))
xlabel('Time (s)'),title('Stern-plane command \delta_s (deg)'),grid
subplot(313),plot(t,ui(:,3))
xlabel('Time (s)'),title('Propeller speed command n (rpm)'),grid
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)

% Ocean currents and speed
figure(4);
if ~isoctave; set(gcf,'Position',[2*scrSz(3)/3,1,scrSz(3)/3,scrSz(4)/2]);end
subplot(311),plot(t,sqrt(nu(:,1).^2+nu(:,2).^2),t,Vc)
xlabel('Time (s)'),grid
legend('Vehicle horizontal speed (m/s)','Ocean current horizontal speed (m/s)',...
    'Location',legendLocation)
subplot(312),plot(t,nu(:,3),t,wc)
xlabel('Time (s)'),grid
legend('Vehicle heave velocity (m/s)','Ocean current heave velocity (m/s)',...
    'Location',legendLocation)
subplot(313),plot(t,rad2deg(betaVc),'r')
xlabel('Time (s)'),grid
legend('Ocean current direction (deg)','Location',legendLocation)
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(findall(gcf,'type','legend'),'FontSize',legendSize)


% Crab angles, SSA and AOA
if ControlFlag == 3 || ControlFlag == 5
    figure(5);
    if ~isoctave; set(gcf,'Position',[100,scrSz(4)/2,scrSz(3)/3,scrSz(4)]); end
    subplot(311)
    plot(t,rad2deg(alpha),'g',t,rad2deg(alpha_c),'b',...
        t,rad2deg(alpha_c_hat),'r')
    title('Vertical crab angle and AOA (deg)')
    xlabel('Time (s)')
    grid
    legend('\alpha Angle of attack (AOA)','\alpha_c Vertical crab angle','\alpha_c ALOS estimate','Location',legendLocation)
    subplot(312)
    plot(t,rad2deg(beta),'g',t,rad2deg(beta_c),'b',...
        t,rad2deg(beta_c_hat),'r')
    title('Horizontal crab angle and SSA (deg)')
    xlabel('Time (s)')
    grid
    legend('\beta Sideslip angle (SSA)','\beta_c Horizontal crab angle','\beta_c ALOS estimate','Location',legendLocation)
    subplot(313)
    plot(t,y_e,t,z_e)
    title('Tracking errors (m)'),grid
    xlabel('Time (s)')
    legend('Cross-track error y_e^p','Vertical-track error z_e^p')
    set(findall(gcf,'type','line'),'linewidth',2)
    set(findall(gcf,'type','text'),'FontSize',14)
    set(findall(gcf,'type','legend'),'FontSize',legendSize)
end

% 2-D position plots with waypoints
if ControlFlag == 3 || ControlFlag == 5
    figure(6);
    if ~isoctave;set(gcf,'Position',[300,200,scrSz(3)/3,scrSz(4)/2]);end
    subplot(211);
    plot(eta(:,2), eta(:,1));
    hold on;
    plot(wpt.pos.y, wpt.pos.x, 'rx', 'MarkerSize', 10);
    hold off;
    xlabel('East');
    ylabel('North');
    title('North-East plot (m)');
    xlim([0, 2500]);
    axis('equal');
    grid on;
    subplot(212);
    plot(eta(:,2), eta(:,3));
    hold on;
    plot(wpt.pos.y, wpt.pos.z, 'rx', 'MarkerSize', 10);
    hold off;
    xlim([0, 2500]);
    ylim([0, 150]);
    xlabel('East');
    ylabel('Down');
    title('Down-East plot (m)');
    grid on;
    legend('Actual path', 'Waypoints', 'Location', legendLocation);
    set(findall(gcf, 'type', 'line'), 'LineWidth', 2);
    set(findall(gcf, 'type', 'text'), 'FontSize', 14);
    set(findall(gcf, 'type', 'legend'), 'FontSize', legendSize);
end

% 3-D position plot with waypoints
if ControlFlag == 3 || ControlFlag == 5
    fig = figure(7); clf;
    set(fig, 'Color', 'w');    
    plot3(eta(:,2),eta(:,1),eta(:,3))
    hold on;
    plot3(wpt.pos.y, wpt.pos.x, wpt.pos.z, 'ro', 'MarkerSize', 15);
    hold off
    % title('North-East-Down plot (m)')
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Down (m)');
    legend('Actual path','Waypoints','Location',legendLocation),grid
    set(gca, 'ZDir', 'reverse');
    set(findall(gcf,'type','line'),'linewidth',2)
    set(findall(gcf,'type','text'),'FontSize',14)
    set(findall(gcf,'type','legend'),'FontSize',legendSize)
    view(-25, 30);  % view(AZ,EL)
end

% Display the vehicle data and an image of the vehicle
vehicleData = {...
    'Length', '1.6 m',...
    'Diameter', '19 cm',...
    'Mass', '31.9 kg',...
    'Max speed', '2.5 m/s',...
    'Max propeller speed', '1525 RPM'};
displayVehicleData('Remus100 AUV', vehicleData, 'remus100.jpg', 8);

