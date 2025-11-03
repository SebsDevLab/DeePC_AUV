%% File Name: DeePC.m
% Author: Sebastian Zieglmeier 
% Date last updated: 30.10.2025
% Description: Controller class for DeePC
% Sources: 
% [1] - Sebastian Zieglmeier, et.al., "Data-Enabled Predictive Control and 
%       Guidance for Autonomous Underwater Vehicles", 
%       https://doi.org/10.48550/arXiv.2510.25309
%
% Properties: 
% T_d: Number of data collection points
% T_ini: Number of initialization points
% T_fut: Prediction horizon
% R, Q: cost matrices for input and output
% lambda_ini, lambda_g: regularization hyperparameter
% H_u, H_y: Hankel/Page matrices for input and output data
% u_min, u_max: input constraints
% y_min, y_max: output constraints
% 
% Inputs Step function: 
% obj: Object
% x0: inital state for parametric model
% u_past, y_past: past T_ini data values for input and output
% y_ref: reference trajectory for prediction horizon
%
% Outputs Step function:
% u_fut: computed control input sequence
% y_value, g_value, sigma_value: Vectors for further investigation
% yalmip_error_flag: error flag for optimization errors
% 
%  
% Notes: 
% 


classdef DeePC < handle
    properties
        % General control variables:
        T_d
        T_ini
        T_fut
        T
        R
        Q
        lambda_ini
        lambda_g

        % Hankel matrices
        H_u
        H_y
        
        % System dimensions
        nu
        ny

        % System boundaries
        u_min
        u_max
        y_min
        y_max
    end
    
    methods
        function obj = DeePC(T_d, T_ini, T_fut, R, Q, lambda_ini, lambda_g, H_u, H_y, sys)
            % General control variables
            obj.T_d = T_d;
            obj.T_ini = T_ini;
            obj.T_fut = T_fut;
            obj.T = T_ini + T_fut;
            obj.R = R;
            obj.Q = Q;
            obj.lambda_ini = lambda_ini;
            obj.lambda_g = lambda_g;
            
            % Hankel matrices
            obj.H_u = H_u;
            obj.H_y = H_y;
            
            obj.nu = sys.nu; % Number of inputs
            obj.ny = sys.ny; % Number of outputs
            
            obj.u_min = sys.constraints.u_min * ones(T_fut, sys.nu);
            obj.u_max = sys.constraints.u_max * ones(T_fut, sys.nu);
            obj.y_min = sys.constraints.y_min * ones(T_fut, sys.ny);
            obj.y_max = sys.constraints.y_max * ones(T_fut, sys.ny);
            
        end
        


        function [u_fut, y_value, g_value, sigma_value, yalmip_error_flag] = step(obj, u_past, y_past, y_ref, H_u, H_y)
            %% Initialize Optimization Variables
            
            % Data driven component
            g = sdpvar(obj.T_d - obj.T + 1, 1);
            sigma = sdpvar(obj.T_ini, obj.ny);
            
            u = sdpvar(obj.T_fut, obj.nu);
            y = sdpvar(obj.T_fut, obj.ny);

            % Reshape Hankel matrizes H_u and H_y
            U_p = H_u(1:obj.nu * obj.T_ini, :);
            U_f = H_u(obj.nu * obj.T_ini + 1:end, :);
            Y_p = H_y(1:obj.ny * obj.T_ini, :);
            Y_f = H_y(obj.ny * obj.T_ini + 1:end, :);

            %% Cost function
            cost = reshape(sigma, [], 1)'*obj.lambda_ini*reshape(sigma, [], 1) + obj.lambda_g * norm(g, 2);
            for i = 1:1:obj.T_fut
                cost = cost + (y(i,:)-y_ref(i,:))*obj.Q*(y(i,:)-y_ref(i,:))' + u(i,:)*obj.R*u(i,:)';
            end
            %% Constraints
            constraints = [
                U_p * g == reshape(u_past', [], 1);
                U_f * g == reshape(u', [], 1);
                Y_p * g == reshape((y_past + sigma)', [], 1) ;
                Y_f * g == reshape((y)', [], 1) 
            ];

            % General control and constraints:
            constraints = [constraints;
                y(end,:) == y_ref(end,:); % Terminal constraint
                u >= obj.u_min;
                u <= obj.u_max;
                y >= obj.y_min;
                y <= obj.y_max;
            ];


            %% Solve the problem
            options = sdpsettings('verbose', 0, 'solver', 'mosek', 'debug',1);
            diagnostics = optimize(constraints, cost, options);
            yalmip_error_flag = 0;
            if diagnostics.problem ~= 0
                % warning('YALMIP had an issue with the optimization. Problem Code: %d', diagnostics.problem);
                yalmip_error_flag = 1;
            end
            u_fut = value(u);

            % Dummy vectors for debugging and further investigation outside the controller:
            y_value = value(y);
            g_value = value(g);
            sigma_value = value(sigma);
        end
    end
end
