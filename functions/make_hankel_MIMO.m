%% File Name: make_hankel_MIMO.m
% Author: Sebastian Zieglmeier
% Date last updated: 30.10.2025
% Description:
% Constructs a block Hankel matrix for multi-input multi-output (MIMO) data
% sequences. This function reshapes and stacks the input data into a Hankel
% structure suitable for use in Data-Enabled Predictive Control (DeePC) and
% other data-driven control formulations.
%
% Usage:
%   H = make_hankel_MIMO(x, num_col, L)
%
% Inputs:
%   x        : Measured input or output data sequence 
%   num_col  : Number of columns in the resulting Hankel matrix.
%   L        : Number of block rows (corresponding to the window length).
%
% Outputs:
%   H        : Constructed Hankel matrix of size (m*L x num_col).
%
% Notes:
% 

function H = make_hankel_MIMO(x, L)
    [m, K] = size(x);
    H = zeros(m*L, K-L+1);

    for i = 1:K-L+1
        H(:, i) = reshape(x(:, i:i+L-1), [], 1);
    end
end