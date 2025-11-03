%% File Name: make_page_MIMO.m
% Author: Sebastian Zieglmeier
% Date last updated: 30.10.2025
% Description:
% Constructs a block Page matrix for multi-input multi-output (MIMO) data
% sequences. The Page matrix is an alternative to the Hankel matrix often
% used in Data-Enabled Predictive Control (DeePC) to improve numerical
% conditioning and reduce redundancy in overlapping data segments.
%
% Usage:
%   H = make_page_MIMO(x, num_col, L)
%
% Inputs:
%   x        : Measured input or output data sequence (minimum data-points: num_col*L),
%   num_col  : Number of columns (segments) in the resulting Page matrix.
%   L        : Number of block rows (segment length).
%
% Outputs:
%   H        : Constructed Page matrix of size (m*L x num_col).
%
% Notes:

function H = make_page_MIMO(x, num_col, L)
    [m, K] = size(x);
    H = zeros(m*L, num_col);
    x_new = reshape(x, [], 1);
    for i = 1:num_col
        H(:, i) = x_new((i-1)*m*L+1:i*m*L);
    end
end