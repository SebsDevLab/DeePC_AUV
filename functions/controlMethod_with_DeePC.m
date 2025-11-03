%% File Name: controlMethod_with_DeePC.m
% Author: Sebastian Zieglmeier
% Date last updated: 30.10.2025
%
% Description:
%   Interactive GUI for selecting the control configuration of the 
%   REMUS 100 Autonomous Underwater Vehicle (AUV) simulation for the
%   publication:
%   S. Zieglmeier, “Data-Enabled Predictive Control and Adaptive Line-of-Sight 
%   Guidance for Autonomous Underwater Vehicles,” arXiv preprint, 2025. 
%   (Submitted for publication)
%
% Usage:
%   [ControlFlag, DisturbanceFlag] = controlMethod_with_DeePC()
%
% Control Modes:
%   ControlFlag = 1 : PID control for heading and depth (successive-loop closure)
%   ControlFlag = 3 : ALOS guidance law for 3-D path following (PID-based)
%   ControlFlag = 4 : Data-driven predictive control (DeePC)
%   ControlFlag = 5 : Hierarchical DeePC with PALOS predictive ALOS guidance
%
% Disturbance Option:
%   DisturbanceFlag = 0 : No ocean current disturbance
%   DisturbanceFlag = 1 : Include ocean current disturbance
%
% Data Handling Option:
%   DataFlag = 0 : Generate new data sequences
%   DataFlag = 1 : Load stored data sequences from MAT file
%
% Outputs:
%   ControlFlag     : Integer flag identifying the selected control configuration
%   DisturbanceFlag : Binary flag for ocean current inclusion (0 = off, 1 = on)

function [ControlFlag, DisturbanceFlag, DataFlag] = controlMethod_with_DeePC()
%% CONTROL METHOD SELECTION GUI
f = figure('Position', [400, 100, 400, 620], ...
    'Name', 'Select Control Method', ...
    'MenuBar', 'none', ...
    'NumberTitle', 'off', ...
    'WindowStyle', 'modal');

% -------------------------------------------------------------------------
% Control method selection
% -------------------------------------------------------------------------
bg1 = uibuttongroup('Parent', f, ...
    'Position', [0.05 0.62 0.9 0.28], ...
    'Title', 'Control Methods', ...
    'FontSize', 14, ...
    'FontWeight', 'bold');

uicontrol(bg1, 'Style', 'radiobutton', 'FontSize', 13, ...
    'String', 'PID control', ...
    'Position', [10 110 500 25], 'Tag', '1');

uicontrol(bg1, 'Style', 'radiobutton', 'FontSize', 13, ...
    'String', 'ALOS guidance law for 3-D path following', ...
    'Position', [10 80 500 25], 'Tag', '3');

uicontrol(bg1, 'Style', 'radiobutton', 'FontSize', 13, ...
    'String', 'Data-driven predictive control (DeePC)', ...
    'Position', [10 50 500 25], 'Tag', '4'); % Default

uicontrol(bg1, 'Style', 'radiobutton', 'FontSize', 13, ...
    'String', 'DeePC + PALOS', ...
    'Position', [10 20 500 25], 'Tag', '5', 'Value', 1);

% -------------------------------------------------------------------------
% Disturbance selection
% -------------------------------------------------------------------------
bg2 = uibuttongroup('Parent', f, ...
    'Position', [0.05 0.38 0.9 0.17], ...
    'Title', 'External Disturbances', ...
    'FontSize', 14, ...
    'FontWeight', 'bold');

uicontrol(bg2, 'Style', 'radiobutton', 'FontSize', 13, ...
    'String', 'No ocean current disturbance', ...
    'Position', [10 50 350 25], 'Tag', '0', 'Value', 1);

uicontrol(bg2, 'Style', 'radiobutton', 'FontSize', 13, ...
    'String', 'Include ocean current disturbance', ...
    'Position', [10 20 350 25], 'Tag', '1');

% -------------------------------------------------------------------------
% Data sequence handling
% -------------------------------------------------------------------------
bg3 = uibuttongroup('Parent', f, ...
    'Position', [0.05 0.20 0.9 0.17], ...
    'Title', 'Data Sequence Handling', ...
    'FontSize', 14, ...
    'FontWeight', 'bold');

uicontrol(bg3, 'Style', 'radiobutton', 'FontSize', 13, ...
    'String', 'Generate new data sequences', ...
    'Position', [10 50 350 25], 'Tag', '0');

uicontrol(bg3, 'Style', 'radiobutton', 'FontSize', 13, ...
    'String', 'Load stored data sequences (MAT file)', ...
    'Position', [10 20 350 25], 'Tag', '1','Value', 1);

% -------------------------------------------------------------------------
% OK button
% -------------------------------------------------------------------------
uicontrol('Style', 'pushbutton', ...
    'String', 'OK', ...
    'FontSize', 13, ...
    'Position', [150 20 100 40], ...
    'Callback', @(src, evt) uiresume(f));

uiwait(f);

% -------------------------------------------------------------------------
% Determine selections
% -------------------------------------------------------------------------
radios = findall(bg1, 'Style', 'radiobutton');
val = find([radios.Value]);
ControlFlag = str2double(radios(val).Tag);

radios2 = findall(bg2, 'Style', 'radiobutton');
val2 = find([radios2.Value]);
DisturbanceFlag = str2double(radios2(val2).Tag);

radios3 = findall(bg3, 'Style', 'radiobutton');
val3 = find([radios3.Value]);
DataFlag = str2double(radios3(val3).Tag);

close(f);

% -------------------------------------------------------------------------
% Display setup summary
% -------------------------------------------------------------------------
disp('-------------------------------------------------------------');
disp('MSS toolbox: Remus 100 AUV');
disp('Kinematic representation: Euler angles (12 states)');
switch ControlFlag
    case 1
        disp('Heading autopilot: PID control');
        disp('Depth autopilot:   Successive-loop closure');
    case 3
        disp('Heading autopilot: PID control');
        disp('Depth autopilot:   Successive-loop closure');
        disp('Path-following:    ALOS guidance law for 3-D path following');
    case 5
        disp('Heading autopilot: DeePC');
        disp('Depth autopilot:   Cascaded DeePC');
        disp('Path-following:    PALOS predictive ALOS guidance (3-D)');
    otherwise
        disp('Heading autopilot: DeePC');
        disp('Depth autopilot:   Cascaded DeePC');
end

if DisturbanceFlag
    disp('External disturbance: Ocean current ENABLED');
else
    disp('External disturbance: None');
end

if DataFlag
    disp('Data handling: Load stored sequences from MAT file');
else
    disp('Data handling: Generate new data sequences');
end

disp('-------------------------------------------------------------');
disp('Simulating...');
end
