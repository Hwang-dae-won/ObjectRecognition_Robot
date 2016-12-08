clear; close all; clc;

%% Gazebo Launch
% type "roslaunch gazebo_ros empty_world.launch" on the ubuntu terminal

%% ROS Init
ipaddress = '0.0.0.0';
rosinit(ipaddress)

% rostopic list
% rosservice list
% Set up a connection to the Gazebo world
C = ExampleHelperGazeboCommunicator();

%% Spawn My home and My households

