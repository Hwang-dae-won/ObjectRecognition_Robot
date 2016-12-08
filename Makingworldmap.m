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

%Home
home           = ExampleHelperGazeboModel('My_Room1_h','gazeboDB');

%Living room
Tv_table_h           = ExampleHelperGazeboModel('My_Tv_table_h','gazeboDB');
Sofa1_h           = ExampleHelperGazeboModel('My_Sofa1_h','gazeboDB');
Coffee_table1_h          = ExampleHelperGazeboModel('My_Coffee table1_h','gazeboDB');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Home
spawnModel(C,home,                 [0,             0,          0]);

%Living room
spawnModel(C,Tv_table_h,          [6560,             1400,          0]/1000);
spawnModel(C,Sofa1_h,          [10060,             1400,          0]/1000);
spawnModel(C,Coffee_table1_h,          [8660,             1400,          0]/1000);



%% Disconnect from the ROS

