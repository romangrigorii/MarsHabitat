%% FK.m
% compute the foward kinematics for space-frame FK with ABB - IRB 6700 (155kg,
% 2.85m) 6-DOF arm

clear;
close all;
clc;

%% Setup environment
% add Modern Robotics library to path:
addpath('/home/daniel/ModernRobotics/packages/MATLAB/mr');

% use LaTeX interpreter:
set(groot,'defaultLegendInterpreter','latex');
set(groot,'defaultTextInterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');

% make text nice and big:
set(groot,'DefaultAxesFontSize',18)

% dimensions (meters)
A = 0.2;
B = 0.532;
C = 0.633;
D = 2.276;
E = 1.125;
F = 1.873;
G = 1.3925;

base_x = 0.377;
base_y = 0;
base_z = 0.780;

elbow_offset = 0.2;

omega = [0,0,1;
         0,1,0;
         0,1,0;
         1,0,0;
         0,1,0;
         1,0,0];
q = [0,0,0;
     base_x,base_y,base_z;
     base_x,base_y,base_z + E;
     base_x,base_y,base_z + E + elbow_offset;
     base_x + G,base_y,base_z + E + elbow_offset;
     base_x,base_y,base_z + E + elbow_offset];
 
for i = 1:6
    v(i,:) = cross(-omega(i,:),q(i,:));
end

Slist = [omega, v]; % screw axes in space frame

M{1} = [1,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1];
M{2} = [0,1,0,base_x; 0,0,1,base_y; 1,0,0,base_z; 0,0,0,1];
M{3} = [0,1,0,base_x; 0,0,1,base_y; 1,0,0,base_z + E; 0,0,0,1];
M{4} = [0,0,1,base_x; 0,1,0,base_y; -1,0,0,base_z + E + elbow_offset; ...
    0,0,0,1];
M{5} = [1,0,0,base_x + G; 0,0,1,base_y; 0,-1,0,base_z + E + elbow_offset; 0,0,0,1];
M{6} = [0, 0, 1, base_x + G + A;
      0, 1, 0, base_y;
     -1,0, 0, base_z + E + elbow_offset;
      0, 0, 0, 1];

joints_home = [0;0;0;0;0;0];
joints_max = deg2rad([170;85;70;300;130;360]);
joints_min = deg2rad([-170;-65;-180;-300;-130;-360]);
thetalist = deg2rad([0;60;-30;0;0;0]);

for i = 1:6
   T{i} = FKinSpace(M{i}, Slist(1:i,:)', thetalist(1:i));
end

cc = lines(6);
joint_marker_size = 50;
link_line_width = 10;
plot3(T{1}(1,4),T{1}(2,4),T{1}(3,4),'.','Color',cc(1,:),...
       'MarkerSize',joint_marker_size);
hold on;
for i = 2:6
   plot3(T{i}(1,4),T{i}(2,4),T{i}(3,4),'.','Color',cc(i,:),...
       'MarkerSize',joint_marker_size);
   line([T{i-1}(1,4),T{i}(1,4)],...
       [T{i-1}(2,4),T{i}(2,4)],...
       [T{i-1}(3,4),T{i}(3,4)],'Color',cc(i,:),'LineWidth',link_line_width);
end
axis([-3.0,3.0, -3.0,3.0, -0.5,5.5]);
hold off;
xlabel('$x$');ylabel('$y$');zlabel('$z$');