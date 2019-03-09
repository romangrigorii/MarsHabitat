%% cylWallTask.m
%
% Description:
%   Simulate the kinematics (forward, inverse) required to print a
%   cylindrical wall in the workspace in Tech AG53 with the ABB IRB 6700
%   arm.
%
% Notes:
%   1 - all linear dimensions are in meters except where otherwise noted.
%
% Author:
%   Dan Lynch
%
% Date:
%   March 9, 2019

%% main function
function cylWallTask
    clear;
    close all;
    clc;
    
    [perim,slab,robot,workpiece] = init_workspace();
    visualize_workspace(perim,slab,robot,workpiece);
    
end

%% Workspace description in task-space frame
function [perim,slab,robot,workpiece] = init_workspace()
% The workspace contains 4 things (represented as structs):
%   1 - workspace "perimeter"
%   2 - concrete "slab" (underneath the robot base)
%   3 - "robot" (whose base is coincident with the center of the concrete
%   slab)
%   4 - "workpiece" (the thing the robot will 3D print)

% perimeter dimensions
perim.frame.x = 0; % x-coordinate of the perimeter frame in the task-space frame
perim.frame.y = 0; % y-coordinate of the perimeter frame in the task-space frame
perim.width_dim = 4.7244;   % extends in the task-frame x-direction
perim.length_dim = 4.4196;  % extends in the task-frame y-direction
perim.height_dim = 2;       % extends in the task-frame z-direction (this vlaue is a guess)

% perimeter corners (A-D, clockwise, A is coincident with task-space frame)
perim.Ax = perim.frame.x + 0;
perim.Ay = perim.frame.y + 0;

perim.Bx = perim.frame.x + 0;
perim.By = perim.frame.y + perim.length_dim;

perim.Cx = perim.frame.x + perim.width_dim;
perim.Cy = perim.frame.y + perim.length_dim;

perim.Dx = perim.frame.x + perim.width_dim;
perim.Dy = perim.frame.y + 0;

perim.x_coords = [perim.Ax, perim.Bx, perim.Cx, perim.Dx];
perim.y_coords = [perim.Ay, perim.By, perim.Cy, perim.Dy];

% slab dimensions:
slab.offset.x = 2.3622;
slab.offset.y = 1.016;
slab.width_dim = 1.1176;    % extends in task-frame x-direction
slab.length_dim = 1.1176;   % extends in task-frame y-direction
slab.frame.x = slab.offset.x - 0.5*slab.width_dim; % x-coordinate of the slab frame in the task-space frame
slab.frame.y = slab.offset.y - 0.5*slab.length_dim; % y-coordinate of the slab frame in the task-space frame

% slab corners (A-D, clockwise, A is coincident with task-space frame)
slab.Ax = slab.frame.x + 0;
slab.Ay = slab.frame.y + 0;

slab.Bx = slab.frame.x + 0;
slab.By = slab.frame.y + slab.length_dim;

slab.Cx = slab.frame.x + slab.width_dim;
slab.Cy = slab.frame.y + slab.length_dim;

slab.Dx = slab.frame.x + slab.width_dim;
slab.Dy = slab.frame.y + 0;

slab.x_coords = [slab.Ax, slab.Bx, slab.Cx, slab.Dx];
slab.y_coords = [slab.Ay, slab.By, slab.Cy, slab.Dy];

% robot workspace dimensions (subsequent code will generate the actual
% robot frames and links)
robot.frames.base.x = slab.frame.x + 0.5*slab.width_dim;
robot.frames.base.y = slab.frame.y + 0.5*slab.length_dim;

robot.reach.min_reach = 0.994; % minimum reachable distance from robot base frame
robot.reach.max_reach = 2.848; % maximum reachable distance from robot base frame
robot.reach.min_reach_circle.x = robot.frames.base.x + ...
    robot.reach.min_reach*cos(linspace(0,2*pi,101));
robot.reach.min_reach_circle.y = robot.frames.base.y + ...
    robot.reach.min_reach*sin(linspace(0,2*pi,101));
robot.reach.max_reach_circle.x = robot.frames.base.x + ...
    robot.reach.max_reach*cos(linspace(0,2*pi,101));
robot.reach.max_reach_circle.y = robot.frames.base.y + ...
    robot.reach.max_reach*sin(linspace(0,2*pi,101));

% workpiece (3D printed cylindrical wall) dimensions
workpiece.offset.x = 0;
workpiece.offset.y = 1;
workpiece.radii.outer_radius = 1.1;
workpiece.frame.x = robot.frames.base.x + workpiece.radii.outer_radius + workpiece.offset.x;
workpiece.frame.y = robot.frames.base.y + workpiece.radii.outer_radius + workpiece.offset.y;
workpiece.points.x = workpiece.frame.x + workpiece.radii.outer_radius*sin(linspace(0,2*pi,101));
workpiece.points.y = workpiece.frame.y + workpiece.radii.outer_radius*cos(linspace(0,2*pi,101));

end

%% Visualize workspace
function visualize_workspace(perim,slab,robot,workpiece)
figure;
% plot workspace perimeter:
plot3(horzcat(perim.x_coords,perim.Ax),horzcat(perim.y_coords,perim.Ay),...
    zeros(1,length(perim.x_coords)+1),'k-');
hold on;
% plot slab:
plot3(horzcat(slab.x_coords,slab.Ax),horzcat(slab.y_coords,slab.Ay),...
    zeros(1,length(slab.x_coords)+1),'k-.');
% plot robot base, max & min reach:
plot3(robot.frames.base.x,robot.frames.base.y,0,'k.','MarkerSize',20);
plot3(robot.reach.min_reach_circle.x,robot.reach.min_reach_circle.y,...
    zeros(1,length(robot.reach.min_reach_circle.x)),'k:')
plot3(robot.reach.max_reach_circle.x,robot.reach.max_reach_circle.y,...
    zeros(1,length(robot.reach.max_reach_circle.x)),'k--')
% plot workpiece:
plot3(workpiece.frame.x,workpiece.frame.y,0,'b.','MarkerSize',20)
plot3(workpiece.points.x,workpiece.points.y,...
    zeros(1,length(workpiece.points.x)),'b-')
axis([-1, perim.width_dim + 1, -1, perim.length_dim + 1, 0, perim.height_dim]);
pbaspect([perim.width_dim, perim.width_dim, perim.height_dim]/perim.width_dim)

legend('workspace boundary','slab','robot: base','robot: min reach',...
    'robot: max reach','workpiece: center','workpiece: outer wall','Location','Best');
view(0,90); % initialize to overhead view
end