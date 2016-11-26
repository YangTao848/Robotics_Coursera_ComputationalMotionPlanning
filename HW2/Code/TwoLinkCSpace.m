%
% TwoLinkRobotCSpace
%

%% Draw Robot and obstacles
clear all
close all
clc

figure(1);
subplot(1,2,1);

% This function sets up the two link robot based on the coordinates
% in configuration space [theta1, theta2]. You can change the configuration
% of the robot by changing the two numbers in the input array.
fv = TwoLinkRobot ([310 64]);

% 'Patch' generate the links for the robot on the plot .
patch (fv,'FaceColor','b','EdgeColor','none');


% These arrays define the vertices and faces of the obstacle as a patch
obstacle.vertices = [3 3; 3 4; 4 3; -6 6; -6 8; -7 6; -8 -6; 8 -6; -8 -10; 8 -10];
obstacle.faces = [1 2 3; 4 5 6; 7 8 9; 8 9 10];

% Generate obstacle on the plot
hold on;
patch(obstacle);
hold off;

axes_size = 12;
axis equal;
axis (axes_size*[-1 1 -1 1]);


%% Compute Configuration Space
theta1_range = 0:2:360;
theta2_range = 0:2:360;

nrows = length(theta2_range);
ncols = length(theta1_range);

cspace = true(nrows, ncols);

for i = 1:nrows
    for j = 1:ncols

        fv = TwoLinkRobot ([theta1_range(j) theta2_range(i)]);
        cspace (i,j) = CollisionCheck (fv, obstacle);
    end
    
    fprintf ('%d of %d\n', i, nrows);
end


%% Plot configuration space
subplot (1,2,2);

axis equal;
axis ([0 360 0 360]);

cmap = [1 1 1; 0 0 0];
colormap(cmap);

% Here we may flip the cspace image to match the axes
imagesc([0 360], [0 360], cspace);
axis xy;

xlabel ('theta1 in degrees');
ylabel ('theta2 in degrees');

title ('Configuration Space');

%% Plot a path through torus space

% New figure to visualize progress of planner
figure(2);

% You should experiment by changing these coordinates
start_x = 2;%310;
start_y = 2;%64;
end_x   = 320;%76;
end_y   = 10;%12;

% To make the process faster, we took 180 values of Thetas,
% ranging from 0 to 360 with a step of 2.
% N.B.: Check "Compute Configuration Space" section
% So divide the actual coordinates by 2
start_coords = [start_y/2, start_x/2];
end_coords = [end_y/2, end_x/2];

% Find a route between the start and end nodes
route = DijkstraTorus (cspace, start_coords, end_coords);

%% Animate the route
figure(1)
[i,j] = ind2sub (size(cspace), route);

y = theta2_range(i);
x = theta1_range(j);

% Plot point in configuration space
subplot(1,2,2);
hold on;
h = plot (x(1), y(1), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'red');

n = length(x);

for i = 1:n
    
    h.XData = x(i);
    h.YData = y(i);
    plot (x(i), y(i), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'red');
    drawnow;
end

hold off;

%% Animate the robot
figure(3)
for i = 1:n
    clf('reset')
    fv = TwoLinkRobot ([x(i) y(i)]);
    patch (fv,'FaceColor','b','EdgeColor','none');

    hold on;
    patch(obstacle);
    hold off;

    axis equal;
    axis (axes_size*[-1 1 -1 1]);
    pause(0.1)
end
