function [route,numExpanded] = DijkstraGrid (input_map, start_coords, dest_coords)
% Run Dijkstra's algorithm on a grid.
% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%   the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%   respectively, the first entry is the row and the second the column.
% Output :
%    route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route. This is a single dimensional vector
%    numExpanded: Remember to also return the total number of nodes
%    expanded during your search. Do not count the goal node as an expanded node.


% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination

%%

cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0; ...
	0.5 0.5 0.5];

colormap(cmap);

% Color variables for the map
FREE_CELL       = 1; % White
OBSTACLE_CELL   = 2; % Black
VISITED_CELL    = 3; % Red
BLUE_CELL       = 4; % Blue
START_CELL      = 5; % Green
DEST_CELL       = 6; % Yellow


% variable to control if the map is being visualized on every
% iteration
drawMapEveryTime = true;

[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = FREE_CELL ;   % Mark free cells
map(input_map)  = OBSTACLE_CELL;   % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = START_CELL;
map(dest_node)  = DEST_CELL;

% Initialize distance array
distanceFromStart = Inf(nrows,ncols);

% For each grid cell this array holds the index of its parent
parent = zeros(nrows,ncols);

distanceFromStart(start_node) = 0;

% keep track of number of nodes expanded 
numExpanded = 0;
% Main Loop
while true
    
    % Draw current map
    map(start_node) = START_CELL;
    map(dest_node)  = DEST_CELL;
    
    % make drawMapEveryTime = true if you want to see how the 
    % nodes are expanded on the grid. 
    if (drawMapEveryTime)
        image(1.5, 1.5, map);
        grid on;
        axis image;
        drawnow;
        pause(0.1);
    end
    
    % Find the node with the minimum distance
    [min_dist, current] = min(distanceFromStart(:));
    
    if ((current == dest_node) || isinf(min_dist))
        break;
    end;
    
    % Update map
    map(current) = 3;         % mark current node as visited
    distanceFromStart(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(distanceFromStart), current);
    
   % ********************************************************************* 
    % YOUR CODE BETWEEN THESE LINES OF STARS
    
    % Left Neighbour
    if (j>1 && j<=ncols)
        colNeighbour = j-1;
        rowNeighbour = i;
        if (map(rowNeighbour,colNeighbour)~=OBSTACLE_CELL && map(rowNeighbour,colNeighbour)~=VISITED_CELL && map(rowNeighbour,colNeighbour)~=START_CELL)
            map(rowNeighbour,colNeighbour) = BLUE_CELL;
            parent(rowNeighbour,colNeighbour) = current;
            distanceFromStart(rowNeighbour,colNeighbour) = min_dist+1;
        end
    end
    
    % Right Neighbour
    if (j>=1 && j<ncols)
        colNeighbour =j+1;
        rowNeighbour = i;
        if (map(rowNeighbour,colNeighbour)~=OBSTACLE_CELL && map(rowNeighbour,colNeighbour)~=VISITED_CELL && map(rowNeighbour,colNeighbour)~=START_CELL)
            map(rowNeighbour,colNeighbour) = BLUE_CELL;
            parent(rowNeighbour,colNeighbour) = current;
            distanceFromStart(rowNeighbour,colNeighbour) = min_dist+1;
        end
    end
    
    % Up Neighbour
    if (i>1 && i<=nrows)
        rowNeighbour = i-1;
        colNeighbour = j;
        if (map(rowNeighbour,colNeighbour)~=OBSTACLE_CELL && map(rowNeighbour,colNeighbour)~=VISITED_CELL && map(rowNeighbour,colNeighbour)~=START_CELL)
            map(rowNeighbour,colNeighbour) = BLUE_CELL;
            parent(rowNeighbour,colNeighbour) = current;
            distanceFromStart(rowNeighbour,colNeighbour) = min_dist+1;
        end
    end
    
    % Down Neighbour
    if (i>=1 && i<nrows)
        rowNeighbour = i+1;
        colNeighbour = j;
        if (map(rowNeighbour,colNeighbour)~=OBSTACLE_CELL && map(rowNeighbour,colNeighbour)~=VISITED_CELL && map(rowNeighbour,colNeighbour)~=START_CELL)
            map(rowNeighbour,colNeighbour) = BLUE_CELL;
            parent(rowNeighbour,colNeighbour) = current;
            distanceFromStart(rowNeighbour,colNeighbour) = min_dist+1;
        end
    end
    
    numExpanded = numExpanded + 1;
    
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.
    %*********************************************************************

end

%% Construct route from start to dest by following the parent links
if (isinf(distanceFromStart(dest_node)))
    route = [];
else
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end
    
        % Snippet of code used to visualize the map and the path
    for k = 2:length(route) - 1        
        map(route(k)) = 7;
        pause(0.1);
        image(1.5, 1.5, map);
        grid on;
        axis image;
    end
end
parent
end
