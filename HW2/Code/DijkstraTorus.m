function route = DijkstraTorus (input_map, start_coords, dest_coords)
% Run Dijkstra's algorithm on a grid.
% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%      the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%       respectively, the first entry is the row and the second the column.
% Output :
%   route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route.

% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination
% *************************************************************************

DEBUG = true;

cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0];

colormap(cmap);

% Color variables for the map
FREE_CELL       = 1; % White
OBSTACLE_CELL   = 2; % Black
VISITED_CELL    = 3; % Red
BLUE_CELL       = 4; % Blue
START_CELL      = 5; % Green
DEST_CELL       = 6; % Yellow

[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = FREE_CELL;  % Mark free cells
map(input_map)  = OBSTACLE_CELL;  % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = START_CELL;
map(dest_node)  = DEST_CELL;

% Initialize distance array
distances = Inf(nrows,ncols);

% For each grid cell this array holds the index of its parent
parent = zeros(nrows,ncols);

distances(start_node) = 0;

% Main Loop
while true
    
    % Draw current map
    map(start_node) = START_CELL;
    map(dest_node)  = DEST_CELL;
    
    image(1.5, 1.5, map);
    grid on;
    axis image;
    drawnow;
    
    % Find the node with the minimum distance
    [min_dist, current] = min(distances(:));
    
    if ((current == dest_node) || isinf(min_dist))
        break;
    end;
    
    % Update map
    map(current) = 3;         % mark current node as visited
    distances(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(distances), current);
    
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.
   
    %%% All of your code should be between the two lines of stars. 
    % *******************************************************************
    
    % Left Neighbour
    if (j == 1)
        colNeighbour = ncols;
    else
        colNeighbour = j-1;
    end
    rowNeighbour = i;
    update(rowNeighbour,colNeighbour,min_dist+1,current);

    

    % Right Neighbour
    if (j == ncols)
        colNeighbour = 1;
    else
        colNeighbour = j+1;
    end
    rowNeighbour = i;
    update(rowNeighbour,colNeighbour,min_dist+1,current);


    
    % Up Neighbour
    if (i == 1)
        rowNeighbour = nrows;
    else
        rowNeighbour = i-1;
    end
    colNeighbour = j;
    update(rowNeighbour,colNeighbour,min_dist+1,current);


    
    % Down Neighbour
    if (i == nrows)
        rowNeighbour = 1;
    else
        rowNeighbour = i+1;
    end
    colNeighbour = j;
    update(rowNeighbour,colNeighbour,min_dist+1,current);


    
    % *******************************************************************
end

if (isinf(distances(dest_node)))
    route = [];
else
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end
end

    function update (i,j,d,p)
        if ( (map(i,j) ~= OBSTACLE_CELL) && (map(i,j) ~= VISITED_CELL) && (map(i,j) ~= START_CELL) && (distances(i,j) > d) )
            distances(i,j) = d;
            map(i,j) = BLUE_CELL;
            parent(i,j) = p;
        end
    end

if (DEBUG)
    for k = 2:length(route) - 1        
        map(route(k)) = 7;
        pause(0.1);
        image(1.5, 1.5, map);
        grid on;
        axis image;
    end
end

end
