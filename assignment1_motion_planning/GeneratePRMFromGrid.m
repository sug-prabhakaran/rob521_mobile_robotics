function [edges] = GeneratePRMFromGrid(milestones, map, size)
% GENERATEPRMFROMGRID - Given a uniform-grid of milestones, build a graph
% inputs: milestones (Nx2 array) - each row is a point of form: [xi yi];
%                map (Mx4 array) - all edges of maze with end pts in a row
%                  size (string) - 'small' or large to compensate for dist
%                                  between the points
% outputs: edges (Mx4 array) - each 1x4 row is edge: [x1 y1 x2 y2];

% for small grid: 8 directions (neighbors) for vertex to traverse:
% up:          (x, y+0.25),         down:       (x, y-0.25) 
% right:       (x+0.25, y),         left:       (x-0.25, y)
% topright:    (x+0.25, y+0.25),    topleft:    (x-0.25, y+0.25)
% bottomright: (x+0.25, y-0.25),    bottomleft: (x-0.25, y-0.25)

% initialize parameters
edges = [0 0 0 0];          
N = length(milestones);

% iterate through all vertices
for i=1:N
    vertex = milestones(i,:);           % set vertex
    x = vertex(1); y = vertex(2);       % x and y coords for vertex
    % neighbors in clockwise order starting with directly above
    if (size == 'small')                % for graph <= 10x10
        neighbors = [x, y+0.25; x+0.25, y+0.25; x+0.25, y; ...
                     x+0.25, y-0.25; x, y-0.25; x-0.25, y-0.25; ...
                     x-0.25, y; x-0.25, y+0.25];
    else                                % if graph is larger than 10x10
        if ((i==1) || (i==N)) % start, finish pts are closer than 1 unit
            neighbors = KNN(milestones, vertex, 3);
        else                  % rest of the points are 1 unit away
            neighbors = [x, y+1; x+1, y+1; x+1, y; x+1, y-1; ...
             x, y-1; x-1, y-1; x-1, y; x-1, y+1];
        end
    end    
    % iterate through each of the neighbors for a vertex
    % if no collision, add it as an edge to build the graph
    for j=1:length(neighbors)
        if (ismember(neighbors(j,:),milestones,'rows'))
            [inCollision, ~] = CheckCollision(vertex, neighbors(j,:), map);
            if ((inCollision==0) && ...
                (~ismember([vertex, neighbors(j,:)],edges, 'rows')) && ...
                (~ismember([neighbors(j,:), vertex],edges, 'rows')))
                edges = [edges; vertex, neighbors(j,:)];
            end
        end
    end
end
end
