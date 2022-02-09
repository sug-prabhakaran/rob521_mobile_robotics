function [edges] = GeneratePRM(milestones, K, map)
% GENERATEPRM - Iterates through K neighbors for each vertex, checks for
%               collision with an edge between them, connects them in graph
% inputs: milestones (Nx2 array) - all possible vertexes 
%                     K (scalar) - number of neighbors to uses
%                map (Mx4 array) - all edges of maze with end pts in a row
% outputs:     edges (Rx4 array) - all valid edges connecting vertices

% 1. set parameters
edges = [];
N = length(milestones);     % # of milestones

% 2. implement KNN Algorithm and collision checking
for i=1:N                   % iterate through 1:N milestones
    % find K neighbors for milestone i
    neighbors = KNN(milestones, milestones(i,:), K);

    for j=1:K               % iterate through 1:K neighbors
        % check if edge between milestone i and neighbor j collide
        [Collision, ~] = CheckCollision(milestones(i,:), neighbors(j,:), map); 

        if (Collision == 0) % if no collision
            % edge can be written in 2 ways (mile.,nbor.) & (nbor., mile.)
            edge = [milestones(i,1),milestones(i,2), ...
                    neighbors(j,1), neighbors(j,2)];
            edge2 =[neighbors(j,1), neighbors(j,2), ...
                    milestones(i,1),milestones(i,2)];
            if (isempty(edges))
                edges = [edge];
            % check if either form of edge is already in 'edges' array
            elseif ((~ismember(edge, edges, 'rows')) && ...
                    (~ismember(edge2, edges, 'rows')))
                % if edge is not in 'edges' add it to the array
                edges = [edges; edge];
            end
        end
    end
end