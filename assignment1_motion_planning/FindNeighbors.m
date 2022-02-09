function [neighbors] = FindNeighbors(edges, point)
% FINDEDGES - Given a point, find all edges connected to that point
% inputs: edges (Nx4 array) - each 1x4 row is edge: [x1 y1 x2 y2];
%         point (2x1 array) - milestone point of form: [P_x, P_y]
% output: neighbors (Kx2 array) - K rows of neighbors: [xj yj;] to point P

neighbors = [];                     % initialize array to store neighbors
[N,~] = size(edges);                % N edges

for i=1:N                           % iterate through all N edges in graph
    
    % point could match start vertex of edge, end vertex of edge or neither
    if point == edges(i,1:2)        % point matches start vertex 
        neighbor = edges(i,3:4);    % neighbor is opposite vertex
    elseif point == edges(i,3:4)    % point matches end vertex     
        neighbor = edges(i,1:2);    % neighbor is opposite vertex
    else
        continue                    % no matches, skip to next edge
    end
    
    % if neighbor is not in neighbor list, add it to the list
    if isempty(neighbors)
        neighbors = [neighbor];
    elseif ~(ismember(neighbor,neighbors, 'rows'))
        neighbors = [neighbors; neighbor];
    end
end
end