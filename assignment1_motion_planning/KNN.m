function [neighbors] = KNN(milestones, point, K)
% KNN - Implement K-Nearest-Neighbors alg. to find K neighbors of a point
% inputs: milestones (Nx2 array) - 2D vertices: [X1 Y1; X2 Y2; ... XN YN;]
%         point (2x1 array)      - current vertex Ex. [x, y]
%         K (scalar)             - number of neighbors to find
% output: neighbors (Kx2 array)  - K nearest neighbors sorted by distance
%                                  [XN1, YN1; XN2, YN2;... XNK, YNK;]

x = point(1); y = point(2);       % x, y coordinates of current node

% array based dist calculation rtning index of distances sorted in ascend. 
[~, idx_array] = sort(sqrt((milestones(:,1) - x).^2 + ...
                          (milestones(:, 2) - y).^2));
% re-sort milestones using sorted distance idx
sorted_points_array = milestones(idx_array, :);

% select only first K neighbors. Note, 1st neighbor is vertex itself
neighbors = sorted_points_array(2:K+1, :);
end
    
