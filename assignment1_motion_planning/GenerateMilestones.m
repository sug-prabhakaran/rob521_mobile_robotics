function [milestones] = GenerateMilestones(nS, row, col, start, finish, map)
% GENERATEMILESTONES - Generate random samples and discard points that are
%                      too close to edges.
% inputs: nS (scalar)    - Number of samples to try. (Ex. 500 points)
%        row (scalar)    - Number of rows for the maze
%        col (scalar)    - Number of columns for the maze
%      start (2x1 array) - start point in form: [xs ys];
%     finish (2x1 array) - end point of maze in form: [xf yf];
%        map (Nx4 array) - array of all edges, each row form: [x1 y1 x2 y2]
% output: milestones (Mx2 array) - array of all points: [x1 y1; x2 y2; ...]

% generate milestones within bound:  X: (0.5, col+0.5), Y: (0.5, row+0.5)
points = [rand(nS,1)*col + 0.5, rand(nS,1)*row + 0.5]; % 500 random points 
pts_bool_filter = MinDist2Edges(points, map)>0.1;  % filter pts near edge
milestones = [start; points(pts_bool_filter, :); finish]; % update milest.
end