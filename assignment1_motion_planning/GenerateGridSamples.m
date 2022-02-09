function [milestones] = GenerateGridSamples(row, col, start, finish, map)
% GENERATEGRIDSAMPLES - take grid size and generate milestones in grid
% inputs: row (scalar)    - number of rows of grid
%         col (scalar)    - number of cols of grid
%       start (2x1 array) - start point coordinates [xs, ys];
%      finish (2x1 array) - end point cooridnates [xf, yf];
% output: milestones (Nx2 array) - all generated vertices in the grid

% Note. maze bounds: bottom left = [0.5 0.5], top right = [col +.5, row+.5]

points = [start];

% smallest grids, generate 9 points per square 
if (row*col <= 100)
   x_coord = linspace(0.75, col+0.25, col*4-1);
   y_coord = linspace(0.75, row+0.25, row*4-1);

   for i=1:length(x_coord)
        for j =1:length(y_coord)
            if ((mod(x_coord(i)-0.5,1)<0.1) || (mod(y_coord(j)-0.5,1)<0.1))
                dist = MinDist2Edges([x_coord(i), y_coord(j)], map);
                if dist <0.1
                    continue
                else
                    points = [points; x_coord(i), y_coord(j)];
                end
            else
                points = [points; x_coord(i), y_coord(j)];
            end
        end
   end

else
   x_coord = 1:col;
   y_coord = 1:row;

   for i=1:length(x_coord)
        for j =1:length(y_coord)
            points = [points; x_coord(i), y_coord(j)];
        end
   end
end

milestones = [points; finish]; 
end