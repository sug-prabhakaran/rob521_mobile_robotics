function [y_laser_pred] = PredictRange(x, y, theta, angle, ogp)
% PREDICTRANGE - Calculate laser range for a given pose, laser heading and
% occupancy grid.
% INPUTS
% x, y, theta (float)   - current pose of particle
% angle (float)         - laser heading wrt part.frame (theta = 0 degrees)
% ogp (180 x 300 array) - occupancy grid with probability of occupation
% OUTPUT
% y_laser_pred (float)  - predicted laser measurement

% INITIAL PARAMETERS

range_max = 10;                     % use max_distance + 2 x std_dev (0.5)
threshold = 0.5;                    % cell occupied if prob > 0.5

% convert from x,y coord to pixel coordinates
[x_pixel, y_pixel] = DistToPixel(x, y);
% find all pixels along ray from particle pose
[x_idxs, y_idxs] = RayTrace(x_pixel, y_pixel, theta, angle, range_max);

J = size(x_idxs, 2);                % number of pixels to test

% if J is empty, return range = 0
if J == 0
    y_laser_pred = 0;
    return
end

max_index = 1;  % index of furthest unoccupied pixel

% loop through each pixel from current particle until cell is occupied
for j=1:J
    % skip over indices if outside bounds
    if y_idxs(j) > 180 || x_idxs(j) > 300 || y_idxs(j) < 1 || x_idxs(j) < 1
        continue
    end
    if ogp(y_idxs(j),x_idxs(j)) >= threshold
        max_index = j;
        break
    end
end

% convert end point from pixel back to x,y coords
[x_end, y_end] = PixelToDist(x_idxs(max_index), y_idxs(max_index));

% calculate euclidean distance between particle loc to ray end point
y_laser_pred = sqrt((x_end - x)^2 + (y_end - y)^2);
return
