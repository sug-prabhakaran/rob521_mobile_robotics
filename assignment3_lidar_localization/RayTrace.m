function [x_idxs, y_idxs] = RayTrace(x_pixel, y_pixel, theta, angle, range)
% RAYTRACE - calculate pixel indexes of a ray given starting point, angle
% and range.
% INPUTS
% x_pixel (int), y_pixel (int) - start coordinates expressed in pixels
% theta (float)                - pose angle of robot/particle 
% angle (float)                - angle relative to pose angle of laser beam
% range (float)                - distance (meters) of laser beam
% OUTPUT
% x_idxs (1 x N array)         - x indices of pixels in path of ray
% y_idxs (1 x N array)         - y indices of pixels in path of ray

%laser scan range
r_max_laser = 10;

% make sure range does not exceed maximum laser range
if range > r_max_laser
    range = 10;
end

range_pixel = range/0.05;        % express range in terms of pixels
% disp('range_pixel:')
% disp(range_pixel)

% calculate angle of beam relative to x-axis
phi = theta + angle;             % angle wrt x-axis is theta + angle
phi = atan2(sin(phi), cos(phi)); % normalize angle between (-pi, pi)

% if beam angle (phi) is closer to y-axis (ex. between 45-135 degrees)
if (phi < 3*pi/4 && phi > pi/4) || (phi > -3*pi/4 && phi < -pi/4)
    % project range onto y-axis
    y_range_pixel = ceil(range_pixel*sin(phi));

    % reduce projection of range onto y-axis if projection exceeds bounds
    if y_range_pixel + y_pixel > 180
        y_range_pixel = 180-y_pixel;
    end
    if y_range_pixel + y_pixel < 0
        y_range_pixel = 0 - y_pixel;
    end

    % create an array of indices starting at 1/-1 to projected range
    if y_range_pixel < 0
        y_idxs = (1:abs(y_range_pixel))*-1;
    else
        y_idxs = 1:y_range_pixel;
    end
    x_idxs = y_idxs/tan(phi);   % calculate corresponding x index

% if beam angle (phi) is closer to x-axis (ex. between -45-45 degrees)
else
    % project range onto x-axis
    x_range_pixel = ceil(range_pixel*cos(phi));

    % reduce projection of range onto x-axis if projection exceeds bounds
    if x_range_pixel + y_pixel > 300
        x_range_pixel = 300-y_pixel;
    elseif x_range_pixel + x_pixel < 0
        x_range_pixel = 0 - x_pixel;
    end

    % create an array of indices starting at 1/-1 to projected range
    if x_range_pixel < 0
        x_idxs = (1:abs(x_range_pixel)) * -1;
    else
        x_idxs = 1:x_range_pixel;
    end
    y_idxs = x_idxs*tan(phi);  % calculate corresponding x index
end

% round indices to int
x_idxs = round(x_idxs + x_pixel,0);
y_idxs = round(y_idxs + y_pixel,0);
return