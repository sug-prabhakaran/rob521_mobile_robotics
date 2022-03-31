function [x_idxs, y_idxs] = RayTrace(x_pixel, y_pixel, theta, angle, range)

%laser scan range
r_max_laser = 10;
r_min_laser = 0.45;

% make sure range does not exceed maximum laser range
if range > r_max_laser
    range = 10;
end

range_pixel = range/0.05;        % express range in terms of pixels
phi = theta + angle;             % angle wrt x-axis is theta + angle
phi = atan2(sin(phi), cos(phi)); % normalize angle

x_range_pixel = ceil(range_pixel*cos(phi));
r_min_laser_pix = floor(r_min_laser/0.05);

x_idxs = r_min_laser_pix:x_range_pixel;
y_idxs = x_idxs*tan(phi);

x_idxs = x_idxs + x_pixel;
y_idxs = round(y_idxs + y_pixel,0);
return