function [x_idxs, y_idxs] = RayTrace(x_pixel, y_pixel, theta, angle, range)

%laser scan range
r_max_laser = 10;

% make sure range does not exceed maximum laser range
if range > r_max_laser
    range = 10;
end

range_pixel = range/0.05;        % express range in terms of pixels
% disp('range_pixel:')
% disp(range_pixel)

phi = theta + angle;             % angle wrt x-axis is theta + angle
phi = atan2(sin(phi), cos(phi)); % normalize angle between (-pi, pi)


if (phi < 3*pi/4 && phi > pi/4) || (phi > -3*pi/4 && phi < -pi/4)
    y_range_pixel = ceil(range_pixel*sin(phi));
%     disp('y_range_pixel:')
%     disp(y_range_pixel)
    if y_range_pixel + y_pixel > 180
        y_range_pixel = 180-y_pixel;
    end
    if y_range_pixel + y_pixel < 0
        y_range_pixel = 0 - y_pixel;
    end
    if y_range_pixel < 0
        y_idxs = (1:abs(y_range_pixel))*-1;
    else
        y_idxs = 1:y_range_pixel;
    end
    x_idxs = y_idxs/tan(phi);
else
    x_range_pixel = ceil(range_pixel*cos(phi));
%     disp('x_range_pixel:')
%     disp(x_range_pixel)
    if x_range_pixel + y_pixel > 300
        x_range_pixel = 300-y_pixel;
    elseif x_range_pixel + x_pixel < 0
        x_range_pixel = 0 - x_pixel;
    end
    if x_range_pixel < 0
        x_idxs = (1:abs(x_range_pixel)) * -1;
    else
        x_idxs = 1:x_range_pixel;
    end
    y_idxs = x_idxs*tan(phi);  
    %disp('y_idxs')
    %disp(y_idxs)
end

x_idxs = round(x_idxs + x_pixel,0);
y_idxs = round(y_idxs + y_pixel,0);
return