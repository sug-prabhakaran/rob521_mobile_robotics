function [x_idxs, y_idxs] = RayTrace(x_pixel, y_pixel, theta, angle, range)

%laser scan range
r_max_laser = 10;

% make sure range does not exceed maximum laser range
if range > r_max_laser
%     disp('range over r_max_laser')
%     disp
    range = 10;
end

range_pixel = range/0.05;        % express range in terms of pixels
phi = theta + angle;             % angle wrt x-axis is theta + angle
phi = atan2(sin(phi), cos(phi)); % normalize angle
% disp('angle phi:')
%disp(phi)

if (phi < 3*pi/4 && phi > pi/4) || (phi > -3*pi/4 && phi < -pi/4)
    %disp('phi is inside')
    y_range_pixel = ceil(range_pixel*sin(phi));
    if y_range_pixel < 0
        y_idxs = (0:abs(y_range_pixel))*-1;
    else
        y_idxs = 0:y_range_pixel;
    end
    %disp('divide tan(phi)')
    x_idxs = y_idxs/tan(phi);
    %disp('x_idxs')
    %disp(x_idxs)
else
    %disp('phi is outside')
    x_range_pixel = ceil(range_pixel*cos(phi));
    
    if x_range_pixel < 0
        x_idxs = (0:abs(x_range_pixel)) * -1;
    else
        x_idxs = 0:x_range_pixel;
    end
    y_idxs = x_idxs*tan(phi);  
    %disp('y_idxs')
    %disp(y_idxs)
end

x_idxs = round(x_idxs + x_pixel,0);
y_idxs = round(y_idxs + y_pixel,0);
return