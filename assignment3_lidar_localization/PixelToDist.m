function [x_coord, y_coord] = PixelToDist(x_pixel, y_pixel)
% PIXELTODIST - convert x, y coordinates from pixels to meters

ogxmin = -7;        % minimum x-value in meters
ogymin = -3;        % minimum y-value in meters
ogres = 0.05;       % resolution: 1 pixel = 0.05m 

x_coord = (x_pixel*ogres) + ogxmin;             % (x_p*0.05)+(-7)
y_coord = (y_pixel*ogres) + ogymin;             % (y_p*0.05)+(-3)
return