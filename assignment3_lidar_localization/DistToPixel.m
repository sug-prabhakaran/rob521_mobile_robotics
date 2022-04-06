function [x_pixel, y_pixel] = DistToPixel(x_coord, y_coord)
% DISTTOPIXEL - convert x, y coordinates from meters to pixels

ogxmin = -7;        % minimum x-value in meters
ogymin = -3;        % minimum y-value in meters
ogres = 0.05;       % resolution: 1 pixel = 0.05m 

x_pixel = floor((x_coord-ogxmin)/ogres);        % (x-(-7))/0.05
y_pixel = floor((y_coord-ogymin)/ogres);        % (y -(-3))/0.05
return