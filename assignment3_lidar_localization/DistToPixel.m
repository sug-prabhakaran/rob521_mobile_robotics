function [x_pixel, y_pixel] = DistToPixel(x_coord, y_coord)

ogxmin = -7;
%ogymax = 6;
ogymin = -3;
ogres = 0.05;
%ogny = (ogymax-ogymin)/ogres;

x_pixel = floor((x_coord-ogxmin)/ogres);        % (x-(-7))/0.05
y_pixel = floor((y_coord-ogymin)/ogres);        % (y -(-3))/0.05
return