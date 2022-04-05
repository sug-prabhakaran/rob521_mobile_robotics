function [x_coord, y_coord] = PixelToDist(x_pixel, y_pixel)

ogxmin = -7;
ogymin = -3;
ogres = 0.05;

x_coord = (x_pixel*ogres) + ogxmin;             % (x_p*0.05)+(-7)
y_coord = (y_pixel*ogres) + ogymin;             % (y_p*0.05)+(-3)

return