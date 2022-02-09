function show_maze(map,row,col,h)

map = [map(:,1) map(:,3) map(:,2) map(:,4)];

figure(h)

%% Add border
%line([.5,col+.5],[.5,.5]) % draw top border
%line([.5,col+.5],[row+.5,row+.5]) % draw bottom border
%line([.5,.5],[1.5,row+.5]) % draw left border
%line([col+.5,col+.5],[.5,row-.5])  % draw right border

for ii = 1:length(map(:,1))
    line(map(ii,1:2),map(ii,3:4));
end

axis equal
axis([.5,col+.5,.5,row+.5])
axis off
return