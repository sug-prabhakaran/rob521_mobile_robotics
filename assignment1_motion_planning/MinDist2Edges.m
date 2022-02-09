function [d] = MinDist2Edges(points,edges)
% Returns the minimum distance to edges for a set of points

n = length(points(:,1));
m = length(edges(:,1));
for ii=1:n
   P = points(ii,:)
   for jj=1:m
        Q1 = edges(jj,1:2);
        Q2 = edges(jj,3:4);
        l2 = norm(Q2-Q1)^2;
        t = dot(P-Q1, Q2-Q1)/l2;
        if (t<0.0)
            dist(ii,jj) = norm(P-Q1);
        elseif (t>1.0)
            dist(ii,jj) = norm(P-Q2);
        else
            dist(ii,jj) = abs(det([Q2-Q1;P-Q1]))/norm(Q2-Q1); % for row vectors.
        end
   end
end

d = min(dist');