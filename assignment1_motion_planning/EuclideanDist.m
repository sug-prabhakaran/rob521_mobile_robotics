function dist = EuclideanDist(P1, P2)
% EUCLIDEANDIST - Calculate euclidean distance (l2 norm) for 2D points
% inputs: P1 (2x1 array) - Point 1 of form: [P1_x, P1_y]
%         P2 (2x1 array) - Point 2 of form: [P2_x, P2_y]
% output: dist (scalar)  - distance between P1, P2

dist = sqrt( ( P2(1)-P1(1) )^2 + ( P2(2)-P1(2) )^2 );
end