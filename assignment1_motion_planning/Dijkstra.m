function [spath, visited] = Dijkstra(edges, milestones)
% DIJKSTRA - Find shortest path from start node to finish node
% inputs: edges (Mx4 array) - each 1x4 row is edge: [x1 y1 x2 y2];
%         milestones (Nx2 array) - each row is a point of form: [xi yi];
%         spath (1xJ array) - contains indices of nodes on shortest path in
%                             milestones array in order of start to finish
% outputs: spath (1x1 array) -index of finish node if alg 'success'
%          visited (Nx5 array) - row form: [xi yi cost parent_x parent_y]; 

% 1. Create and initialize variables, arrays
start = milestones(1,:);      % start is first node in milestones
finish = milestones(end, :);  % finish is last node in milestones
queue = [start, 0];    % pri. queue of unvisited nodes + cost-to-come (c2c)
visited = [start, 0, start;]; % list of visited nodes + c2c + parent node


% 2. Implement Dijkstra's Algorithm 
while ~isempty(queue)       % run as long as nodes remaining in queue

    % A. Get First Node
    x = queue(1,1:2);       % new node 'x' first queue elm first 2 cols
    %scatter(x(1), x(2), 'co', 'filled')
    x_cost = queue(1, 3);   % cost-to-come of new node 'x'
    queue = queue(2:end,:); % pop vertex 'x' from queue

    % B. Termination Condition: if finish node reached
    if (finish == x)        
        [~, idx] = ismember(finish, milestones, 'rows'); % store finish idx 
        spath = [idx];
        disp('Success')
        return
    end

    % C. find all reachable neighbors of 'x'
    neighbors = FindNeighbors(edges, x);    % rtn neighboring nodes array
    [J,~] = size(neighbors);                % total J neighbors

    % D. Iterate through each neighbor
    for j=1:J                               % iterate thru each neighbor
        x_prime = neighbors(j,:);           % set neighbor node x'
        %scatter(x_prime(1),x_prime(2), 'ro', 'filled')

    % E. For neighbors not visited, update Queue and Visited arrays
        if ~(ismember(x_prime, visited(:,1:2), 'rows'))   % if not visited
            cost_come = x_cost + EuclideanDist(x_prime, x); % x' c2c
            visited(end+1,:) = [x_prime, cost_come, x];   % mark as visited
            queue = InsertQueue(queue, x_prime, cost_come); % add to queue
        end
    end
end

spath = [];
disp('Unsucessful - Did not find a path')
end