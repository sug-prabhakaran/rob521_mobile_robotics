function [spath] = BackTrack(spath, visited, milestones)
% BACKTRACK - backtrack from finish vertex to start vertex through 
%             respective parent nodes until the parent node = current node
% input: spath (1x1 array)      - contains index of finish node
%        visited (Nx5 array)    - contains data on parent nodes in col 4:5
%        milestones (Nx2 array) - contains all milestone vertices in graph
% output: spath (1xM array)     - contains updated indexes of milestones in 
%                                 order start to finish on shortest path

% initialize current and parent nodes
cur_node = milestones(spath(end), :);    % set current node to finish
[~, cur_idx_vis] = ismember(cur_node, visited(:,1:2),'rows');%parent idx in visited
parent_node = visited(cur_idx_vis,4:5);          % set parent node

% loop through starting at finish node and add parent nodes to spath
% loop until parent = cur
while (~(parent_node(1) == cur_node(1)) || ~(parent_node(2) == cur_node(2)))
    % get parent idx from milestone
    [~, parent_idx_mil] = ismember(parent_node, milestones, 'rows');
    spath = [parent_idx_mil; spath];         % add parent node to spath
    cur_node = parent_node;              % set parent node to current node
    [~, parent_idx_vis] = ismember(cur_node, visited(:,1:2),'rows'); % new parent id
    parent_node = visited(parent_idx_vis,4:5);      % set new parent node
end
end