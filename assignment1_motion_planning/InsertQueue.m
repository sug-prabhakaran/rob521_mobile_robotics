function [updated_queue] = InsertQueue(queue, vertex, cost)
% INSERTQUEUE - Insert vertex into  queue in ascend. order of cost-to-come
% inputs: queue (Nx5 array) - of row form: [Vi_x, Vi_y, 3.21, Vj_x, Vj_y]
%         vertex (2x1 array) - point of form: [Vi_x, Vi_y];
%         cost (scalar) - cost-to-come from start to vertex 'Vi'
% output: updated_queue (Nx3 array) - with vertex 'Vi' inserted

if (isempty(queue))     % if queue is empty, insert into empty array
    updated_queue = [vertex, cost];
    return

else                    % else, find id of queue elm with cost < Vi cost 
    idx = find(queue(:,3) < cost, 1, 'last'); 

    if (isempty(idx))   % if all elms cost > Vi cost, insert Vi at front
        updated_queue = [[vertex, cost]; queue(:, :)];
        return
    else                % otherwise, insert Vi right after idx
        updated_queue = [queue(1:idx,:); [vertex, cost]; ...
                         queue(idx+1:end, :)];
        return
    end
end
return