function n_index = node_index(OPEN,xval,yval)
    %This function returns the index of the location of a node in the list
    %OPEN
    %
    %   Copyright 2009-2010 The MathWorks, Inc.
    i=1;
    open_count = size(OPEN,1);
    while(OPEN(i,2) ~= xval || OPEN(i,3) ~= yval )
        i=i+1;
        if(i == open_count+1)  % 若OPEN list中没有需查找的节点，则返回当前OPEN list中节点总+1
            break
        end
    end
    n_index=i;
        
end