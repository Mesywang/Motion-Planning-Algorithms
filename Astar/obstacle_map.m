 function map = obstacle_map(xStart,yStart,xTarget,yTarget,MAX_X,MAX_Y)
%This function returns a map contains random distribution obstacles.
%函数返回的map是起点+终点+所有障碍物的坐标，不是一个二维地图，二维地图在A_star_search.m中构建
    rand_map = rand(MAX_X,MAX_Y);
    map = [];     %存储地图中所有点的坐标 map[n,2]
    map(1,1) = xStart;  
    map(1,2) = yStart;
    k=2;
    obstacle_ratio = 0.4;
    for i = 1:1:MAX_X
        for j = 1:1:MAX_Y
            if( (rand_map(i,j) < obstacle_ratio) && (i~= xStart || j~=yStart) && (i~= xTarget || j~=yTarget))
                map(k,1) = i;
                map(k,2) = j;
                k=k+1;
            end    
        end
    end
    map(k,1) = xTarget;
    map(k,2) = yTarget;
end

