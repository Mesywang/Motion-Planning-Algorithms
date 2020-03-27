function visualize_map(map,path)
%This function visualizes the 2D grid map 
%consist of obstacles/start point/target point/optimal path.

    % obstacles
    for obs_cnt = 2: size(map, 1) - 1
        %scatter(map(obs_cnt, 1)-0.5,map(obs_cnt, 2)-0.5,250,155,'filled');  %绘制障碍物的散点图
        scatter(map(obs_cnt, 1)-0.5,map(obs_cnt, 2)-0.5,280,'k','filled');  %绘制障碍物的散点图
        hold on;
        grid on;
        %grid minor;
        axis equal;        
        axis ([0 10 0 10 ]);   %坐标系横纵坐标
        hold on;
    end
    % start point
    scatter(map(1, 1)-0.5, map(1, 2)-0.5,100,'b','p');
    hold on;
    % target point
	scatter(map(size(map, 1), 1)-0.5, map(size(map, 1), 2)-0.5,100,'r','p');
    hold on;
    %optimal path
    for path_cnt = 2:size(path,1)-1  % size(path,1)取出path的行数
        scatter(path(path_cnt,1)-0.5,path(path_cnt,2)-0.5,120,155,'filled');
        hold on;
    end

end

