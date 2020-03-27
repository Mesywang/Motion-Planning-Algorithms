clear all; close all; clc;
%% 参数初始化
x_I = 1; y_I = 1;           % 设置初始点
x_G = 750; y_G = 750;       % 设置目标点
GoalThreshold = 30;         % 设置目标点阈值
Delta = 30;                 % 设置扩展步长 default:30
RadiusForNeib = 80;         % rewire的范围,半径r
MaxIterations = 2500;       % 最大迭代次数
UpdateTime = 50;            % 更新路径的时间间隔
DelayTime = 0.0;            % 绘图延迟时间
%% 建树初始化:T是树,v是节点
T.v(1).x = x_I;             % 把起始节点加入到T中
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;         % 节点的父节点坐标:起点的父节点是其本身
T.v(1).yPrev = y_I;
T.v(1).totalCost = 0;         % 从起始节点开始的累计cost，这里取欧氏距离
T.v(1).indPrev = 0;         % 父节点的索引
%% 开始构建树
figure(1);
ImpRgb = imread('map.png');
Imp = rgb2gray(ImpRgb);
imshow(Imp)
xL = size(Imp,1);   % 地图x轴长度
yL = size(Imp,2);   % 地图y轴长度
hold on
plot(x_I, y_I, 'mo', 'MarkerSize',10, 'MarkerFaceColor','m');   % 绘制起点和目标点
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
count = 1;
pHandleList = [];
lHandleList = [];
resHandleList = [];
findPath = 0;
update_count = 0;
path.pos = [];
for iter = 1:MaxIterations
    
    %Step 1: 在地图中随机采样一个点x_rand (Sample)
    x_rand = [unifrnd(0,xL),unifrnd(0,yL)];	%产生随机点(x,y)
    
    %Step 2: 遍历树，从树中找到最近邻近点x_near (Near)
    minDis = sqrt((x_rand(1) - T.v(1).x)^2 + (x_rand(2) - T.v(1).y)^2);
    minIndex = 1;
    for i = 2:size(T.v,2)	% T.v按行向量存储，size(T.v,2)获得节点总数
    	distance = sqrt((x_rand(1) - T.v(i).x)^2 + (x_rand(2) - T.v(i).y)^2);   %两节点间距离
        if(distance < minDis)
            minDis = distance;
            minIndex = i;   
        end     
    end
    
    x_near(1) = T.v(minIndex).x;    % 找到当前树中离x_rand最近的节点
    x_near(2) = T.v(minIndex).y;
    temp_parent = minIndex;         % 临时父节点的索引
    temp_cost = Delta + T.v(minIndex).totalCost;   % 临时累计代价

    %Step 3: 扩展得到x_new节点 (Steer)
    theta = atan2((x_rand(2) - x_near(2)),(x_rand(1) - x_near(1)));
    x_new(1) = x_near(1) + cos(theta) * Delta;
    x_new(2) = x_near(2) + sin(theta) * Delta;  
    %plot(x_rand(1), x_rand(2), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
    %plot(x_new(1), x_new(2), 'bo', 'MarkerSize',10, 'MarkerFaceColor','b');
    
    % 检查节点是否是collision-free
    if ~collisionChecking(x_near,x_new,Imp) 
        continue;   %有障碍物
    end

    %Step 4: 在以x_new为圆心,半径为R的圆内搜索节点 (NearC)
    disToNewList = [];    % 每次循环要把队列清空
    nearIndexList = [];
    for index_near = 1:count
        disTonew = sqrt((x_new(1) - T.v(index_near).x)^2 + (x_new(2) - T.v(index_near).y)^2);
        if(disTonew < RadiusForNeib)    % 满足条件:欧氏距离小于R
            disToNewList = [disToNewList disTonew];     % 满足条件的所有节点到x_new的cost
            nearIndexList = [nearIndexList index_near];     % 满足条件的所有节点基于树T的索引
        end
    end
    
    %Step 5: 选择x_new的父节点,使x_new的累计cost最小 (ChooseParent)
    for cost_index = 1:length(nearIndexList)    % cost_index是基于disToNewList的索引,不是整棵树的索引
        costToNew = disToNewList(cost_index) + T.v(nearIndexList(cost_index)).totalCost;
        if(costToNew < temp_cost)    % temp_cost为通过minDist节点的路径的cost
            x_mincost(1) = T.v(nearIndexList(cost_index)).x;     % 符合剪枝条件节点的坐标
            x_mincost(2) = T.v(nearIndexList(cost_index)).y;
            if ~collisionChecking(x_mincost,x_new,Imp) 
            	continue;   %有障碍物
            end
        	temp_cost = costToNew;
        	temp_parent = nearIndexList(cost_index);
        end
    end
    
    %Step 6: 将x_new插入树T (AddNodeEdge)
    count = count+1;    %最新节点的索引
    
    T.v(count).x = x_new(1);          
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = T.v(temp_parent).x;     
    T.v(count).yPrev = T.v(temp_parent).y;
    T.v(count).totalCost = temp_cost; 
    T.v(count).indPrev = temp_parent;     %其父节点x_near的index
    
   l_handle = plot([T.v(count).xPrev, x_new(1)], [T.v(count).yPrev, x_new(2)], 'b', 'Linewidth', 2);
   p_handle = plot(x_new(1), x_new(2), 'ko', 'MarkerSize', 4, 'MarkerFaceColor','k');
   
   pHandleList = [pHandleList p_handle];    %绘图的句柄索引即为count
   lHandleList = [lHandleList l_handle];
   pause(DelayTime);
    %Step 7: 剪枝 (rewire)
    for rewire_index = 1:length(nearIndexList)
        if(nearIndexList(rewire_index) ~= temp_parent)    % 若不是之前计算的最小cost的节点
            newCost = temp_cost + disToNewList(rewire_index);    % 计算neib经过x_new再到起点的代价          
            if(newCost < T.v(nearIndexList(rewire_index)).totalCost)    % 需要剪枝
                x_neib(1) = T.v(nearIndexList(rewire_index)).x;     % 符合剪枝条件节点的坐标
                x_neib(2) = T.v(nearIndexList(rewire_index)).y;
                if ~collisionChecking(x_neib,x_new,Imp) 
                    continue;   %有障碍物
                end
                T.v(nearIndexList(rewire_index)).xPrev = x_new(1);      % 对该neighbor信息进行更新
                T.v(nearIndexList(rewire_index)).yPrev = x_new(2);
                T.v(nearIndexList(rewire_index)).totalCost = newCost;
                T.v(nearIndexList(rewire_index)).indPrev = count;       % x_new的索引
                
                %delete(pHandleList());
                %delete(lHandleList(nearIndexList(rewire_index)));
                lHandleList(nearIndexList(rewire_index)) = plot([T.v(nearIndexList(rewire_index)).x, x_new(1)], [T.v(nearIndexList(rewire_index)).y, x_new(2)], 'r', 'Linewidth', 2);

                %pHandleList = [pHandleList p_handle];    %绘图的句柄索引即为count
                %lHandleList = [lHandleList l_handle];
            end
        end
    end
    
    %Step 8:检查是否到达目标点附近 
    disToGoal = sqrt((x_new(1) - x_G)^2 + (x_new(2) - y_G)^2);
    if(disToGoal < GoalThreshold && ~findPath)    % 找到目标点，此条件只进入一次
        findPath = 1;

        count = count+1;    %手动将Goal加入到树中
        Goal_index = count;
        T.v(count).x = x_G;          
        T.v(count).y = y_G; 
        T.v(count).xPrev = x_new(1);     
        T.v(count).yPrev = x_new(2);
        T.v(count).totalCost = T.v(count - 1).totalCost + disToGoal;
        T.v(count).indPrev = count - 1;     %其父节点x_near的index
    end
    
    if(findPath == 1)
        update_count = update_count + 1;
        if(update_count == UpdateTime)
            update_count = 0;
            j = 2;
            path.pos(1).x = x_G; 
            path.pos(1).y = y_G;
            pathIndex = T.v(Goal_index).indPrev;
            while 1     
                path.pos(j).x = T.v(pathIndex).x;
                path.pos(j).y = T.v(pathIndex).y;
                pathIndex = T.v(pathIndex).indPrev;    % 沿终点回溯到起点
                if pathIndex == 0
                    break
                end
                j=j+1;
            end  
            
            for delete_index = 1:length(resHandleList)
            	delete(resHandleList(delete_index));
            end
            for j = 2:length(path.pos)
                res_handle = plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'g', 'Linewidth', 4);
                resHandleList = [resHandleList res_handle];
            end
        end
    end  
	pause(DelayTime); %暂停DelayTime s,使得RRT*扩展过程容易观察
end

for delete_index = 1:length(resHandleList)
	delete(resHandleList(delete_index));
end
for j = 2:length(path.pos)
	res_handle = plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'g', 'Linewidth', 4);
	resHandleList = [resHandleList res_handle];
end
            
disp('The path is found!');

