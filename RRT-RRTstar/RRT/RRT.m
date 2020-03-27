clear all; close all; clc;
%% 参数初始化
x_I = 1; y_I = 1;           % 设置初始点
x_G = 750; y_G = 750;       % 设置目标点
Thr = 30;                   % 设置目标点阈值
Delta = 40;                 % 设置扩展步长 
%% 建树初始化
T.v(1).x = x_I;           	% T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;         % 起始节点的父节点仍然是其本身
T.v(1).yPrev = y_I;
T.v(1).dist=0;              % 从父节点到该节点的距离，这里可取欧氏距离
T.v(1).indPrev = 0;         % 父节点的索引
%% 开始构建树
figure(1);
ImpRgb = imread('map.png');
Imp = rgb2gray(ImpRgb);
imshow(Imp)
xL = size(Imp,1);   % 地图x轴长度
yL = size(Imp,2);   % 地图y轴长度
hold on
plot(x_I, y_I, 'mo', 'MarkerSize',10, 'MarkerFaceColor','m');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');   % 绘制起点和目标点
count = 1;
for iter = 1:3000

    %Step 1: 在地图中随机采样一个点x_rand
    x_rand = [unifrnd(0,800),unifrnd(0,800)];	% 产生随机点(x,y)
    
    %Step 2: 遍历树，从树中找到最近邻近点x_near 
    minDis = sqrt((x_rand(1) - T.v(1).x)^2 + (x_rand(2) - T.v(1).y)^2);
    minIndex = 1;
    for i = 2:size(T.v,2)	% T.v按行向量存储，size(T.v,2)获得节点总数
    	distance = sqrt((x_rand(1) - T.v(i).x)^2 + (x_rand(2) - T.v(i).y)^2);   % 两节点间距离
        if(distance < minDis)
            minDis = distance;
            minIndex = i;   
        end     
    end
    x_near(1) = T.v(minIndex).x;    % 找到当前树中离x_rand最近的节点
    x_near(2) = T.v(minIndex).y;
    
    %Step 3: 扩展得到x_new节点
    theta = atan2((x_rand(2) - x_near(2)),(x_rand(1) - x_near(1)));
    x_new(1) = x_near(1) + cos(theta) * Delta;
    x_new(2) = x_near(2) + sin(theta) * Delta;  
    
    
    %检查节点是否是collision-free
    if ~collisionChecking(x_near,x_new,Imp) 
        continue;   % 有障碍物
    end
    
    count = count+1;
    
    %Step 4: 将x_new插入树T 
    T.v(count).x = x_new(1);          
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = x_near(1);     
    T.v(count).yPrev = x_near(2);
    T.v(count).dist = Delta;
    T.v(count).indPrev = minIndex;     % 其父节点x_near的index
    
    %Step 5:检查是否到达目标点附近 
    disToGoal = sqrt((x_new(1) - x_G)^2 + (x_new(2) - y_G)^2);
    if(disToGoal < Thr)
        break
    end
   %Step 6:将x_near和x_new之间的路径画出来
   plot([x_near(1), x_new(1)], [x_near(2), x_new(2)], 'b', 'Linewidth', 2);
   plot(x_new(1), x_new(2), 'ko', 'MarkerSize', 4, 'MarkerFaceColor','k');
   
   pause(0.02);     % 暂停0.02s，使得RRT扩展过程容易观察
end
%% 路径已经找到，反向查询
if iter < 2000
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % 终点加入路径
    j=0;
    while 1
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % 沿终点回溯到起点
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % 起点加入路径
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'g', 'Linewidth', 4);
    end
else
    disp('Error, no path found!');
end
