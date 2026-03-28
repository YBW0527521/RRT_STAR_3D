function Path = RRTStar(startPoint, axisStart, axisLWH, goalPoint, cubeInfo, cylinderInfo, sphereInfo)
%% 原始RRT*算法实现
rng('shuffle');  % 使用系统时间作为随机种子
totalTime = tic;  % 开始计时

%% 参数设置
calcuDis = @(x,y) sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2); 
iterMax = 10000;   % 最大迭代次数
iter = 0;         % 当前迭代次数
searchRadius = 0.15; % 搜索半径
step = 0.05;      % 基础步长
Thr = 0.1;        % 阈值

% 关闭图形显示以加快规划速度
set(0,'DefaultFigureVisible','off');

%% 初始化树
nodes = zeros(iterMax, 3);  % 存储所有节点坐标
nodes(1,:) = startPoint;    % 第一个节点是起点
costs = zeros(iterMax, 1);  % 从起点到该节点的代价
costs(1) = 0;               % 起点代价为0
parents = zeros(iterMax, 1); % 各节点的父节点索引
nodeCount = 1;              % 当前节点数

% 初始化碰撞计数
collisionCount = 0;

% 存储边的信息数组
edges = zeros(iterMax, 6);  % [x1,y1,z1,x2,y2,z2]
edge_count = 0;

% 路径规划计时
planningTimeStart = tic;

%% 主循环
while iter < iterMax
    iter = iter + 1;
    if mod(iter, 500) == 0
        fprintf('当前迭代: %d\n', iter);
    end
    
    % 随机采样
    if rand < 0.01  % 1%的概率采样目标点
        randPoint = goalPoint;
    else
        randPoint = [rand*axisLWH(1)+axisStart(1), ...
                    rand*axisLWH(2)+axisStart(2), ...
                    rand*axisLWH(3)+axisStart(3)];
    end
    
    % 找到最近节点
    minDist = inf;
    nearestIdx = 0;
    for i = 1:nodeCount
        dist = calcuDis(nodes(i,:), randPoint);
        if dist < minDist
            minDist = dist;
            nearestIdx = i;
        end
    end
    
    % 按步长方向扩展
    nearestNode = nodes(nearestIdx,:);
    direction = randPoint - nearestNode;
    norm_direction = norm(direction);
    
    if norm_direction > 0
        direction = direction / norm_direction;
        newNode = nearestNode + step * direction;
    else
        continue;
    end
    
    % 碰撞检测
    if isCubeCollision(cubeInfo, nearestNode, newNode, step) || ...
       isCylinderCollision(cylinderInfo, nearestNode, newNode, step) || ...
       isSphereCollision(sphereInfo, nearestNode, newNode, step)
        collisionCount = collisionCount + 1;
        continue;
    end
    
    % 找到搜索半径内的邻近节点
    nearNodes = [];
    for i = 1:nodeCount
        if calcuDis(nodes(i,:), newNode) <= searchRadius
            nearNodes = [nearNodes, i];
        end
    end
    
    % 选择最优父节点
    minCost = inf;
    minIdx = nearestIdx;
    
    for i = 1:length(nearNodes)
        idx = nearNodes(i);
        potentialCost = costs(idx) + calcuDis(nodes(idx,:), newNode);
        
        % 检查连接是否有效
        if ~isCubeCollision(cubeInfo, nodes(idx,:), newNode, step) && ...
           ~isCylinderCollision(cylinderInfo, nodes(idx,:), newNode, step) && ...
           ~isSphereCollision(sphereInfo, nodes(idx,:), newNode, step)
            
            if potentialCost < minCost
                minCost = potentialCost;
                minIdx = idx;
            end
        end
    end
    
    % 添加新节点
    nodeCount = nodeCount + 1;
    nodes(nodeCount,:) = newNode;
    parents(nodeCount) = minIdx;
    costs(nodeCount) = minCost;
    
    % 存储边信息
    edge_count = edge_count + 1;
    edges(edge_count,:) = [nodes(minIdx,:), newNode];
    
    % 重布线(Rewiring)
    for i = 1:length(nearNodes)
        idx = nearNodes(i);
        if idx == minIdx
            continue;
        end
        
        potentialCost = costs(nodeCount) + calcuDis(newNode, nodes(idx,:));
        
        if potentialCost < costs(idx)
            % 检查连接是否有效
            if ~isCubeCollision(cubeInfo, newNode, nodes(idx,:), step) && ...
               ~isCylinderCollision(cylinderInfo, newNode, nodes(idx,:), step) && ...
               ~isSphereCollision(sphereInfo, newNode, nodes(idx,:), step)
                
                parents(idx) = nodeCount;
                costs(idx) = potentialCost;
                
                % 更新边信息
                edge_count = edge_count + 1;
                edges(edge_count,:) = [newNode, nodes(idx,:)];
            end
        end
    end
    
    % 检查是否接近目标
    if calcuDis(newNode, goalPoint) < Thr
        % 检查能否无碰撞直连终点
        if ~isCubeCollision(cubeInfo, newNode, goalPoint, step) && ...
           ~isCylinderCollision(cylinderInfo, newNode, goalPoint, step) && ...
           ~isSphereCollision(sphereInfo, newNode, goalPoint, step)
            nodeCount = nodeCount + 1;
            nodes(nodeCount,:) = goalPoint;
            parents(nodeCount) = find(all(nodes(1:nodeCount-1,:) == newNode,2),1);
            costs(nodeCount) = costs(parents(nodeCount)) + calcuDis(newNode, goalPoint);
            
            % 存储最后一条边
            edge_count = edge_count + 1;
            edges(edge_count,:) = [newNode, goalPoint];
            break;
        end
    end
end

% 记录路径规划时间
planningTime = toc(planningTimeStart);

% 检查是否找到路径
if iter == iterMax
    Path = [];
    disp('路径规划失败');
    return;
end

%% 寻找路径
pathCount = 1;
Path = [];
currentIdx = nodeCount;

while currentIdx ~= 0
    Path(pathCount,:) = nodes(currentIdx,:);
    currentIdx = parents(currentIdx);
    pathCount = pathCount + 1;
end

% 将路径翻转以使从起点到终点
Path = flipud(Path);

% 将探索相关信息存储在Path结构体中
Path_info.edges = edges(1:edge_count,:);
Path_info.path = Path;
Path_info.iter = iter;
Path_info.count = nodeCount;
Path_info.collisionCount = collisionCount;
Path_info.planningTime = planningTime;
Path_info.totalTime = toc(totalTime);
Path_info.edge_countA = edge_count;

% 返回结构体
Path = Path_info;

% 输出统计信息
fprintf('规划成功!\n');
fprintf('总的迭代次数: %d\n', iter);
fprintf('总节点数: %d\n', nodeCount);
fprintf('有效节点数: %d\n', nodeCount - collisionCount);
fprintf('路径长度: %.2f m\n', sum(sqrt(sum(diff(Path.path).^2,2))));
fprintf('路径规划时间: %.3f秒\n', planningTime);
fprintf('总执行时间: %.3f秒\n', Path_info.totalTime);

end