function Path = RRTStarConnect(startPoint, axisStart, axisLWH, goalPoint, cubeInfo, cylinderInfo, sphereInfo)
%% RRT*-Connect算法寻找路径规划，适用于控制xArm6机械臂 - 只返回最优路径
rng('shuffle');  % 使用系统时间作为随机种子，确保每次运行结果不同
% 初始计时
totalTime = tic;

%% 参数设置
calcuDis = @(x,y) sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2); 
iterMax = 10000;   % 最大迭代次数
iter = 0;          % 当前迭代次数
searchRadius = 0.06; % 搜索半径

step = 0.03;  % 基础步长
Thr = 0.1;    % 阈值

% 关闭图形显示以加快规划速度
set(0,'DefaultFigureVisible','off');

% 初始化起点树Ta和终点树Tb：
% 起点树
nodesA = zeros(iterMax, 3);
nodesA(1,:) = startPoint;
costsA = zeros(iterMax, 1);
parentsA = zeros(iterMax, 1);
nodeCountA = 1;

% 终点树
nodesB = zeros(iterMax, 3);
nodesB(1,:) = goalPoint;
costsB = zeros(iterMax, 1);
parentsB = zeros(iterMax, 1);
nodeCountB = 1;

% 初始化碰撞计数
collisionCount = 0;

% 存储边的信息数组
edgesA = zeros(iterMax, 6);
edge_countA = 0;
edgesB = zeros(iterMax, 6);
edge_countB = 0;

% 连接点索引
connectIdxA = -1;
connectIdxB = -1;

% 路径规划计时
planningTimeStart = tic;

while iter < iterMax
    iter = iter + 1;
    if mod(iter, 500) == 0
        fprintf('当前迭代: %d\n', iter);
    end
    
    %% 交替扩展两棵树
    % 从起点树Ta向外扩展
    if ~growTree(true)
        continue;
    end
    
    % 尝试连接到Tb
    if connectTrees(true)
        break;
    end
    
    % 从终点树Tb向外扩展
    if ~growTree(false)
        continue;
    end
    
    % 尝试连接到Ta
    if connectTrees(false)
        break;
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

%% 构建最终路径
% 从Ta树连接点回溯到起点
pathA = [];
pathCount = 1;
currentIdx = connectIdxA;
while currentIdx ~= 0
    pathA(pathCount,:) = nodesA(currentIdx,:);
    currentIdx = parentsA(currentIdx);
    pathCount = pathCount + 1;
end

% 翻转pathA
pathA = flipud(pathA);

% 从Tb树连接点回溯到终点
pathB = [];
pathCount = 1;
currentIdx = connectIdxB;
while currentIdx ~= 0
    pathB(pathCount,:) = nodesB(currentIdx,:);
    currentIdx = parentsB(currentIdx);
    pathCount = pathCount + 1;
end

% 合并路径
Path = [pathA; pathB];

% 探索树的信息存储在Path结构中，以便后续可使用
% 合并两棵树的边
all_edges = zeros(edge_countA + edge_countB, 6);
all_edges(1:edge_countA,:) = edgesA(1:edge_countA,:);
all_edges(edge_countA+1:edge_countA+edge_countB,:) = edgesB(1:edge_countB,:);

Path_info.edges = all_edges;
Path_info.path = Path;
Path_info.iter = iter;
Path_info.count = nodeCountA + nodeCountB;
Path_info.collisionCount = collisionCount;
Path_info.planningTime = planningTime;
Path_info.totalTime = toc(totalTime);
Path_info.edge_countA = edge_countA;
Path_info.edge_countB = edge_countB;

% 返回结构体而非简单的路径
Path = Path_info;

% 输出统计信息
fprintf('规划成功!\n');
fprintf('总的迭代次数: %d\n', iter);
fprintf('总节点数: %d\n', nodeCountA + nodeCountB);
fprintf('有效节点数: %d\n', (nodeCountA + nodeCountB) - collisionCount);
fprintf('路径长度: %.2f m\n', sum(sqrt(sum(diff(Path.path).^2,2))));
fprintf('路径规划时间: %.3f秒\n', planningTime);
fprintf('总执行时间: %.3f秒\n', Path_info.totalTime);

%% 嵌套函数：扩展树
    function success = growTree(isTreeA)
        % 确定当前树和目标树
        if isTreeA
            nodes = nodesA;
            costs = costsA;
            parents = parentsA;
            nodeCount = nodeCountA;
            targetNodes = nodesB;
            targetNodeCount = nodeCountB;
        else
            nodes = nodesB;
            costs = costsB;
            parents = parentsB;
            nodeCount = nodeCountB;
            targetNodes = nodesA;
            targetNodeCount = nodeCountA;
        end
        
        % 随机采样
        if rand < 0.1  % 10%概率采样目标树中的点，与RRT一致
            randIdx = randi(targetNodeCount);
            randPoint = targetNodes(randIdx,:);
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
            success = false;
            return;
        end
        
        % 碰撞检测
        if isCubeCollision(cubeInfo, nearestNode, newNode, step) || ...
           isCylinderCollision(cylinderInfo, nearestNode, newNode, step) || ...
           isSphereCollision(sphereInfo, nearestNode, newNode, step)
            collisionCount = collisionCount + 1;
            success = false;
            return;
        end
        
        % 找到搜索半径内的邻近节点
        nearNodes = [];
        for i = 1:nodeCount
            if calcuDis(nodes(i,:), newNode) <= searchRadius
                nearNodes = [nearNodes, i];
            end
        end
        
        % 添加新节点
        nodeCount = nodeCount + 1;
        nodes(nodeCount,:) = newNode;
        
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
        
        % 连接到最优父节点
        parents(nodeCount) = minIdx;
        costs(nodeCount) = minCost;
        
        % 存储边信息
        if isTreeA
            edge_countA = edge_countA + 1;
            edgesA(edge_countA,:) = [nodes(minIdx,1), nodes(minIdx,2), nodes(minIdx,3), newNode(1), newNode(2), newNode(3)];
        else
            edge_countB = edge_countB + 1;
            edgesB(edge_countB,:) = [nodes(minIdx,1), nodes(minIdx,2), nodes(minIdx,3), newNode(1), newNode(2), newNode(3)];
        end
        
        % 重布线(Rewiring)
        for i = 1:length(nearNodes)
            idx = nearNodes(i);
            if idx == minIdx
                continue;
            end
            
            potentialCost = costs(nodeCount) + calcuDis(newNode, nodes(idx,:));
            
            % 检查是否能提供更优的路径
            if potentialCost < costs(idx)
                % 检查连接是否有效
                if ~isCubeCollision(cubeInfo, newNode, nodes(idx,:), step) && ...
                   ~isCylinderCollision(cylinderInfo, newNode, nodes(idx,:), step) && ...
                   ~isSphereCollision(sphereInfo, newNode, nodes(idx,:), step)
                    
                    parents(idx) = nodeCount;
                    costs(idx) = potentialCost;
                    
                    % 更新边信息
                    if isTreeA
                        edge_countA = edge_countA + 1;
                        edgesA(edge_countA,:) = [newNode(1), newNode(2), newNode(3), nodes(idx,1), nodes(idx,2), nodes(idx,3)];
                    else
                        edge_countB = edge_countB + 1;
                        edgesB(edge_countB,:) = [newNode(1), newNode(2), newNode(3), nodes(idx,1), nodes(idx,2), nodes(idx,3)];
                    end
                end
            end
        end
        
        % 更新相应的树
        if isTreeA
            nodesA = nodes;
            costsA = costs;
            parentsA = parents;
            nodeCountA = nodeCount;
        else
            nodesB = nodes;
            costsB = costs;
            parentsB = parents;
            nodeCountB = nodeCount;
        end
        
        success = true;
    end

%% 嵌套函数：尝试连接两棵树
    function success = connectTrees(fromTreeA)
        % 确定源树和目标树
        if fromTreeA
            sourceNodes = nodesA;
            sourceNodeCount = nodeCountA;
            targetNodes = nodesB;
            targetNodeCount = nodeCountB;
        else
            sourceNodes = nodesB;
            sourceNodeCount = nodeCountB;
            targetNodes = nodesA;
            targetNodeCount = nodeCountA;
        end
        
        % 获取源树的新节点
        newNode = sourceNodes(sourceNodeCount,:);
        
        % 在目标树中找最近点
        minDist = inf;
        nearestIdx = 0;
        for i = 1:targetNodeCount
            dist = calcuDis(targetNodes(i,:), newNode);
            if dist < minDist
                minDist = dist;
                nearestIdx = i;
            end
        end
        
        % 检查是否可以直接连接
        if minDist < Thr
            % 检查连接是否有碰撞
            if ~isCubeCollision(cubeInfo, newNode, targetNodes(nearestIdx,:), step) && ...
               ~isCylinderCollision(cylinderInfo, newNode, targetNodes(nearestIdx,:), step) && ...
               ~isSphereCollision(sphereInfo, newNode, targetNodes(nearestIdx,:), step)
                
                % 记录连接点
                if fromTreeA
                    connectIdxA = sourceNodeCount;
                    connectIdxB = nearestIdx;
                    
                    % 存储连接边
                    edge_countA = edge_countA + 1;
                    edgesA(edge_countA,:) = [newNode(1), newNode(2), newNode(3), targetNodes(nearestIdx,1), targetNodes(nearestIdx,2), targetNodes(nearestIdx,3)];
                else
                    connectIdxB = sourceNodeCount;
                    connectIdxA = nearestIdx;
                    
                    % 存储连接边
                    edge_countB = edge_countB + 1;
                    edgesB(edge_countB,:) = [newNode(1), newNode(2), newNode(3), targetNodes(nearestIdx,1), targetNodes(nearestIdx,2), targetNodes(nearestIdx,3)];
                end
                
                success = true;
                return;
            end
        end
        
        success = false;
    end

end 