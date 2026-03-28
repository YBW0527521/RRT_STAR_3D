function Path_info = DR_IRRTStar_GC(startPoint, axisStart, axisLWH, goalPoint, cubeInfo, cylinderInfo, sphereInfo)
%% DR-IRRT*-GC算法实现
% 基于Informed RRT*，结合自适应目标偏置、双向搜索、动态节点拒绝和贪婪连接
rng('shuffle');  % 使用系统时间作为随机种子，确保每次运行结果不同
% 初始计时
totalTime = tic;

%% 参数设置
calcuDis = @(x,y) sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2); 
iterMax = 10000;   % 最大迭代次数
iter = 0;          % 当前迭代次数
searchRadius = 0.15; % 搜索半径 (调整到与你原DR_IRRTStar_GC一致)

step = 0.05;  % 基础步长 (保持一致)
Thr = 0.1;    % 阈值 (保持一致)

% --- Informed RRT* 相关参数 ---
c_best = inf;     % 最优路径代价，初始为无穷大
findpath = 0;     % 是否找到路径的标志，0为未找到，1为已找到
no_improve_count = 0;  % 连续未改善次数
max_no_improve = 150;  % 最大连续未改善次数 (参考你原DR_IRRTStar_GC的动态调整前的默认值)
improvement_threshold = 0.005;  % 改善阈值 (从0.05降低到0.001，使算法更容易找到更好的路径)
% --- 新增参数 ---
rejection_prob_base = 0.1; % 动态节点拒绝的基础概率
% ------------------------------


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
collisionCount = 0; % 这个计数可能需要更精细，区分扩展碰撞和节点拒绝

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

%% 主循环
while iter < iterMax
    iter = iter + 1;
    % 打印进度
    if mod(iter, 500) == 0
        fprintf('当前迭代: %d\n', iter);
    end
    
    % --- 轮流扩展两棵树 ---
    % 先扩展起点树A
    % 调用 growTree，传递A树变量和所有必需的参数，包括 iterMax
    [successA, nodesA, costsA, parentsA, nodeCountA, edgesA, edge_countA, current_collision_count] = ...
        growTree(true, nodesA, costsA, parentsA, nodeCountA, edgesA, edge_countA, ...
                 nodesB, nodeCountB, cubeInfo, cylinderInfo, sphereInfo, ...
                 calcuDis, searchRadius, step, iter, iterMax, findpath, c_best, startPoint, goalPoint, ...
                 axisStart, axisLWH, rejection_prob_base, collisionCount);
    collisionCount = current_collision_count; % 更新主函数的碰撞计数


    % 如果成功扩展A，尝试连接到B
    if successA
        % 调用 connectTrees，传递两棵树变量和Informed相关变量，并接收更新后的变量
        [connected, connectIdxA, connectIdxB, c_best, findpath, no_improve_count, edgesA, edge_countA, edgesB, edge_countB] = ...
            connectTrees(true, nodesA, nodeCountA, costsA, ...
                        nodesB, nodeCountB, costsB, ...
                        cubeInfo, cylinderInfo, sphereInfo, calcuDis, Thr, step, ...
                        connectIdxA, connectIdxB, findpath, c_best, no_improve_count, improvement_threshold, max_no_improve, ...
                        edgesA, edge_countA, edgesB, edge_countB, startPoint, goalPoint);

        % 如果连接成功，并且找到了更好的路径（connectTrees会返回connected=true）
        if connected
            % 检查是否应该停止优化 (如果 max_no_improve > 0)
             if findpath == 1 && max_no_improve > 0
                 if no_improve_count >= max_no_improve
                     fprintf('连续%d次未找到更好的路径，停止优化\n', max_no_improve);
                     break; % 退出主循环
                 end
             end
        end
    end

    % 后扩展终点树B (只有在未连接或未停止优化时才执行)
     if ~(successA && connected) || (findpath == 1 && no_improve_count < max_no_improve)
         % 调用 growTree，传递B树变量和所有必需的参数，包括 iterMax
        [successB, nodesB, costsB, parentsB, nodeCountB, edgesB, edge_countB, current_collision_count] = ...
            growTree(false, nodesB, costsB, parentsB, nodeCountB, edgesB, edge_countB, ...
                 nodesA, nodeCountA, cubeInfo, cylinderInfo, sphereInfo, ...
                 calcuDis, searchRadius, step, iter, iterMax, findpath, c_best, startPoint, goalPoint, ...
                 axisStart, axisLWH, rejection_prob_base, collisionCount);
         collisionCount = current_collision_count; % 更新主函数的碰撞计数

         % 如果成功扩展B，尝试连接到A
        if successB
             % 调用 connectTrees，传递两棵树变量和Informed相关变量，并接收更新后的变量
            [connected, connectIdxA, connectIdxB, c_best, findpath, no_improve_count, edgesA, edge_countA, edgesB, edge_countB] = ...
                connectTrees(false, nodesB, nodeCountB, costsB, ...
                            nodesA, nodeCountA, costsA, ...
                            cubeInfo, cylinderInfo, sphereInfo, calcuDis, Thr, step, ...
                            connectIdxA, connectIdxB, findpath, c_best, no_improve_count, improvement_threshold, max_no_improve, ...
                            edgesA, edge_countA, edgesB, edge_countB, startPoint, goalPoint);

             % 如果连接成功，并且找到了更好的路径 (connectTrees会返回connected=true)
             if connected
                 % 检查是否应该停止优化 (如果 max_no_improve > 0)
                 if findpath == 1 && max_no_improve > 0
                     if no_improve_count >= max_no_improve
                         fprintf('连续%d次未找到更好的路径，停止优化\n', max_no_improve);
                         break; % 退出主循环
                     end
                 end
            end
         end
     end

     % 如果在未找到路径阶段 iterMax 迭代完成，规划失败
     if iter == iterMax && findpath == 0
          Path_info = []; % 返回空路径结构体表示失败
          disp('路径规划失败：未找到初始路径');
          return;
     end

end

% 记录路径规划时间
planningTime = toc(planningTimeStart);

% 检查是否找到路径 (如果循环因迭代次数耗尽而退出，但 findpath 为1，说明找到了路径但没再优化)
if findpath == 0
    Path_info = [];
    disp('路径规划失败：未找到路径');
    return;
end


%% 构建最终路径
% ... (构建路径的逻辑不变) ...
% 从Ta树连接点回溯到起点
pathA = [];
pathCount = 1;
currentIdx = connectIdxA;
while currentIdx ~= 0 && currentIdx <= size(nodesA,1) % 增加边界检查
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
while currentIdx ~= 0 && currentIdx <= size(nodesB,1) % 增加边界检查
    pathB(pathCount,:) = nodesB(currentIdx,:);
    currentIdx = parentsB(currentIdx);
    pathCount = pathCount + 1;
end

% 合并路径
% 注意：连接点会被重复添加一次，通常需要移除一个
if ~isempty(pathA) && ~isempty(pathB) && isequal(pathA(end,:), pathB(1,:))
    Path = [pathA; pathB(2:end,:)]; % 移除 pathB 的第一个点（连接点）
else
     Path = [pathA; pathB]; % 如果连接点不完全重合或路径为空，直接合并
end


% 将探索树的信息存储在Path结构中，以便后续可使用
% 合并两棵树的边
all_edges = zeros(edge_countA + edge_countB, 6);
all_edges(1:edge_countA,:) = edgesA(1:edge_countA,:);
all_edges(edge_countA+1:edge_countA+edge_countB,:) = edgesB(1:edge_countB,:);

% 修改返回结构体字段，使其与main.m匹配
Path_info.path = Path;
Path_info.iter = iter;
Path_info.count = nodeCountA + nodeCountB;  % 添加count字段
Path_info.nodeCountA = nodeCountA;  % 保留原有字段
Path_info.nodeCountB = nodeCountB;  % 保留原有字段
Path_info.totalNodeCount = nodeCountA + nodeCountB;  % 保留原有字段
Path_info.collisionCount = collisionCount;
Path_info.planningTime = planningTime;
Path_info.totalTime = toc(totalTime);
Path_info.edges = all_edges;
Path_info.edge_countA = edge_countA;
Path_info.edge_countB = edge_countB;

% 返回结构体
Path_info = Path_info;

% 输出统计信息
fprintf('规划成功!\n');
fprintf('总的迭代次数: %d\n', iter);
fprintf('起点树节点数: %d, 终点树节点数: %d, 总节点数: %d\n', nodeCountA, nodeCountB, nodeCountA + nodeCountB);
fprintf('有效节点数: %d\n', (nodeCountA + nodeCountB) - collisionCount);  % 添加有效节点数统计
fprintf('最终路径长度: %.2f m\n', sum(sqrt(sum(diff(Path_info.path).^2,2))));
fprintf('路径规划时间: %.3f秒\n', planningTime);
fprintf('总执行时间: %.3f秒\n', Path_info.totalTime);

end

%% 嵌套函数：扩展树
    function [success, nodes, costs, parents, nodeCount, edges, edge_count, collisionCount] = growTree(isTreeA, nodes, costs, parents, nodeCount, edges, edge_count, ...
                                                                                                        targetNodes, targetNodeCount, cubeInfo, cylinderInfo, sphereInfo, ...
                                                                                                        calcuDis, searchRadius, step, iter, iterMax, findpath, c_best, startPoint, goalPoint, ...
                                                                                                        axisStart, axisLWH, rejection_prob_base, collisionCount)
        % 嵌套函数 growTree，扩展一棵树
        % 输入：
        %   isTreeA: boolean, true表示扩展A树，false表示扩展B树
        %   nodes, costs, parents, nodeCount, edges, edge_count: 当前树的变量 (传入修改)
        %   targetNodes, targetNodeCount: 目标树的变量 (只读)
        %   其他参数...
        % 输出：
        %   success: boolean, 是否成功扩展
        %   nodes, costs, parents, nodeCount, edges, edge_count: 更新后的当前树变量
        %   collisionCount: 更新后的碰撞计数

        % --- 自适应目标偏置采样 ---
        % samplePoint_adaptive 函数只需要一些参数来决定采样点，不需要修改树结构
        randPoint = samplePoint_adaptive(axisStart, axisLWH, findpath, c_best, startPoint, goalPoint, targetNodes, targetNodeCount, isTreeA, iter, iterMax);
    
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
    
        % 碰撞检测 (检查从 nearestNode 到 newNode 的线段)
    if isCubeCollision(cubeInfo, nearestNode, newNode, step) || ...
       isCylinderCollision(cylinderInfo, nearestNode, newNode, step) || ...
       isSphereCollision(sphereInfo, nearestNode, newNode, step)
            collisionCount = collisionCount + 1; % 增加碰撞计数
            success = false;
            return;
        end

        % --- 动态概率节点拒绝策略 ---
        dist_to_nearest_obstacle = calculateMinDistToObstacle(newNode, cubeInfo, cylinderInfo, sphereInfo);
        % 拒绝概率随距离障碍物越近而增加
        rejection_prob = rejection_prob_base + (1 - rejection_prob_base) * exp(-dist_to_nearest_obstacle / 0.5); % 0.5是影响因子，可调

        if rand() < rejection_prob
            % 拒绝该节点
            % fprintf('拒绝节点 (距离障碍物 %.3f): %.3f\n', dist_to_nearest_obstacle, rejection_prob);
            success = false;
            return;
        end
        % -----------------------------

    
    % 找到搜索半径内的邻近节点
    nearNodes = [];
    for i = 1:nodeCount
        if calcuDis(nodes(i,:), newNode) <= searchRadius
            nearNodes = [nearNodes, i];
        end
    end
    
        % 添加新节点 (只有在未被拒绝时才执行)
        nodeCount = nodeCount + 1;
        % 检查数组大小是否足够
        if nodeCount > size(nodes, 1)
             fprintf('Warning: nodes array size exceeded. Consider increasing iterMax.\n');
             success = false; % 节点数量达到上限，扩展失败
             return;
        end

        nodes(nodeCount,:) = newNode;

        % 选择最优父节点 (基于costs)
    minCost = inf;
    minIdx = nearestIdx;
    
    for i = 1:length(nearNodes)
        idx = nearNodes(i);
            % 计算从根节点到当前节点再到newNode的潜在代价
        potentialCost = costs(idx) + calcuDis(nodes(idx,:), newNode);
        
            % 检查连接是否有效 (这里的碰撞检查在前面已经做过，但RRT*在这里也会检查)
        if ~isCubeCollision(cubeInfo, nodes(idx,:), newNode, step) && ...
           ~isCylinderCollision(cylinderInfo, nodes(idx,:), newNode, step) && ...
           ~isSphereCollision(sphereInfo, nodes(idx,:), newNode, step)
            
                 % Informed RRT* 阶段：只考虑在椭球内的连接 (如果在findpath=1之后)
                 if findpath == 1
                     % 需要计算newNode与起止点组成的椭球关系
                     c_min = norm(startPoint - goalPoint);
                     x_center = (startPoint + goalPoint) / 2;
                     a_ellipsoid = c_best / 2;
                     b_ellipsoid = sqrt(c_best^2 - c_min^2) / 2;

                      % 计算旋转矩阵 R (从标准坐标系到椭球主轴对齐坐标系)
                      L = goalPoint - startPoint;
                      e1 = L / norm(L);
                      e2 = [1, 0, 0]; if abs(dot(e1, e2)) > 0.9; e2 = [0, 1, 0]; end
                      e2 = e2 - dot(e2, e1) * e1; e2 = e2 / norm(e2);
                      e3 = cross(e1, e2); R_transform = [e1', e2', e3'];

                     % 检查 newNode 是否在椭球内
                     relative_newNode = (R_transform' * (newNode - x_center)')';
                     isInEllipsoid = ((relative_newNode(1)/a_ellipsoid)^2 + (relative_newNode(2)/b_ellipsoid)^2 + (relative_newNode(3)/b_ellipsoid)^2 <= 1.0);

                     % 只有当连接在椭球内时才考虑
                     if isInEllipsoid
                        if potentialCost < minCost
                            minCost = potentialCost;
                            minIdx = idx;
                        end
                     end
                 else % 未找到路径阶段，直接选择最优父节点
            if potentialCost < minCost
                minCost = potentialCost;
                minIdx = idx;
                     end
            end
        end
    end
    
        % 连接到最优父节点
    parents(nodeCount) = minIdx;
        costs(nodeCount) = minCost; % 从根节点到newNode的代价
    
        % 存储边信息 (基于parents[nodeCount]和nodeCount)
    edge_count = edge_count + 1;
        if edge_count > size(edges, 1); fprintf('Warning: edges array size exceeded.\n'); end
        edges(edge_count,:) = [nodes(minIdx,:), newNode]; % 存储父节点到新节点的边

    
        % --- 重布线(Rewiring) (保留RRT*特性，如果需要"非优化"则移除此段) ---
    for i = 1:length(nearNodes)
        idx = nearNodes(i);
            if idx == minIdx % 跳过最优父节点
            continue;
        end
        
            % 计算通过newNode连接到idx的潜在代价
        potentialCost = costs(nodeCount) + calcuDis(newNode, nodes(idx,:));
        
            % 检查是否能提供更优的路径 (从根节点到idx)
        if potentialCost < costs(idx)
                % 检查连接是否有效 (newNode 到 nodes(idx,:))
            if ~isCubeCollision(cubeInfo, newNode, nodes(idx,:), step) && ...
               ~isCylinderCollision(cylinderInfo, newNode, nodes(idx,:), step) && ...
               ~isSphereCollision(sphereInfo, newNode, nodes(idx,:), step)
                
                     % Informed RRT* 阶段：只考虑在椭球内的重布线
                     if findpath == 1
                         % 需要计算 nodes(idx,:) 与起止点组成的椭球关系
                         c_min = norm(startPoint - goalPoint);
                         x_center = (startPoint + goalPoint) / 2;
                         a_ellipsoid = c_best / 2;
                         b_ellipsoid = sqrt(c_best^2 - c_min^2) / 2;

                          L = goalPoint - startPoint;
                          e1 = L / norm(L);
                          e2 = [1, 0, 0]; if abs(dot(e1, e2)) > 0.9; e2 = [0, 1, 0]; end
                          e2 = e2 - dot(e2, e1) * e1; e2 = e2 / norm(e2);
                          e3 = cross(e1, e2); R_transform = [e1', e2', e3'];

                         relative_idxNode = (R_transform' * (nodes(idx,:) - x_center)')';
                         isInEllipsoid = ((relative_idxNode(1)/a_ellipsoid)^2 + (relative_idxNode(2)/b_ellipsoid)^2 + (relative_idxNode(3)/b_ellipsoid)^2 <= 1.0);

                         % 只有当连接在椭球内时才进行重布线
                         if isInEllipsoid
                            parents(idx) = nodeCount; % 更改父节点为newNode
                            costs(idx) = potentialCost; % 更新代价

                            % 更新边信息 (移除旧边，添加新边)
                             % 移除旧边逻辑比较复杂，通常简单地添加新边，可视化时处理
                             edge_count = edge_count + 1;
                             if edge_count > size(edges, 1); fprintf('Warning: edges array size exceeded.\n'); end
                             edges(edge_count,:) = [newNode, nodes(idx,:)];
                         end
                     else % 未找到路径阶段，直接重布线
                         parents(idx) = nodeCount; % 更改父节点为newNode
                         costs(idx) = potentialCost; % 更新代价
                
                % 更新边信息
                edge_count = edge_count + 1;
                         if edge_count > size(edges, 1); fprintf('Warning: edges array size exceeded.\n'); end
                edges(edge_count,:) = [newNode, nodes(idx,:)];
                     end
                 end
            end
        end
        % --- 重布线结束 ---

        success = true; % 扩展成功
    end

%% 嵌套函数：尝试连接两棵树 (使用贪婪连接思想)
    function [connected, connectIdxA, connectIdxB, c_best, findpath, no_improve_count, edgesA, edge_countA, edgesB, edge_countB] = connectTrees(fromTreeA, sourceNodes, sourceNodeCount, sourceCosts, ...
                                                                                                                                              targetNodes, targetNodeCount, targetCosts, ...
                                                                                                                                              cubeInfo, cylinderInfo, sphereInfo, calcuDis, Thr, step, ...
                                                                                                                                              connectIdxA, connectIdxB, findpath, c_best, no_improve_count, improvement_threshold, max_no_improve, ...
                                                                                                                                              edgesA, edge_countA, edgesB, edge_countB, startPoint, goalPoint)
        % 嵌套函数 connectTrees，尝试连接两棵树
        % 输入：
        %   fromTreeA: boolean, true表示从A树连接到B树，false表示从B树连接到A树
        %   sourceNodes, sourceNodeCount, sourceCosts: 源树的变量 (传入修改，但不在这儿修改)
        %   targetNodes, targetNodeCount, targetCosts: 目标树的变量 (只读)
        %   其他参数...
        % 输出：
        %   connected: boolean, 是否连接成功并找到更好的路径
        %   connectIdxA, connectIdxB: 更新后的连接点索引
        %   c_best, findpath, no_improve_count: 更新后的Informed相关变量
        %   edgesA, edge_countA, edgesB, edge_countB: 更新后的边信息

        % --- 初始化输出变量，确保在所有情况下都有值 ---
        connected = false; % 默认未连接成功或未找到更好路径
        % 其他输出变量保持其输入值，除非在函数内部被更新
        % connectIdxA, connectIdxB, c_best, findpath, no_improve_count, edgesA, edge_countA, edgesB, edge_countB

        % 获取源树的新节点 (即最近扩展的节点)
        newNode = sourceNodes(sourceNodeCount,:);

        % 在目标树中找最近点
        minDist = inf;
        nearestIdx_target = 0; % 在目标树中的索引
        for i = 1:targetNodeCount
            dist = calcuDis(targetNodes(i,:), newNode);
            if dist < minDist
                minDist = dist;
                nearestIdx_target = i;
            end
        end

        % 检查是否可以直接连接 (贪婪连接)
        if minDist < Thr % 如果距离小于阈值，尝试连接
            % 检查连接是否有碰撞 (从 newNode 到目标树的最近点 targetNodes(nearestIdx_target,:))
            if ~isCubeCollision(cubeInfo, newNode, targetNodes(nearestIdx_target,:), step) && ...
               ~isCylinderCollision(cylinderInfo, newNode, targetNodes(nearestIdx_target,:), step) && ...
               ~isSphereCollision(sphereInfo, newNode, targetNodes(nearestIdx_target,:), step)

                % 连接成功！计算路径总代价
                % 总代价 = 从源树根节点到 newNode 的代价 + newNode 到 targetNodes(nearestIdx_target,:) 的距离 + 从 targetNodes(nearestIdx_target,:) 到目标树根节点的代价
                current_path_cost = sourceCosts(sourceNodeCount) + calcuDis(newNode, targetNodes(nearestIdx_target,:)) + targetCosts(nearestIdx_target);

                % --- Informed RRT* 优化逻辑 ---
                if current_path_cost < c_best
                    % 检查改善程度
                    if findpath == 0 % 第一次找到路径
                         fprintf('第一次找到路径，代价: %.3f\n', current_path_cost);
                         c_best = current_path_cost; % 更新最优代价
                         findpath = 1; % 设置找到路径标志
                         no_improve_count = 0; % 重置未改善计数
                         % 可以根据第一次找到的路径长度动态调整max_no_improve和improvement_threshold
                         % 例如：max_no_improve = min(30, max(10, round(c_best / (norm(startPoint-goalPoint) * step))));
                         % improvement_threshold = max(0.01, c_best * 0.01);
                         fprintf('启用Informed阶段，设置初始c_best: %.3f\n', c_best);

                    else % 不是第一次找到路径，检查是否是更好的路径
                        improvement = c_best - current_path_cost;
                         if improvement > improvement_threshold % 如果改善大于阈值
                             fprintf('找到更好路径，代价从 %.3f 降低到 %.3f\n', c_best, current_path_cost);
                             c_best = current_path_cost; % 更新最优代价
                             no_improve_count = 0; % 重置未改善计数
                             % 可以在这里动态调整 improvement_threshold 和 max_no_improve
                             % 例如：if improvement > improvement_threshold * 2; improvement_threshold = improvement_threshold * 0.8; max_no_improve = max_no_improve + 2; end
                             % 例如：if improvement < improvement_threshold * 0.5; improvement_threshold = improvement_threshold * 1.2; max_no_improve = max(max_no_improve - 1, 5); end
                             % fprintf('动态调整阈值和次数，当前阈值: %.3f，未改善次数: %d\n', improvement_threshold, max_no_improve);

                         else % 找到路径但改善不显著
                             no_improve_count = no_improve_count + 1;
                             fprintf('找到路径但未找到更好的路径，当前未改善次数: %d\n', no_improve_count);
                             fprintf('当前路径代价: %.3f，最优路径代价: %.3f\n', current_path_cost, c_best);
                             
                             % 在这里添加检查
                             if no_improve_count >= max_no_improve
                                 fprintf('连续%d次未找到更好的路径，停止优化\n', max_no_improve);
                                 connected = true; % 设置为true以触发主循环中的break
                                 return;
                             end
                         end
                    end

                     % 记录连接点索引 (用于构建最终路径)
                    if fromTreeA
                        connectIdxA = sourceNodeCount; % sourceNodes(sourceNodeCount,:) 是 newNode
                        connectIdxB = nearestIdx_target; % targetNodes(nearestIdx_target,:) 是目标树的连接点

                        % 存储连接边
                        edge_countA = edge_countA + 1; % 使用输入参数 edge_countA 并修改
                        if edge_countA > size(edgesA, 1); fprintf('Warning: edgesA array size exceeded.\n'); end
                        edgesA(edge_countA,:) = [newNode, targetNodes(nearestIdx_target,:)];
                    else
                        connectIdxB = sourceNodeCount;
                        connectIdxA = nearestIdx_target;

                        % 存储连接边
                        edge_countB = edge_countB + 1; % 使用输入参数 edge_countB 并修改
                        if edge_countB > size(edgesB, 1); fprintf('Warning: edgesB array size exceeded.\n'); end
                        edgesB(edge_countB,:) = [newNode, targetNodes(nearestIdx_target,:)];
                    end

                    % 如果找到更好的路径，设置 connected = true
                    % 如果只是连接成功但没找到更好路径，connected 保持 false
                     connected = true; % <--- 只有在找到更好路径时才设置为 true

                else
                    % 找到路径但代价没有更好
                    if findpath == 1 % 如果已经在Informed阶段
                         no_improve_count = no_improve_count + 1;
                         fprintf('找到路径但未找到更好的路径，当前未改善次数: %d\n', no_improve_count);
                         fprintf('当前路径代价: %.3f，最优路径代价: %.3f\n', current_path_cost, c_best);
                         
                         % 在这里添加检查
                         if no_improve_count >= max_no_improve
                             fprintf('连续%d次未找到更好的路径，停止优化\n', max_no_improve);
                             connected = true; % 设置为true以触发主循环中的break
                             return;
                         end
                    end
                    % 即使代价没有更好，连接仍然是成功的，但connected=false，主循环会继续尝试扩展另一棵树
                    connected = false; % <--- 保持 false，表示未找到更好路径
                end
                % --- Informed RRT* 优化逻辑结束 ---

                 % 如果连接成功（无论是否找到更好路径），主循环都应继续尝试扩展另一棵树
                 % 因此这里不使用 return，让函数正常执行到末尾
            end
        end
        % 如果距离大于阈值或有碰撞，connected 保持默认值 false
    end % 函数结束

%% 新增嵌套函数：自适应采样策略
    function point = samplePoint_adaptive(axisStart, axisLWH, findpath, c_best, startPoint, goalPoint, targetNodes, targetNodeCount, isTreeA, iter, iterMax)
        % 自适应采样策略
        % 输入：
        %   axisStart, axisLWH: 工作空间边界
        %   findpath: 是否已找到路径
        %   c_best: 当前最优路径代价
        %   startPoint, goalPoint: 整个问题的起止点
        %   targetNodes, targetNodeCount: 目标树的节点信息
        %   isTreeA: 是否是起点树A (用于确定当前树的目标偏置方向)
        %   iter, iterMax: 当前迭代次数和最大迭代次数 (用于动态调整偏置)

        c_min = norm(startPoint - goalPoint); % 整个问题的起止点距离

        % --- 自适应目标偏置概率计算 ---
        % 在未找到初始路径阶段，偏向目标树采样概率随迭代次数增加而降低
        % 在已找到路径（Informed）阶段，偏向终点采样概率根据路径质量调整
        
        if findpath == 0 % 未找到初始路径阶段 (双向搜索快速探索)
            % 偏向目标树的概率，随迭代次数增加而降低，鼓励后期随机探索
             bias_to_target_tree_prob = 0.6 * (1 - iter/iterMax) + 0.1; % 从0.7下降到0.1
            
            if rand() < bias_to_target_tree_prob && targetNodeCount > 0 % 确保目标树非空
                 % 偏向目标树的节点采样
                 randIdx = randi(targetNodeCount);
                 point = targetNodes(randIdx,:);
                 return; % 返回偏向采样的点
            end
        else % 已找到初始路径 (Informed 阶段)
             % 在Informed阶段，主要在椭球内采样。这里可以根据路径质量微调偏向终点采样的概率
             ratio = c_best / c_min;
             % 偏向终点采样的概率，路径质量越好概率越高
             bias_to_goal_prob_informed = 0.1 + 0.2 * max(0, (2.0 - ratio)/1.0); % 路径质量从好到差，概率从0.3下降到0.1

             if rand() < bias_to_goal_prob_informed
                 % 直接在终点附近进行小范围采样
                 point = goalPoint + randn(1,3) * 0.05; % 在终点附近正态分布采样
                 % 确保采样点在工作空间边界内 (Informed 阶段主要在椭球内，但终点可能在边界附近)
                 point = max(point, axisStart);
                 point = min(point, axisStart + axisLWH);
                 return; % 返回偏向终点的采样点
             end

             % --- Informed RRT* 椭球采样 ---
             % 计算椭球参数 (基于整个问题的startPoint和goalPoint)
        x_center = (startPoint + goalPoint) / 2;
             a_ellipsoid = c_best / 2;
             % 确保 b_ellipsoid 是实数，避免 c_best == c_min 时 sqrt(0)
             b_ellipsoid_sq = max(0, c_best^2 - c_min^2) / 4; % 半短轴平方，之前计算有误
             b_ellipsoid = sqrt(b_ellipsoid_sq);

             % 如果椭球退化成线 (c_best ≈ c_min)，进行直线上的采样加扰动
             if b_ellipsoid < 1e-3 * a_ellipsoid % 使用一个阈值判断是否退化
                 t = rand(); % 在0到1之间采样
                 point = startPoint + t * (goalPoint - startPoint) + randn(1,3) * 0.01; % 在直线附近加小扰动
                  % 确保采样点在工作空间边界内
                 point = max(point, axisStart);
                 point = min(point, axisStart + axisLWH);
                 return;
             end


             % 计算旋转矩阵 R
        L = goalPoint - startPoint;
        e1 = L / norm(L);
              e2 = [1, 0, 0]; if abs(dot(e1, e2)) > 0.9; e2 = [0, 1, 0]; end
              e2 = e2 - dot(e2, e1) * e1; e2 = e2 / norm(e2);
              e3 = cross(e1, e2); R_transform = [e1', e2', e3'];

             % 在椭球内采样 (使用接受-拒绝采样)
             max_attempts = 200; % 增加尝试次数
             for attempt = 1:max_attempts
                 % 在包围椭球的AABB（轴对齐包围盒）内均匀采样
                 % AABB的范围根据椭球参数和中心计算
                 % 简化处理：直接在更大的空间范围内采样，然后检查是否在椭球内
                 % 更优方法：在标准化的椭球内采样，然后变换
                 
                 % 在标准球内采样后缩放并变换是更标准的做法
                 % 在单位球内采样
                 r_unit = rand(); % 在 [0, 1] 内均匀分布
                 theta_unit = 2 * pi * rand();
                 phi_unit = acos(2 * rand() - 1); % [0, pi] (球面坐标)

                 % 在椭球主轴对齐的坐标系下生成点
                 x_prime = r_unit * sin(phi_unit) * cos(theta_unit);
                 y_prime = r_unit * sin(phi_unit) * sin(theta_unit);
                 z_prime = r_unit * cos(phi_unit);

                 % 应用椭球的半轴长度进行缩放
                 point_scaled = [a_ellipsoid * x_prime, b_ellipsoid * y_prime, b_ellipsoid * z_prime];

                 % 旋转并平移回原坐标系
                 sampled_point = (R_transform * point_scaled')' + x_center;

                 % 检查是否在边界内 (在 Informed 阶段采样主要在椭球内，可能超出边界)
                 if all(sampled_point >= axisStart) && all(sampled_point <= (axisStart + axisLWH))
                     point = sampled_point;
                     return; % 成功采样，返回点
                 end
             end

             % 如果椭球采样多次失败，回退到在边界内随机采样
             % 提高回退概率，避免陷入局部
             % if rand() < 0.1 % 从0.05提高到0.1
                 point = axisStart + rand(1,3) .* axisLWH;
                  point = max(point, axisStart); % 确保在边界内
                  point = min(point, axisStart + axisLWH); % 确保在边界内
                 % fprintf('椭球采样失败多次，回退到边界内随机采样\n');
                 return;
             % end
        end

        % --- 如果没有触发上述偏置采样，进行常规随机采样 ---
        % 在整个工作空间内随机采样
        point = axisStart + rand(1,3) .* axisLWH;

        % 确保在边界内 (虽然上面的随机采样应该在边界内，但多一步检查更安全)
         point = max(point, axisStart);
         point = min(point, axisStart + axisLWH);
    end

%% 新增辅助函数：计算点到最近障碍物的距离
    function minDist = calculateMinDistToObstacle(point, cubeInfo, cylinderInfo, sphereInfo)
        % 计算点到最近障碍物的距离
        minDist = inf;

        % 检查立方体障碍物
        if ~isempty(cubeInfo) && size(cubeInfo, 1) > 0
            for i = 1:size(cubeInfo, 1)
                if size(cubeInfo, 2) >= 6
                    cube_center = cubeInfo(i, 1:3);
                    cube_size = cubeInfo(i, 4:6);
                    
                    % 简单的点到中心距离作为参考（不精确，但快速）
                    dist = norm(point - cube_center) - max(cube_size)/2; % 减去半对角线近似
                    % 更精确的点到AABB距离计算会更复杂
                    minDist = min(minDist, max(0, dist)); % 距离不能为负
                end
            end
        end
        
        % 检查圆柱体障碍物
        if ~isempty(cylinderInfo) && size(cylinderInfo, 1) > 0
             for i = 1:size(cylinderInfo, 1)
                 if size(cylinderInfo, 2) >= 5
                     cylinder_center = cylinderInfo(i, 1:3);
                     cylinder_radius = cylinderInfo(i, 4);
                     cylinder_height = cylinderInfo(i, 5);

                      % 计算点到圆柱体轴线的水平距离
                      dist_horizontal = norm(point(1:2) - cylinder_center(1:2));
                      % 计算点到圆柱体顶/底面的垂直距离
                      dist_vertical_top = point(3) - (cylinder_center(3) + cylinder_height/2);
                      dist_vertical_bottom = (cylinder_center(3) - cylinder_height/2) - point(3);

                      % 计算点到圆柱体表面的距离
                      if dist_horizontal <= cylinder_radius && point(3) >= cylinder_center(3) - cylinder_height/2 && point(3) <= cylinder_center(3) + cylinder_height/2
                          % 点在圆柱体水平范围内且在高度范围内
                          dist = 0; % 点在障碍物内部或表面，距离为0
                      elseif dist_horizontal <= cylinder_radius % 在圆柱体水平范围内但不在高度范围内
                          dist = max(dist_vertical_top, dist_vertical_bottom);
                      elseif point(3) >= cylinder_center(3) - cylinder_height/2 && point(3) <= cylinder_center(3) + cylinder_height/2 % 在高度范围内但不在水平范围内
                          dist = dist_horizontal - cylinder_radius;
                      else % 都不在范围内
                          % 点到圆柱体边缘的距离，复杂，这里简化为到最近圆柱体端面圆周的距离近似
                          dist_to_cap = sqrt((dist_horizontal - cylinder_radius)^2 + max(0, dist_vertical_top, dist_vertical_bottom)^2);
                           dist = dist_to_cap;
                      end

                     minDist = min(minDist, max(0, dist)); % 确保距离非负
                 end
             end
        end

        % 检查球体障碍物
        if ~isempty(sphereInfo) && size(sphereInfo, 1) > 0
            for i = 1:size(sphereInfo, 1)
                if size(sphereInfo, 2) >= 4
                    sphere_center = sphereInfo(i, 1:3);
                    sphere_radius = sphereInfo(i, 4);
                    
                    % 计算点到球体表面的距离
                    dist = norm(point - sphere_center) - sphere_radius;
                    minDist = min(minDist, max(0, dist)); % 距离不能为负
                end
            end
        end

         % 如果没有障碍物，距离无穷大
         if minDist == inf
             minDist = 1000; % 使用一个较大的值表示距离很远
         end
    end


% --- 你需要在这里添加你的碰撞检测函数：isCubeCollision, isCylinderCollision, isSphereCollision ---
% 确保这些函数能够正确地检查从 p1 到 p2 的线段是否与对应的障碍物碰撞。
% 如果你的碰撞检测函数只需要检查点是否在障碍物内，可能需要修改或补充线段碰撞检测。
% 示例函数签名 (请替换为你自己的实现):
% function isCollision = isCubeCollision(cubeInfo, p1, p2, step)
%     % 实现检查线段p1-p2是否与cubeInfo中的立方体碰撞的逻辑
% end
% function isCollision = isCylinderCollision(cylinderInfo, p1, p2, step)
%     % 实现检查线段p1-p2是否与cylinderInfo中的圆柱体碰撞的逻辑
% end
% function isCollision = isSphereCollision(sphereInfo, p1, p2, step)
%     % 实现检查线段p1-p2是否与sphereInfo中的球体碰撞的逻辑
% end