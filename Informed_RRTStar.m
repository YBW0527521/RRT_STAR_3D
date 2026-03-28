function Path = Informed_RRTStar(startPoint, axisStart, axisLWH, goalPoint, cubeInfo, cylinderInfo, sphereInfo)
%% Informed RRT*算法实现
rng('shuffle');  % 使用系统时间作为随机种子
totalTime = tic;  % 开始计时

%% 参数设置
calcuDis = @(x,y) sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2); 
iterMax = 10000;   % 最大迭代次数
iter = 0;         % 当前迭代次数
searchRadius = 0.15; % 搜索半径
step = 0.05;      % 基础步长
Thr = 0.1;        % 阈值

% 初始化路径代价
c_best = inf;     % 最优路径代价
findpath = 0;     % 是否找到路径的标志
no_improve_count = 0;  % 连续未改善次数
max_no_improve = 15;  % 最大连续未改善次数
improvement_threshold = 0.05;  % 改善阈值

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
    
    % 使用改进的采样策略
    randPoint = samplePoint_improve(axisStart, axisLWH, findpath, c_best, startPoint, goalPoint, cubeInfo, cylinderInfo, sphereInfo);
    
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
            
            % 更新路径代价
            current_cost = costs(nodeCount);
            if current_cost < c_best
                % 检查改善程度
                if (c_best - current_cost) > improvement_threshold
                    no_improve_count = 0;  % 重置未改善计数
                end
                c_best = current_cost;
                findpath = 1;  % 找到路径后启用椭球采样
                fprintf('找到路径，启用椭球采样，当前路径代价: %.3f\n', c_best);
            else
                no_improve_count = no_improve_count + 1;
            end
            
            % 存储最后一条边
            edge_count = edge_count + 1;
            edges(edge_count,:) = [newNode, goalPoint];
            
            % 检查是否应该停止优化
            if no_improve_count >= max_no_improve
                fprintf('连续%d次未找到更好的路径，停止优化\n', max_no_improve);
                break;
            end
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

% 采样函数
function point = samplePoint_improve(axisStart, axisLWH, findpath, c_best, startPoint, goalPoint, cubeInfo, cylinderInfo, sphereInfo)
    % 计算起点到终点的距离
    start_goal_dist = norm(goalPoint - startPoint);
    c_min = start_goal_dist;

    if findpath == 1
        % 计算椭球参数
        x_center = (startPoint + goalPoint) / 2;
        a_ellipsoid = c_best / 2;
        b_ellipsoid = sqrt(c_best^2 - c_min^2) / 2;
        
        % 计算旋转矩阵
        L = goalPoint - startPoint;
        e1 = L / norm(L);
        e2 = [1, 0, 0];
        if abs(dot(e1, e2)) > 0.9
            e2 = [0, 1, 0];
        end
        e2 = e2 - dot(e2, e1) * e1;
        e2 = e2 / norm(e2);
        e3 = cross(e1, e2);
        R_transform = [e1', e2', e3'];

        % 根据路径质量动态调整目标点采样概率
        ratio = c_best / c_min;
        bias_to_goal_prob = 0.2 + 0.2 * max(0, (2.0 - ratio)/1.0);  % 降低目标点采样概率
        
        % 尝试采样
        max_attempts = 100;
        for attempt = 1:max_attempts
            if rand() < bias_to_goal_prob
                % 直接在目标点附近采样
                point = goalPoint + randn(1,3) * 0.15;  % 增加采样范围
            else
                % 在椭球内采样，使用改进的采样策略
                r_unit = rand()^(1/3);  % 使用立方根使采样更均匀
                theta_unit = 2 * pi * rand();
                phi_unit = acos(2 * rand() - 1);

                x_prime = r_unit * sin(phi_unit) * cos(theta_unit);
                y_prime = r_unit * sin(phi_unit) * sin(theta_unit);
                z_prime = r_unit * cos(phi_unit);

                % 根据椭球形状调整采样分布
                point_scaled = [a_ellipsoid * x_prime, b_ellipsoid * y_prime, b_ellipsoid * z_prime];
                
                % 添加随机扰动使分布更均匀
                random_offset = randn(1,3) * 0.05 * b_ellipsoid;
                point_scaled = point_scaled + random_offset;
                
                point = (R_transform * point_scaled')' + x_center;
            end

            % 检查是否在边界内
            if all(point >= axisStart) && all(point <= (axisStart + axisLWH))
                % 应用动态节点拒绝策略
                rejection_prob_base = 0.05;  % 降低基础拒绝概率
                dist_to_nearest_obstacle = calculateMinDistToObstacle(point, cubeInfo, cylinderInfo, sphereInfo);
                rejection_prob = rejection_prob_base + (1 - rejection_prob_base) * exp(-dist_to_nearest_obstacle / 0.3);  % 调整衰减系数

                if rand() >= rejection_prob
                    return;
                end
            end
        end

        % 如果多次尝试都失败，回退到边界内随机采样
        point = axisStart + rand(1,3) .* axisLWH;
        point = max(point, axisStart);
        point = min(point, axisStart + axisLWH);
        return;
    else
        % 未找到路径时的采样策略
        % 使用指数函数动态调整采样参数
        goal_bias = 0.7 + 0.2 * (1 - exp(-start_goal_dist));
        direct_goal_prob = 0.3 + 0.2 * (1 - exp(-start_goal_dist));
        r_power = 1.0 + 0.5 * (1 - exp(-start_goal_dist));

        if rand() < direct_goal_prob
            point = goalPoint;
        elseif rand() < goal_bias
            t = rand()^r_power;
            point = startPoint + t * (goalPoint - startPoint) + 0.1 * randn(1,3);
        else
            point = axisStart + rand(1,3) .* axisLWH;
        end
    end
end

% 计算点到障碍物的最小距离
function min_dist = calculateMinDistToObstacle(point, cubeInfo, cylinderInfo, sphereInfo)
    min_dist = inf;
    
    % 计算到立方体障碍物的最小距离
    if ~isempty(cubeInfo) && size(cubeInfo, 1) > 0 && size(cubeInfo, 2) >= 6
        for i = 1:size(cubeInfo, 1)
            try
                cube_center = cubeInfo(i, 1:3);
                cube_size = cubeInfo(i, 4:6);
                
                % 计算点到立方体表面的最小距离
                dx = max(0, abs(point(1) - cube_center(1)) - cube_size(1)/2);
                dy = max(0, abs(point(2) - cube_center(2)) - cube_size(2)/2);
                dz = max(0, abs(point(3) - cube_center(3)) - cube_size(3)/2);
                
                dist = sqrt(dx^2 + dy^2 + dz^2);
                min_dist = min(min_dist, dist);
            catch
                continue;
            end
        end
    end
    
    % 计算到圆柱体障碍物的最小距离
    if ~isempty(cylinderInfo) && size(cylinderInfo, 1) > 0 && size(cylinderInfo, 2) >= 8
        for i = 1:size(cylinderInfo, 1)
            try
                cyl_center = cylinderInfo(i, 1:3);
                cyl_radius = cylinderInfo(i, 4);
                cyl_height = cylinderInfo(i, 5);
                cyl_axis = cylinderInfo(i, 6:8);
                
                % 计算点到圆柱体轴线的投影
                v = point - cyl_center;
                proj = dot(v, cyl_axis) * cyl_axis;
                perp = v - proj;
                
                % 计算到圆柱体表面的最小距离
                dist_axis = norm(perp);
                dist_height = max(0, abs(dot(v, cyl_axis)) - cyl_height/2);
                
                if dist_height == 0
                    dist = abs(dist_axis - cyl_radius);
                else
                    dist = sqrt(dist_height^2 + max(0, dist_axis - cyl_radius)^2);
                end
                
                min_dist = min(min_dist, dist);
            catch
                continue;
            end
        end
    end
    
    % 计算到球体障碍物的最小距离
    if ~isempty(sphereInfo) && size(sphereInfo, 1) > 0 && size(sphereInfo, 2) >= 4
        for i = 1:size(sphereInfo, 1)
            try
                sphere_center = sphereInfo(i, 1:3);
                sphere_radius = sphereInfo(i, 4);
                
                dist = norm(point - sphere_center) - sphere_radius;
                min_dist = min(min_dist, dist);
            catch
                continue;
            end
        end
    end
end