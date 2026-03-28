function Path = TA_FF_RRTStar(startPoint, axisStart, axisLWH, goalPoint, cubeInfo, cylinderInfo, sphereInfo)
%% TA-FF-RRT* (Type-Aware Failure-Feedback RRT*)
% 改进版：在基础 RRT* 上加入失败类型感知 + 多核排斥反馈
% 适合四旋翼纯几何路径规划

rng('shuffle');
totalTime = tic;

%% ==================== 参数设置 ====================
iterMax = 15000;          
step = 0.05;              
searchRadius = 0.15;      
Thr = 0.1;                
goalBias = 0.08;          

% 失败反馈参数
feedbackPeriod = 300;     
failureRateThreshold = 0.65; 
maxFailureRecords = 3000; 
beta = 0.72;              
decayRate = 0.95;         
decayPeriod = 2000;

thetaMax = 120 * pi/180;   % 尖角阈值
lMin = 0.02;               % 最小段长度

%% ==================== 初始化 ====================
nodes = zeros(iterMax, 3);
nodes(1,:) = startPoint;
costs = zeros(iterMax, 1); costs(1) = 0;
parents = zeros(iterMax, 1);
nodeCount = 1;

edges = zeros(iterMax*2, 6);
edge_count = 0;

% 失败记录: [x y z type iter weight]
failureRecords = zeros(maxFailureRecords, 6);
failureCount = 0;

collisionCount = 0;
sharpCount = 0;
fragmentCount = 0;

lastFeedbackIter = 0;
failurePositions = [];   % 用于简化排斥场计算

planningTimeStart = tic;
fprintf('TA-FF-RRT* 开始规划...\n');

iter = 0;
consecutiveFailures = 0;

%% ==================== 主循环 ====================
while iter < iterMax
    iter = iter + 1;
    
    if mod(iter, 500) == 0
        fprintf('迭代: %d | 节点: %d | 碰撞失败率: %.1f%%\n', ...
            iter, nodeCount, collisionCount*100/max(iter,1));
    end
    
    % === 失败反馈更新（批处理）===
    currentFailureRate = collisionCount / max(iter, 1);
    if mod(iter, feedbackPeriod) == 0 || ...
       (currentFailureRate > failureRateThreshold && iter - lastFeedbackIter > 50) || ...
       (iter - lastFeedbackIter > decayPeriod)
        
        failurePositions = failureRecords(1:failureCount, 1:3);  % 更新失败位置列表
        lastFeedbackIter = iter;
        
        % 历史衰减
        if mod(iter, decayPeriod) == 0 && failureCount > 0
            failureRecords(1:failureCount, 6) = failureRecords(1:failureCount, 6) * decayRate;
        end
    end
    
    % === 采样（带排斥调制 + 目标偏置）===
    if rand < goalBias + 0.02*consecutiveFailures  % 连续失败时增加目标偏置
        randPoint = goalPoint;
    else
        randPoint = axisStart + rand(1,3) .* axisLWH;
        
        % 简单排斥调制：尝试几次采样，避开高排斥区
        if failureCount > 20
            for attempt = 1:6
                candidate = axisStart + rand(1,3) .* axisLWH;
                if getRepulsion(candidate, failurePositions) < 0.82
                    randPoint = candidate;
                    break;
                end
            end
        end
    end
    
    % === 最近节点 ===
    dists = vecnorm(nodes(1:nodeCount,:) - randPoint, 2, 2);
    [~, nearestIdx] = min(dists);
    nearestNode = nodes(nearestIdx,:);
    
    % === 扩展 ===
    direction = randPoint - nearestNode;
    normDir = norm(direction);
    if normDir < 1e-8, continue; end
    direction = direction / normDir;
    newNode = nearestNode + step * direction;
    
    % === 碰撞检测 ===
    if isCubeCollision(cubeInfo, nearestNode, newNode, step) || ...
       isCylinderCollision(cylinderInfo, nearestNode, newNode, step) || ...
       isSphereCollision(sphereInfo, nearestNode, newNode, step)
        
        collisionCount = collisionCount + 1;
        recordFailure(newNode, 1, iter);
        consecutiveFailures = consecutiveFailures + 1;
        continue;
    end
    
    % === 其他失败类型检查 ===
    edgeVec = newNode - nearestNode;
    if nearestIdx > 1
        prevEdge = nearestNode - nodes(parents(nearestIdx),:);
        if norm(prevEdge) > 1e-6
            angle = acos( max(-1, min(1, dot(prevEdge, edgeVec)/(norm(prevEdge)*norm(edgeVec))) ));
            if angle > thetaMax
                sharpCount = sharpCount + 1;
                recordFailure(newNode, 2, iter);
            end
        end
    end
    if norm(edgeVec) < lMin
        fragmentCount = fragmentCount + 1;
        recordFailure(newNode, 3, iter);
    end
    
    % === 近邻 + 最佳父节点 + Rewire（保留原逻辑）===
    nearIdx = find(vecnorm(nodes(1:nodeCount,:) - newNode, 2, 2) <= searchRadius);
    
    minCost = costs(nearestIdx) + norm(newNode - nearestNode);
    bestParent = nearestIdx;
    
    for k = 1:length(nearIdx)
        idx = nearIdx(k);
        if idx == nearestIdx, continue; end
        potCost = costs(idx) + norm(nodes(idx,:) - newNode);
        if potCost < minCost && ...
           ~isCubeCollision(cubeInfo, nodes(idx,:), newNode, step) && ...
           ~isCylinderCollision(cylinderInfo, nodes(idx,:), newNode, step) && ...
           ~isSphereCollision(sphereInfo, nodes(idx,:), newNode, step)
            minCost = potCost;
            bestParent = idx;
        end
    end
    
    % 添加新节点
    nodeCount = nodeCount + 1;
    nodes(nodeCount,:) = newNode;
    parents(nodeCount) = bestParent;
    costs(nodeCount) = minCost;
    
    edge_count = edge_count + 1;
    edges(edge_count,:) = [nodes(bestParent,:), newNode];
    
    % Rewire
    for k = 1:length(nearIdx)
        idx = nearIdx(k);
        if idx == bestParent, continue; end
        potCost = costs(nodeCount) + norm(newNode - nodes(idx,:));
        if potCost < costs(idx) && ...
           ~isCubeCollision(cubeInfo, newNode, nodes(idx,:), step) && ...
           ~isCylinderCollision(cylinderInfo, newNode, nodes(idx,:), step) && ...
           ~isSphereCollision(sphereInfo, newNode, nodes(idx,:), step)
            parents(idx) = nodeCount;
            costs(idx) = potCost;
            edge_count = edge_count + 1;
            edges(edge_count,:) = [newNode, nodes(idx,:)];
        end
    end
    
    consecutiveFailures = 0;
    
    % === 到达目标检查 ===
    if norm(newNode - goalPoint) < Thr
        if ~isCubeCollision(cubeInfo, newNode, goalPoint, step) && ...
           ~isCylinderCollision(cylinderInfo, newNode, goalPoint, step) && ...
           ~isSphereCollision(sphereInfo, newNode, goalPoint, step)
            
            nodeCount = nodeCount + 1;
            nodes(nodeCount,:) = goalPoint;
            parents(nodeCount) = nodeCount - 1;
            costs(nodeCount) = costs(nodeCount-1) + norm(newNode - goalPoint);
            edge_count = edge_count + 1;
            edges(edge_count,:) = [newNode, goalPoint];
            break;
        end
    end
end

planningTime = toc(planningTimeStart);

%% ==================== 路径提取 ====================
if nodeCount <= 1 || iter >= iterMax
    Path = [];
    disp('TA-FF-RRT* 规划失败，未找到路径');
    return;
end

% 回溯
path = zeros(nodeCount, 3);
idx = 1;
current = nodeCount;
while current >= 1
    path(idx,:) = nodes(current,:);
    current = parents(current);
    idx = idx + 1;
end
path = path(1:idx-1, :);
Path.path = flipud(path);

Path.edges = edges(1:min(edge_count, size(edges,1)), :);
Path.iter = iter;
Path.nodeCount = nodeCount;
Path.collisionCount = collisionCount;
Path.sharpCount = sharpCount;
Path.fragmentCount = fragmentCount;
Path.planningTime = planningTime;
Path.totalTime = toc(totalTime);

fprintf('TA-FF-RRT* 规划成功！\n迭代: %d | 节点: %d | 碰撞:%d | 尖角:%d | 碎片:%d\n', ...
    iter, nodeCount, collisionCount, sharpCount, fragmentCount);
fprintf('规划时间: %.3f秒 | 路径长度: %.2f m\n', planningTime, sum(vecnorm(diff(Path.path),2,2)));

%% ==================== 嵌套辅助函数 ====================
function r = getRepulsion(point, failPos)
    if isempty(failPos) || size(failPos,1)==0
        r = 0; return;
    end
    dists = vecnorm(failPos - point, 2, 2);
    r = mean(exp(-dists.^2 / (2*0.12^2)));  % sigma ≈ 0.12，可调
    r = min(r * beta, 1.0);
end

function recordFailure(pos, fType, currIter)
    failureCount = failureCount + 1;  % 注意：这里用 outer scope 的变量（嵌套函数可直接访问）
    if failureCount > maxFailureRecords
        failureCount = maxFailureRecords;
        failureRecords = circshift(failureRecords, -1, 1);
    end
    failureRecords(failureCount, :) = [pos(1), pos(2), pos(3), fType, currIter, 1.0];
    
    if fType == 1
        collisionCount = collisionCount + 1;
    elseif fType == 2
        sharpCount = sharpCount + 1;
    elseif fType == 3
        fragmentCount = fragmentCount + 1;
    end
end

end   % 主函数结束