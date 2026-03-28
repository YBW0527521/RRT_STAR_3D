function Path = RRT_APF(startPoint,axisStart,axisLWH,goalPoint,cubeInfo,cylinderInfo,sphereInfo)
%% RRT人工势场法算法寻找路径规划，适用于控制xArm6机械臂 - 只返回最优路径
rng('shuffle');  % 使用系统时间作为随机种子，确保每次运行结果不同
% 初始计时
totalTime = tic;

%% 参数设置
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2); 
iterMax = 10000;   % 最大迭代次数
iter = 0;   % 当前迭代次数
count = 1;  % 节点计数

step = 0.05;  % 基础步长
Thr = 0.1;    % 阈值

% 关闭图形显示以加快规划速度
set(0,'DefaultFigureVisible','off');

% 初始化树
T.x(1) = startPoint(1);
T.y(1) = startPoint(2);
T.z(1) = startPoint(3);
T.pre(1) = 0;

% 人工势场参数
k_att = 1;     % 引力系数
k_rep = 20.0;  % 斥力系数
rho_0 = 0.2;   % 斥力影响范围

% 初始化碰撞计数
collisionCount = 0;

% 存储边的信息数组
edges = zeros(iterMax, 6);
edge_count = 0;

% 路径规划计时
planningTimeStart = tic;

while iter < iterMax
    
    iter = iter+1;
    if mod(iter, 500) == 0
        fprintf('当前迭代: %d\n', iter);
    end
    
    %% 在空间内随机采样
    if rand < 0.01  % 有一定概率直接采样目标点
        randCoor = goalPoint;
    else
        randCoor = [rand*axisLWH(1)+axisStart(1), ...
                   rand*axisLWH(2)+axisStart(2), ...
                   rand*axisLWH(3)+axisStart(3)];
    end
    
    %% 寻找最近的节点
    [nearCoor,preIndex] = findNearPoint(randCoor,T);
    
    %% 计算人工势场
    % 计算引力
    diff_goal = goalPoint - nearCoor;
    dist_to_goal = norm(diff_goal);
    if dist_to_goal > rho_0
        att_force = k_att * rho_0 * diff_goal / dist_to_goal;  % 远距离使用常值引力
    else
        att_force = k_att * diff_goal;  % 近距离使用线性引力
    end
    
    % 计算斥力
    rep_force = zeros(1,3);
    
    % 计算立方体障碍物
    if cubeInfo.exist
        for i = 1:size(cubeInfo.axisX,2)
            obs_center = [cubeInfo.axisX(i) + cubeInfo.length(i)/2, ...
                        cubeInfo.axisY(i) + cubeInfo.width(i)/2, ...
                        cubeInfo.axisZ(i) + cubeInfo.height(i)/2];
            diff_obs = nearCoor - obs_center;
            dist_obs = norm(diff_obs);
            
            if dist_obs < rho_0
                rep_i = k_rep * (1/dist_obs - 1/rho_0) * (1/dist_obs^2) * (diff_obs/dist_obs);
                % 添加目标方向的抑制分量
                rep_force = rep_force + rep_i - dot(rep_i, diff_goal/dist_to_goal) * ...
                          (diff_goal/dist_to_goal) * 0.5;
            end
        end
    end
    
    % 圆柱体障碍物斥力
    if cylinderInfo.exist
        for i = 1:size(cylinderInfo.X,2)
            obs_center = [cylinderInfo.X(i), cylinderInfo.Y(i), ...
                        cylinderInfo.Z(i) + cylinderInfo.height(i)/2];
            diff_obs = nearCoor - obs_center;
            dist_obs = norm(diff_obs);
            
            if dist_obs < rho_0
                rep_i = k_rep * (1/dist_obs - 1/rho_0) * (1/dist_obs^2) * (diff_obs/dist_obs);
                rep_force = rep_force + rep_i - dot(rep_i, diff_goal/dist_to_goal) * ...
                          (diff_goal/dist_to_goal) * 0.5;
            end
        end
    end
    
    % 球体障碍物斥力
    if sphereInfo.exist
        for i = 1:size(sphereInfo.centerX,2)
            obs_center = [sphereInfo.centerX(i), sphereInfo.centerY(i), ...
                        sphereInfo.centerZ(i)];
            diff_obs = nearCoor - obs_center;
            dist_obs = norm(diff_obs);
            
            if dist_obs < rho_0
                rep_i = k_rep * (1/dist_obs - 1/rho_0) * (1/dist_obs^2) * (diff_obs/dist_obs);
                rep_force = rep_force + rep_i - dot(rep_i, diff_goal/dist_to_goal) * ...
                          (diff_goal/dist_to_goal) * 0.5;
            end
        end
    end
    
    % 合成力场
    total_force = att_force + rep_force;
    if norm(total_force) > 0
        total_force = total_force / norm(total_force);
    end
    
    %% 确定基本扩展方向，根据RRT采样
    basic_dir = randCoor - nearCoor;
    if norm(basic_dir) > 0
        basic_dir = basic_dir / norm(basic_dir);
    else
        continue;
    end
    
    % 动态调整权重
    if dist_to_goal > 0.2
        alpha = 0.6;  % 远离目标时，注重势场引导
    else
        alpha = 0.8;  % 接近目标时，注重RRT采样
    end
    
    %% 混合方向
    final_dir = alpha * basic_dir + (1-alpha) * total_force;
    if norm(final_dir) > 0
        final_dir = final_dir / norm(final_dir);
        newCoor = nearCoor + step * final_dir;
    else
        continue;
    end
    
    %% 碰撞检测
    cubeFlag = isCubeCollision(cubeInfo,nearCoor,newCoor,step);   % 立方体碰撞检测函数
    cylinderFlag = isCylinderCollision(cylinderInfo,nearCoor,newCoor,step);  % 圆柱体碰撞检测函数
    sphereFlag = isSphereCollision(sphereInfo,nearCoor,newCoor,step);   % 球体碰撞检测函数
    
    if cubeFlag || cylinderFlag || sphereFlag
        collisionCount = collisionCount + 1;
        continue;
    end
    
    %% 添加新节点到树
    count = count+1;
    T.x(count) = newCoor(1);
    T.y(count) = newCoor(2);
    T.z(count) = newCoor(3);
    T.pre(count) = preIndex;
    
    % 存储边信息
    edge_count = edge_count + 1;
    edges(edge_count,:) = [nearCoor(1) nearCoor(2) nearCoor(3) newCoor(1) newCoor(2) newCoor(3)];
    
    if calcuDis(newCoor,goalPoint) < Thr
        break;
    end 
    
end

% 记录路径规划时间
planningTime = toc(planningTimeStart);

if iter == iterMax
    Path = [];
    disp('路径规划失败');
    return;
end

%% 寻找路径
index = count;
pathCount = 1;
Path = [];

while T.pre(index)~=0
    Path(pathCount,1) = T.x(index);
    Path(pathCount,2) = T.y(index);
    Path(pathCount,3) = T.z(index);
    index = T.pre(index);
    pathCount = pathCount+1;
end

% 将起点添加到Path中
Path(pathCount,1) = startPoint(1);
Path(pathCount,2) = startPoint(2);
Path(pathCount,3) = startPoint(3);

% 将目标点添加到Path中
Path = flipud(Path);
pathCount = pathCount+1;
Path(pathCount,1) = goalPoint(1);
Path(pathCount,2) = goalPoint(2);
Path(pathCount,3) = goalPoint(3);

% 将探索相关信息存储在Path结构体中，以便后续使用
Path_info.edges = edges(1:edge_count,:);
Path_info.path = Path;
Path_info.iter = iter;
Path_info.count = count;
Path_info.collisionCount = collisionCount;
Path_info.planningTime = planningTime;
Path_info.totalTime = toc(totalTime);
Path_info.edge_countA = edge_count;

% 返回结构体而不是简单的路径
Path = Path_info;

% 输出统计信息
fprintf('规划成功!\n');
fprintf('总的迭代次数: %d\n', iter);
fprintf('总节点数: %d\n', count);
fprintf('有效节点数: %d\n', count - collisionCount);
fprintf('路径长度: %.2f m\n', sum(sqrt(sum(diff(Path.path).^2,2))));
fprintf('路径规划时间: %.3f秒\n', planningTime);
fprintf('总执行时间: %.3f秒\n', Path_info.totalTime);

end

function [nearCoor,preIndex] = findNearPoint(randCoor,T)
    % 寻找最近采样点的节点
    calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2);
    
    minDist = inf;
    preIndex = 1;
    
    for i = 1:length(T.x)
        nodeCoor = [T.x(i), T.y(i), T.z(i)];
        dist = calcuDis(nodeCoor, randCoor);
        
        if dist < minDist
            minDist = dist;
            preIndex = i;
        end
    end
    
    nearCoor = [T.x(preIndex), T.y(preIndex), T.z(preIndex)];
end