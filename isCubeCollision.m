function cubeFlag = isCubeCollision(cubeInfo,nearCoor,newCoor,step)
%% 长方体碰撞检测函数，如果发生碰撞则返回1
 
cubeFlag = 0;
 
if cubeInfo.exist
    % 遍历所有立方体障碍物
    for k1 = 1:size(cubeInfo.axisX,2)
        % 获取当前立方体的各个边界坐标
        xMin = cubeInfo.axisX(k1);
        xMax = cubeInfo.axisX(k1) + cubeInfo.length(k1);
        yMin = cubeInfo.axisY(k1);
        yMax = cubeInfo.axisY(k1) + cubeInfo.width(k1);  % 修正：使用width而不是length
        zMin = cubeInfo.axisZ(k1);
        zMax = cubeInfo.axisZ(k1) + cubeInfo.height(k1); % 修正：使用height而不是length
        
        % 方法一：检查路径上的多个点
        % 生成更密集的检查点
        checkPoints = generateCheckPoints(nearCoor, newCoor, min(step/10, 0.001));
        
        % 检查每个点是否在立方体内
        for i = 1:size(checkPoints, 1)
            checkPoint = checkPoints(i, :);
            
            if (xMin <= checkPoint(1) && checkPoint(1) <= xMax) && ...
               (yMin <= checkPoint(2) && checkPoint(2) <= yMax) && ...
               (zMin <= checkPoint(3) && checkPoint(3) <= zMax)
                cubeFlag = 1;
                return;
            end
        end
        
        % 方法二：检查线段是否与立方体的任何一个面相交
        if checkLineBoxIntersection(nearCoor, newCoor, [xMin, yMin, zMin], [xMax, yMax, zMax])
            cubeFlag = 1;
            return;
        end
    end
end
end

% 生成两点之间的检查点
function checkPoints = generateCheckPoints(p1, p2, checkStep)
    % 计算两点间的总距离
    totalDist = norm(p2 - p1);
    
    % 计算需要的点数
    numPoints = ceil(totalDist / checkStep);
    
    % 初始化检查点数组
    checkPoints = zeros(numPoints, 3);
    
    % 计算方向向量
    dir = (p2 - p1) / totalDist;
    
    % 生成检查点
    for i = 1:numPoints
        t = (i-1) * checkStep / totalDist;
        if t > 1
            t = 1;
        end
        checkPoints(i, :) = p1 + t * (p2 - p1);
    end
end

% 检查线段是否与立方体相交
function intersect = checkLineBoxIntersection(p1, p2, boxMin, boxMax)
    % 线段方向向量
    dir = p2 - p1;
    
    % 初始化参数范围
    tmin = 0;
    tmax = 1;
    
    % 检查每个维度
    for i = 1:3
        if abs(dir(i)) < 1e-8
            % 线段平行于这个轴
            if p1(i) < boxMin(i) || p1(i) > boxMax(i)
                % 线段在盒子外部
                intersect = false;
                return;
            end
        else
            % 计算t值
            t1 = (boxMin(i) - p1(i)) / dir(i);
            t2 = (boxMax(i) - p1(i)) / dir(i);
            
            % 确保t1 <= t2
            if t1 > t2
                temp = t1;
                t1 = t2;
                t2 = temp;
            end
            
            % 更新tmin和tmax
            tmin = max(tmin, t1);
            tmax = min(tmax, t2);
            
            if tmin > tmax
                % 线段与盒子不相交
                intersect = false;
                return;
            end
        end
    end
    
    % 如果我们到达这里，说明线段与盒子相交
    intersect = true;
end