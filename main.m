%% 多算法路径规划对比测试
clear; clc; close all;

% ==================== 环境与起点目标 ====================
startPoint = [-0.8, -0.8, 0.5];     % 起点放在相对空旷区域
axisStart  = [-1, -1, 0];
axisLWH    = [2, 2, 2];

goalPoint  = [0.8, 0.8, 1.2];      % 目标也调整到较安全位置

% ==================== 障碍物 ====================
mapType = 3;                       % 3 = 简单地图（障碍较少，推荐先测试）
cubeInfo = createCubeObject(struct('exist',0), mapType);
cylinderInfo = createCylinderObject(struct('exist',0), mapType);
sphereInfo = createSphereObject(struct('exist',0), mapType);   % 简单地图下 sphere 通常 exist=0

fprintf('使用地图类型: %d | Cube exist=%d | Cylinder exist=%d | Sphere exist=%d\n', ...
    mapType, cubeInfo.exist, cylinderInfo.exist, sphereInfo.exist);

% ==================== 定义算法列表 ====================
algorithms = {
    'RRT*', @RRTStar;
    'TA-FF-RRT*', @TA_FF_RRTStar;
    'RRT-APF', @RRT_APF;
    'RRT*-Connect', @RRTStarConnect;
    'Informed RRT*', @Informed_RRTStar;
    %'DIB-RRT*-Connect', @DIB_RRTStar_Connect;
    'DR-IRRT*-GC', @DR_IRRTStar_GC;
    % 'MSBI-RRT*', @Informed_RRTStar;  % 注意：MSBI_RRTStar.m 文件实际是Informed_RRTStar函数
    %'Q-RRT*', @Q_RRTStar;
};

% 颜色列表用于可视化
colors = {'b-', 'r-', 'g-', 'c-', 'm-', 'y-'}; %'k--''w-''k-'

% ==================== 运行所有算法 ====================
results = cell(length(algorithms), 1);
for i = 1:length(algorithms)
    algName = algorithms{i, 1};
    algFunc = algorithms{i, 2};
    
    fprintf('运行 %s ...\n', algName);
    tic;
    try
        result = algFunc(startPoint, axisStart, axisLWH, goalPoint, cubeInfo, cylinderInfo, sphereInfo);
        time_taken = toc;
        
        % 统一结构体字段
        if isfield(result, 'path') && ~isempty(result.path) && size(result.path,1) > 1
            results{i} = struct('name', algName, 'path', result.path, 'success', true, 'time', time_taken, 'iter', result.iter, 'planningTime', result.planningTime);
            fprintf('%s 成功！路径点数: %d | 迭代: %d | 时间: %.3f秒\n', algName, size(result.path,1), result.iter, time_taken);

            % 兼容各算法返回字段：collisionCount/sharpCount/fragmentCount
            collisionCount = 0; sharpCount = 0; fragmentCount = 0;
            if isfield(result, 'collisionCount'); collisionCount = result.collisionCount; end
            if isfield(result, 'sharpCount'); sharpCount = result.sharpCount; end
            if isfield(result, 'fragmentCount'); fragmentCount = result.fragmentCount; end

            if collisionCount ~= 0 || sharpCount ~= 0 || fragmentCount ~= 0
                fprintf('碰撞失败:%d | 尖角:%d | 碎片:%d\n', collisionCount, sharpCount, fragmentCount);
            elseif isfield(result, 'collisionCount')
                fprintf('碰撞失败:%d\n', collisionCount);
            end
        else
            results{i} = struct('name', algName, 'path', [], 'success', false, 'time', time_taken, 'iter', 0, 'planningTime', 0);
            fprintf('%s 失败\n', algName);
        end
    catch ME
        results{i} = struct('name', algName, 'path', [], 'success', false, 'time', toc, 'iter', 0, 'planningTime', 0);
        fprintf('%s 出错: %s\n', algName, ME.message);
    end
end

% ==================== 打印路径 ====================
for i = 1:length(results)
    fprintf('\n%s 路径:\n', results{i}.name);
    if results{i}.success
        disp(results{i}.path);
    else
        disp('无路径');
    end
end

% ==================== 可视化路径 ====================
set(0,'DefaultFigureVisible','on');  % 启用图形显示

figure('Name','多算法路径规划对比','NumberTitle','off','Position',[100 100 1200 800]);
hold on; grid on; view(3);
axis([-1 1 -1 1 0 2]);

% 绘制障碍物
drawCylinderObject(cylinderInfo, [0.3 0.6 0.9], 0.7);
drawCubeObject(cubeInfo, [0.8 0.4 0.2], 0.7);
drawSphereObject(sphereInfo);  % 球体颜色在函数内部设置

% 绘制起点和目标
h_start = plot3(startPoint(1),startPoint(2),startPoint(3),'go','MarkerSize',12,'MarkerFaceColor','g');
h_goal = plot3(goalPoint(1),goalPoint(2),goalPoint(3),'ro','MarkerSize',12,'MarkerFaceColor','r');

% 绘制所有路径
legend_entries = [h_start, h_goal];
legend_labels = {'起点', '目标'};

for i = 1:length(results)
    if results{i}.success
        h_path = plot3(results{i}.path(:,1), results{i}.path(:,2), results{i}.path(:,3), colors{i}, 'LineWidth', 2);
        legend_entries = [legend_entries, h_path];
        legend_labels = [legend_labels, sprintf('%s (%.3fs)', results{i}.name, results{i}.time)];
    else
        h_path = plot3(nan, nan, nan, colors{i}, 'LineWidth', 2);  % 占位符
        legend_entries = [legend_entries, h_path];
        legend_labels = [legend_labels, sprintf('%s (失败)', results{i}.name)];
    end
end

xlabel('X'); ylabel('Y'); zlabel('Z');
title('多算法路径规划对比');

% 添加图例
legend(legend_entries, legend_labels, 'Location', 'bestoutside');  
