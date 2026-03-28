function cubeInfo = createCubeObject(cubeInfo, map_type)
% 生成适合机械臂工作空间[-1 1 -1 1 0 2]的立方体障碍物

rng(789);% 设置随机数种子以确保结果可重现

% map_type: 1=狭窄通道 2=墙面 3=简单地图

switch map_type      % 根据统一的工作空间范围[-1,1]×[-1,1]×[0,2]设置障碍物
    case 1  % 狭窄通道
        % 第一个立方体 - 放置在工作空间右上角
        length(1) = 0.1;      
        width(1) = 0.95;
        height(1) = 0.5;
        axisX(1) = 0.25;     
        axisY(1) = -0.55;
        axisZ(1) = 1.5;  % 立方体底部z坐标
        
        % 第二个立方体 - 放置在工作空间左上角
        length(2) = 0.1;
        width(2) = 0.8;
        height(2) = 1.2;
        axisX(2) = 0.25;     
        axisY(2) = -0.45;     
        axisZ(2) = 0.0;  % 立方体底部z坐标

        % 第三个立方体 - 放置在工作空间左上角
        length(3) = 0.1;
        width(3) = 0.05;
        height(3) = 2.0;
        axisX(3) = 0.25;     
        axisY(3) = 0.35;     
        axisZ(3) = 0.0;  % 立方体底部z坐标

        cubeInfo.exist = 1;
    case 2  % 墙面
        % 第一个立方体 - 放置在工作空间右上角
        length(1) = 0.1;      
        width(1) = 0.7;
        height(1) = 1.3;
        axisX(1) = 0.0;     
        axisY(1) = 0.25;
        axisZ(1) = 0.0;  % 立方体底部z坐标
        
        cubeInfo.exist = 1;
    case 3  % 简单地图
        % 第一个立方体 - 放置在工作空间中部偏下
        length(1) = 0.18;
        width(1) = 0.6;
        height(1) = 1.2;
        axisX(1) = -0.4;     
        axisY(1) = -0.6;     
        axisZ(1) = 0.0;  % 立方体底部z坐标
        
        cubeInfo.exist = 1;
    case 4  % 空间球体地图
        % 第一个立方体 - 放置在工作空间中部偏下
        length(1) = 0.18;
        width(1) = 0.6;
        height(1) = 1.2;
        axisX(1) = -0.4;     
        axisY(1) = -0.6;     
        axisZ(1) = 0.0;  % 立方体底部z坐标
        
        cubeInfo.exist = 0;
    otherwise
        cubeInfo.exist = 0; % 没有障碍物
end

% 赋值给cubeInfo结构
cubeInfo.axisX = axisX;
cubeInfo.axisY = axisY;
cubeInfo.axisZ = axisZ;
cubeInfo.length = length;
cubeInfo.width = width;
cubeInfo.height = height;

end
