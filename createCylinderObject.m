function cylinderInfo = createCylinderObject(cylinderInfo, map_type)
% 生成适合机械臂工作空间[-1 1 -1 1 0 2]的圆柱体障碍物

rng(789);% 设置随机数种子以确保结果可重现

% map_type: 1=狭窄通道 2=墙面 3=简单地图

switch map_type      % 根据统一的工作空间范围[-1,1]×[-1,1]×[0,2]设置障碍物
    case 1  % 狭窄通道
        % 第一个圆柱体 - 放置在工作空间左上角
        radius(1) = 0.07;      
        height(1) = 1.29;
        centerX(1) = 0.48;     
        centerY(1) = -0.15;
        centerZ(1) = 0.0;  % 圆柱体底部z坐标
        
        % 第二个圆柱体 - 放置在工作空间左上角
        radius(2) = 0.07; 
        height(2) = 1.29;
        centerX(2) = -0.4;     
        centerY(2) = -0.3;     
        centerZ(2) = 0.0;  % 圆柱体底部z坐标

        cylinderInfo.exist = 1;
    case 2  % 墙面
        % 第一个圆柱体 - 放置在工作空间左上角
        radius(1) = 0.07;      
        height(1) = 1.15;
        centerX(1) = -0.4;     
        centerY(1) = 0.4;
        centerZ(1) = 0.0;  % 圆柱体底部z坐标
        
        % 第二个圆柱体 - 放置在工作空间左上角
        radius(2) = 0.07;
        height(2) = 1.15;
        centerX(2) = 0.4;     
        centerY(2) = 0.4;     
        centerZ(2) = 0.0;  % 圆柱体底部z坐标

        cylinderInfo.exist = 1;
    case 3  % 简单地图
        % 第一个圆柱体 - 放置在工作空间左上角
        radius(1) = 0.12;      
        height(1) = 1.15;
        centerX(1) = 0.4;     
        centerY(1) = 0;
        centerZ(1) = 0.0;  % 圆柱体底部z坐标

        % 第二个圆柱体 - 放置在工作空间左上角
        radius(2) = 0.12;
        height(2) = 1.15;
        centerX(2) = -0.4;     
        centerY(2) = 0;     
        centerZ(2) = 0.0;  % 圆柱体底部z坐标
        
                % 第三个圆柱体 - 放置在工作空间左上角
        radius(3) = 0.12;
        height(3) = 1.15;
        centerX(3) = 0.1;     
        centerY(3) = 0.4;     
        centerZ(3) = 0.0;  % 圆柱体底部z坐标
        
        cylinderInfo.exist = 1;
    case 4  % 空间球体地图
        % 第一个圆柱体 - 放置在工作空间左上角
        radius(1) = 0.12;      
        height(1) = 1.15;
        centerX(1) = 0.4;     
        centerY(1) = -0.35;
        centerZ(1) = 0.0;  % 圆柱体底部z坐标

        cylinderInfo.exist = 0;       
    otherwise
        cylinderInfo.exist = 0; % 没有障碍物
    end

% 赋值给cylinderInfo结构
cylinderInfo.X = centerX;
cylinderInfo.Y = centerY;
cylinderInfo.Z = centerZ;
cylinderInfo.radius = radius;
cylinderInfo.height = height;

end