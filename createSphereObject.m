function sphereInfo = createSphereObject(sphereInfo, map_type)
% 生成适合机械臂工作空间[-1 1 -1 1 0 2]的球体障碍物

rng(456);% 设置随机数种子以确保结果可重现

% map_type: 1=狭窄通道 2=墙面 3=简单地图

switch map_type      % 根据统一的工作空间范围[-1,1]×[-1,1]×[0,2]设置障碍物
    case 1  % 狭窄通道
        % 第一个球体 - 放置在工作空间左下角
        radius(1) = 0.15;      
        centerX(1) = -0.7;     
        centerY(1) = -0.7;
        centerZ(1) = 0.7;

        sphereInfo.exist = 0;
    case 2  % 墙面
        % 第一个球体 - 放置在工作空间左下角
        radius(1) = 0.15;      
        centerX(1) = -0.7;     
        centerY(1) = -0.7;
        centerZ(1) = 0.7;
        

        sphereInfo.exist = 0;
    case 3  % 简单地图
        % 第一个球体 - 放置在工作空间左下角
        radius(1) = 0.15;      
        centerX(1) = -0.7;     
        centerY(1) = -0.7;
        centerZ(1) = 0.7;    

        sphereInfo.exist = 1;

    case 4  % 空间球体地图
        % 第一个球体 - 放置在工作空间左下角
        radius(1) = 0.4;      
        centerX(1) = 0.1;     
        centerY(1) = 0.0;
        centerZ(1) = 1.0;
        
        % 第二个球体 - 放置在工作空间中心
        radius(2) = 0.3;
        centerX(2) = -0.6;     
        centerY(2) = 0.6;     
        centerZ(2) = 1.6;
        
        % 第三个球体 - 放置在工作空间右上角
        radius(3) = 0.3;
        centerX(3) = 0.6;     
        centerY(3) = -0.6;     
        centerZ(3) = 0.6;
        
        
        % 第四个球体 - 放置在工作空间右上角
        radius(4) = 0.25;
        centerX(4) = -0.65;     
        centerY(4) = 0.1;     
        centerZ(4) = 1.2;
        
        % 第五个球体 - 放置在工作空间右上角
        radius(5) = 0.3;
        centerX(5) = -0.2;     
        centerY(5) = -0.6;     
        centerZ(5) = 0.6;
        
        % 第六个球体 - 放置在工作空间右上角
        radius(6) = 0.3;
        centerX(6) = 0.6;     
        centerY(6) = -0.2;     
        centerZ(6) = 1.4;

        % 第七个球体 - 放置在工作空间右上角
        radius(7) = 0.3;
        centerX(7) = 0.5;     
        centerY(7) = 0.2;     
        centerZ(7) = 1.0;

        % 第八个球体 - 放置在工作空间右上角
        radius(8) = 0.3;
        centerX(8) = 0.0;     
        centerY(8) = 0.4;     
        centerZ(8) = 1.6;

        % 第九个球体 - 放置在工作空间右上角
        radius(9) = 0.3;
        centerX(9) = -0.6;     
        centerY(9) = 0.7;     
        centerZ(9) = 0.3;

        % 第十个球体 - 放置在工作空间右上角
        radius(10) = 0.3;
        centerX(10) = 0.6;     
        centerY(10) = 0.7;     
        centerZ(10) = 1.6;        

        % 第十一个球体 - 放置在工作空间右上角
        radius(11) = 0.3;
        centerX(11) = 0.0;     
        centerY(11) = 0.0;     
        centerZ(11) = 0.4;    

        sphereInfo.exist = 1;        
    otherwise
        sphereInfo.exist = 0; % 没有障碍物
end

% 赋值给sphereInfo结构
sphereInfo.centerX = centerX;
sphereInfo.centerY = centerY;
sphereInfo.centerZ = centerZ;
sphereInfo.radius = radius;

end