function drawSphereObject(sphereInfo,colorMat,pellucidity)

if ~sphereInfo.exist
    return;
end

% 设置三色渐变
color1 = [1 0.9 0.2];    % 黄色
color2 = [0.2 0.6 1];    % 蓝色
color3 = [0.3 0.9 0.4];  % 绿色
pellucidity = 0.6;       % 透明度

numSpheres = length(sphereInfo.centerX);

for i = 1:numSpheres
    centerX = sphereInfo.centerX(i);
    centerY = sphereInfo.centerY(i);
    centerZ = sphereInfo.centerZ(i);
    radius = sphereInfo.radius(i);
    
    % 增加球面分辨率，使线条更平滑
    [x,y,z] = sphere(30);
    x = radius * x + centerX;
    y = radius * y + centerY;
    z = radius * z + centerZ;
    
    % 创建三色渐变效果
    c = (z - min(z(:))) / (max(z(:)) - min(z(:)));
    colormap = zeros(size(z,1), size(z,2), 3);
    
    % 三色渐变插值
    for j = 1:3
        % 下半部分：黄色到蓝色
        mask_lower = c <= 0.5;
        colormap(:,:,j) = mask_lower .* (color1(j) + (c*2) .* (color2(j) - color1(j)));
        
        % 上半部分：蓝色到绿色
        mask_upper = c > 0.5;
        colormap(:,:,j) = colormap(:,:,j) + mask_upper .* (color2(j) + ((c-0.5)*2) .* (color3(j) - color2(j)));
    end
    
    % 绘制带渐变色的球体
    surf(x,y,z,...
        'CData', colormap,...         % 使用渐变色数据
        'FaceColor','interp',...      % 插值显示渐变色
        'EdgeColor',[0.2 0.4 0.6],... % 中性色线条
        'EdgeAlpha',0.7,...           % 线条透明度
        'FaceAlpha',pellucidity,...   % 面的透明度
        'LineWidth',0.5);             % 线条宽度
    
    % 添加三个主圆环
    t = linspace(0,2*pi,50);
    ringColor = [0.4 0.6 0.4];  % 中性偏绿色圆环
    
    % XY平面圆环
    plot3(centerX + radius*cos(t), centerY + radius*sin(t), centerZ*ones(size(t)),...
        'Color',ringColor,'LineWidth',1.2);
    
    % XZ平面圆环
    plot3(centerX + radius*cos(t), centerY*ones(size(t)), centerZ + radius*sin(t),...
        'Color',ringColor,'LineWidth',1.2);
    
    % YZ平面圆环
    plot3(centerX*ones(size(t)), centerY + radius*cos(t), centerZ + radius*sin(t),...
        'Color',ringColor,'LineWidth',1.2);
end

end