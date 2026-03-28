function sphereFlag = isSphereCollision(sphereInfo, nearCoor, newCoor, step)
sphereFlag = 0;
    calcuDis = @(x,y) sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2);
 
if sphereInfo.exist
   for k1 = 1:size(sphereInfo.centerX,2)
       center = [sphereInfo.centerX(k1) sphereInfo.centerY(k1) sphereInfo.centerZ(k1)];
            radius = sphereInfo.radius(k1);
            
            % 快速预检测：检查线段端点到球心的距离
            dist_near = calcuDis(nearCoor, center);
            dist_new = calcuDis(newCoor, center);
            
            % 如果两个端点都在球体外，且线段最近点也在球体外，则无碰撞
            if dist_near > radius && dist_new > radius
                % 计算线段到球心的最短距离
                segment = newCoor - nearCoor;
                segment_length = norm(segment);
                
                if segment_length < 1e-6  % 如果两点重合
                    continue;
                end
                
                % 计算线段到球心的最短距离
                t = max(0, min(1, dot(center - nearCoor, segment) / (segment_length^2)));
                closest_point = nearCoor + t * segment;
                min_dist = calcuDis(closest_point, center);
                
                % 如果最短距离大于半径，则无需详细检查
                if min_dist > radius * 1.05  % 添加5%的安全裕度
                    continue;
                end
            end
            
            % 详细碰撞检测
            total_dist = calcuDis(nearCoor, newCoor);
            % 根据距离和半径自适应调整检测密度
            num_checks = max(ceil(total_dist/min(step/10, radius/5)), 15);
            
            % 使用非均匀采样，在端点附近采样更密集
            for k2 = 0:num_checks
                t = k2/num_checks;
                % 使用余弦函数使采样在端点附近更密集
                t_modified = (1 - cos(t * pi)) / 2;
                checkPoint = nearCoor + t_modified * (newCoor - nearCoor);
                
                if calcuDis(checkPoint, center) <= radius * 1.02  % 添加2%的安全裕度
                sphereFlag = 1;
                return;
                end
            end
        end
   end
end
 