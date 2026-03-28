function cylinderFlag = isCylinderCollision(cylinderInfo,nearCoor,newCoor,step)
%% Բ������ײ��⺯������������ײ��ʱ�򷵻�1
 
cylinderFlag = 0;
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2);
 
if cylinderInfo.exist
    for k1 = 1:size(cylinderInfo.X,2)
        zMin = cylinderInfo.Z(k1);
        zMax = zMin+cylinderInfo.height(k1);
        
        for k2 = 0:step/100:step
            deltaX = newCoor(1) - nearCoor(1);
            deltaY = newCoor(2) - nearCoor(2);
            deltaZ = newCoor(3) - nearCoor(3);
 
            r = sqrt(deltaX^2+deltaY^2+deltaZ^2);
            fai = atan2(deltaY,deltaX);
            theta = acos(deltaZ/r);
 
            x = k2*sin(theta)*cos(fai);
            y = k2*sin(theta)*sin(fai);
            z = k2*cos(theta);
            
            checkPoint = [x+nearCoor(1),y+nearCoor(2),z+nearCoor(3)];
            if calcuDis(checkPoint(1:2),[cylinderInfo.X(k1) cylinderInfo.Y(k1)])<cylinderInfo.radius(k1) && zMin<checkPoint(3) & checkPoint(3) < zMax
                cylinderFlag = 1;
                return;
            end
            
        end
        
    end
    
end
 
 
end
 