function [UAV_x,UAV_y,UAV_z] = UAV_point(spatial_resolution,body_length,body_width,body_height,arm_length,rotor_length,rotateTheta)

    length_point = fix(body_length/spatial_resolution);
    width_point = fix(body_width/spatial_resolution);
    height_point = fix(body_height/spatial_resolution);

    body_x = zeros(1,height_point*length_point*width_point);
    body_y = zeros(1,height_point*length_point*width_point);
    body_z = zeros(1,height_point*length_point*width_point);

    point_index = 1;
    for ww = 0:width_point
        for ll = 0:length_point
            for hh = 0:height_point
                body_x(point_index) = -body_width/2 + ww*spatial_resolution;
                body_y(point_index) = -body_length/2 + ll*spatial_resolution;
                body_z(point_index) = hh*spatial_resolution;
                point_index = point_index + 1;
            end
        end
    end

    arm_point = fix(arm_length/spatial_resolution);

    arm_x = zeros(1,4*arm_point);
    arm_y = zeros(1,4*arm_point);
    arm_z = zeros(1,4*arm_point);

    point_index = 1;

    for theta = 45:90:315
        for aa = 0:arm_point
            arm_x(point_index) = aa*spatial_resolution*cosd(theta);
            arm_y(point_index) = aa*spatial_resolution*sind(theta);
            arm_z(point_index) = 0;
            point_index = point_index + 1;
        end
    end

    bodyArm_x = [body_x,arm_x];
    bodyArm_y = [body_y,arm_y];
    bodyArm_z = [body_z,arm_z];

    single_rotor_point = fix(rotor_length/spatial_resolution);
    single_rotor_x = zeros(1,4*single_rotor_point);
    single_rotor_y = zeros(1,4*single_rotor_point);
    single_rotor_z = zeros(1,4*single_rotor_point);

    point_index = 1;

    for theta = 90:90:360
        for rr = 0:single_rotor_point
            single_rotor_x(point_index) = rr*spatial_resolution*cosd(theta);
            single_rotor_y(point_index) = rr*spatial_resolution*sind(theta);
            single_rotor_z(point_index) = spatial_resolution;
            point_index = point_index + 1;
        end
    end

    [rotor_x,rotor_y,rotor_z] = rotateBlade(single_rotor_x, single_rotor_y, single_rotor_z, rotateTheta);


    rotor_x = [rotor_x+arm_length/sqrt(2),rotor_x-arm_length/sqrt(2),rotor_x-arm_length/sqrt(2),rotor_x+arm_length/sqrt(2)];
    rotor_y = [rotor_y+arm_length/sqrt(2),rotor_y+arm_length/sqrt(2),rotor_y-arm_length/sqrt(2),rotor_y-arm_length/sqrt(2)];
    rotor_z = [rotor_z,rotor_z,rotor_z,rotor_z];

    UAV_x = [bodyArm_x,rotor_x];
    UAV_y = [bodyArm_y,rotor_y];
    UAV_z = [bodyArm_z,rotor_z];


end

% 生成并旋转叶片散射点  
function [rotatedBladeX, rotatedBladeY, rotatedBladeZ] = rotateBlade(bladeX, bladeY, bladeZ, angleDeg)  
    angleRad = deg2rad(angleDeg);  
    R = [cos(angleRad) -sin(angleRad) 0; sin(angleRad) cos(angleRad) 0;0 0 1];  
    bladePoints = [bladeX; bladeY; bladeZ]; 
    rotatedBladePoints = R * bladePoints;  
    rotatedBladeX = rotatedBladePoints(1, :);  
    rotatedBladeY = rotatedBladePoints(2, :);  
    rotatedBladeZ = rotatedBladePoints(3, :);  
end 