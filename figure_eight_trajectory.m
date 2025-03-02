function [target_position,target_velocity] = figure_eight_trajectory(target_Init_position,time_Interval)
    %参数设置
    x0 = target_Init_position(1);   % 初始位置 x
    y0 = target_Init_position(2);   % 初始位置 y
    z0 = target_Init_position(3);   % 初始位置 z

    Ax = 400;   % 8字形在x方向的幅度
    Ay = 200;   % 8字形在y方向的幅度

    omega = 0.01; % 角速度
    T = 100;   % 总时间
    dt = time_Interval;  % 时间步长
    
    % 时间向量
    t = 0:dt:T-dt;

    % 轨迹生成
    x = x0 + Ax * sin(omega * t);
    y = y0 + Ay * sin(2 * omega * t);
    z = z0 * ones(size(t)); % z方向保持恒定
    azimuth =  atand(y./x);
    elevation = atand(z./sqrt(x.^2+y.^2));

    target_position(1,:) = x;
    target_position(2,:) = y;
    target_position(3,:) = z;
    target_position(4,:) = azimuth;
    target_position(5,:) = elevation;


    % 速度计算（有限差分法）
    vx = diff(x) / dt; % x方向速度
    vy = diff(y) / dt; % y方向速度
    vz = diff(z) / dt; % z方向速度

    % 对齐时间长度（补零）
    vx = [vx, vx(end)];
    vy = [vy, vy(end)];
    vz = [vz, vz(end)];

    % 速度大小
    v = sqrt(vx.^2 + vy.^2 + vz.^2);

    target_velocity(1,:) = vx;
    target_velocity(2,:) = vy;
    target_velocity(3,:) = vz;
    target_velocity(4,:) = v;

    % % 可视化轨迹
    % figure;
    % plot3(x, y, z, 'b-', 'LineWidth', 2);
    % grid on;
    % xlabel('X (m)');
    % ylabel('Y (m)');
    % zlabel('Z (m)');
    % title('8字形运动轨迹');
    % view(3); % 三维视图
    % 
    % % 可视化轨迹和速度
    % figure;
    % subplot(3, 1, 1);
    % plot(t, x, 'r', t, y, 'g', t, z, 'b');
    % xlabel('Time (s)');
    % ylabel('Position (m)');
    % legend('x(t)', 'y(t)', 'z(t)');
    % title('8字形运动轨迹');
    % 
    % subplot(3, 1, 2);
    % plot(t, vx, 'r', t, vy, 'g', t, vz, 'b');
    % xlabel('Time (s)');
    % ylabel('Velocity (m/s)');
    % legend('v_x(t)', 'v_y(t)', 'v_z(t)');
    % title('速度分量');
    % 
    % subplot(3, 1, 3);
    % plot(t, v, 'k');
    % xlabel('Time (s)');
    % ylabel('Speed (m/s)');
    % title('速度大小');

end

