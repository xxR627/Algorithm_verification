function [best_velocity, best_theta, final_height, trajectory] = basketball_simulation(target_distance)
% 篮球投篮轨迹模拟
% 输入：target_distance - 目标距离(m)
% 输出：best_velocity - 最优发射速度(m/s)
%       best_theta - 最优发射角度(rad)
%       final_height - 终点高度(m)
%       trajectory - 包含轨迹坐标的结构体

% 定义环境参数
environment.air_density = 1.225;  % 空气密度 (kg/m^3)
environment.gravity = 9.81;       % 重力加速度 (m/s^2)

% 定义篮球参数
basketball.mass = 0.62;           % 篮球质量 (kg)
basketball.radius = 0.12;         % 篮球半径 (m)
basketball.cross_area = pi * basketball.radius^2;  % 横截面积
basketball.drag_coeff = 0.47;     % 阻力系数

% 定义目标参数
target.target_height = 2.43;       % 篮筐高度 (m)
target.optimal_entry_angle = 55;   % 最佳入射角度（度）

% 定义机器人参数
robot.base_angle = 65;            % 基准角度 (度)
robot.axis_height = 0.5;          % 轴心高度 (m)
robot.shoot_radius = 0.2;         % 发射机构半径 (m)

% 计算实际发射点高度
mechanical_angle = robot.base_angle * pi / 180;
robot.robot_height = robot.axis_height + robot.shoot_radius * sin(mechanical_angle);

% 判断距离，选择不同的算法
if target_distance < 2.0
    % 使用近距离算法
    [best_velocity, best_theta, final_height, trajectory] = close_range_algorithm(target_distance, target, robot, environment, basketball);
else
    % 使用远距离算法
    [best_velocity, best_theta, final_height, trajectory] = long_range_algorithm(target_distance, target, robot, environment, basketball);
end

% 显示结果
fprintf('目标距离: %.2f m\n', target_distance);
fprintf('最优发射角度: %.2f度\n', best_theta * 180/pi);
fprintf('最优发射速度: %.2f m/s\n', best_velocity);
fprintf('终点高度: %.2f m\n', final_height);

% 绘制轨迹
plot_results(trajectory, target_distance, target.target_height);
end

function [best_velocity, best_theta, final_height, trajectory] = close_range_algorithm(target_distance, target, robot, environment, basketball)
    % 近距离算法 - 采用更陡峭的抛物线
    
    % 调整搜索范围 - 增加速度范围
    min_angle = 65 * pi / 180;  % 最小角度65度
    max_angle = 85 * pi / 180;  % 最大角度85度
    
    % 根据距离动态调整速度范围 - 提高速度
    if target_distance < 1.0
        vel_min = 4.5;          % 提高最小速度
        vel_max = 6.5;          % 提高最大速度
    else
        vel_min = 5.5;          % 提高最小速度
        vel_max = 7.5;          % 提高最大速度
    end
    
    best_theta = min_angle;
    best_velocity = vel_min;
    min_error = 1000;
    dt = 0.01;
    
    % 更细致的搜索步长
    angle_step = 0.2 * pi / 180;
    vel_step = 0.05;
    
    % 从大角度开始搜索
    for test_theta = max_angle:-angle_step:min_angle
        for test_vel = vel_min:vel_step:vel_max
            [error, traj_x, traj_y, entry_angle] = calculate_trajectory_close(test_theta, test_vel, ...
                target_distance, robot, environment, basketball, target, dt, vel_min);
            
            if error < min_error
                min_error = error;
                best_theta = test_theta;
                best_velocity = test_vel;
            end
        end
    end
    
    [~, final_trajectory_x, final_trajectory_y] = calculate_trajectory_close(best_theta, best_velocity, ...
        target_distance, robot, environment, basketball, target, dt, vel_min);
    
    trajectory.x = final_trajectory_x;
    trajectory.y = final_trajectory_y;
    final_height = final_trajectory_y(end);
end

function [best_velocity, best_theta, final_height, trajectory] = long_range_algorithm(target_distance, target, robot, environment, basketball)
    % 远距离算法
    min_angle = 45 * pi / 180;
    max_angle = 65 * pi / 180;
    vel_min = 8.0;
    vel_max = 11.0;
    
    best_theta = min_angle;
    best_velocity = vel_min;
    min_error = 1000;
    dt = 0.01;
    
    angle_step = 0.2 * pi / 180;
    vel_step = 0.05;
    
    for test_theta = max_angle:-angle_step:min_angle
        for test_vel = vel_min:vel_step:vel_max
            [error, traj_x, traj_y, entry_angle] = calculate_trajectory_long(test_theta, test_vel, ...
                target_distance, robot, environment, basketball, target, dt);
            
            if error < min_error
                min_error = error;
                best_theta = test_theta;
                best_velocity = test_vel;
            end
        end
    end
    
    [~, final_trajectory_x, final_trajectory_y] = calculate_trajectory_long(best_theta, best_velocity, ...
        target_distance, robot, environment, basketball, target, dt);
    
    trajectory.x = final_trajectory_x;
    trajectory.y = final_trajectory_y;
    final_height = final_trajectory_y(end);
end

function [error, traj_x, traj_y, entry_angle] = calculate_trajectory_close(theta, velocity, target_distance, ...
    robot, environment, basketball, target, dt, vel_min)
    % 近距离轨迹计算和误差评估
    
    % 初始化参数
    x = 0;
    y = robot.robot_height;
    vx = velocity * cos(theta);
    vy = velocity * sin(theta);
    
    traj_x = x;
    traj_y = y;
    prev_x = x;
    prev_y = y;
    
    % 轨迹计算
    while x <= target_distance && y >= 0 && length(traj_x) < 500
        v = sqrt(vx^2 + vy^2);
        drag = 0.5 * environment.air_density * basketball.cross_area * ...
               basketball.drag_coeff * v;
        
        ax = -(drag * vx / v) / basketball.mass;
        ay = -(drag * vy / v) / basketball.mass - environment.gravity;
        
        vx = vx + ax * dt;
        vy = vy + ay * dt;
        
        prev_x = x;
        prev_y = y;
        x = x + vx * dt;
        y = y + vy * dt;
        
        traj_x(end+1) = x;
        traj_y(end+1) = y;
        
        if abs(x - target_distance) < 0.01
            break;
        end
    end
    
    % 计算入射角度
    dx = x - prev_x;
    dy = y - prev_y;
    entry_angle = atan2(-dy, dx) * 180 / pi;
    
    % 修改近距离特殊误差计算
    if abs(x - target_distance) < 0.01
        height_error = abs(y - target.target_height);
        error = height_error * 50.0;  % 增加高度误差权重
        
        % 严格控制高度不足的情况
        if y < target.target_height
            error = error + (target.target_height - y) * 60.0;  % 增加低于目标的惩罚
        end
        
        % 入射角度误差
        angle_error = abs(entry_angle - 60);
        error = error + angle_error * 1.0;  % 降低角度误差权重
        
        % 轨迹高度检查
        max_height = max(traj_y);
        if max_height > target.target_height + 1.5  % 允许更高的轨迹
            error = error + (max_height - (target.target_height + 1.5)) * 5.0;
        end
        
        % 添加速度惩罚
        error = error + (velocity - vel_min) * 0.2;  % 降低速度惩罚
    else
        error = 1000;
    end
end

function [error, traj_x, traj_y, entry_angle] = calculate_trajectory_long(theta, velocity, target_distance, ...
    robot, environment, basketball, target, dt)
    % 远距离轨迹计算和误差评估（与近距离类似，但参数不同）
    
    % 初始化参数
    x = 0;
    y = robot.robot_height;
    vx = velocity * cos(theta);
    vy = velocity * sin(theta);
    
    traj_x = x;
    traj_y = y;
    prev_x = x;
    prev_y = y;
    
    while x <= target_distance && y >= 0 && length(traj_x) < 500
        v = sqrt(vx^2 + vy^2);
        drag = 0.5 * environment.air_density * basketball.cross_area * ...
               basketball.drag_coeff * v;
        
        ax = -(drag * vx / v) / basketball.mass;
        ay = -(drag * vy / v) / basketball.mass - environment.gravity;
        
        vx = vx + ax * dt;
        vy = vy + ay * dt;
        
        prev_x = x;
        prev_y = y;
        x = x + vx * dt;
        y = y + vy * dt;
        
        traj_x(end+1) = x;
        traj_y(end+1) = y;
        
        if abs(x - target_distance) < 0.02
            break;
        end
    end
    
    dx = x - prev_x;
    dy = y - prev_y;
    entry_angle = atan2(-dy, dx) * 180 / pi;
    
    if abs(x - target_distance) < 0.02
        height_error = abs(y - target.target_height);
        error = height_error * 25.0;
        
        if y < target.target_height
            error = error + (target.target_height - y) * 30.0;
        end
        
        angle_error = abs(entry_angle - target.optimal_entry_angle);
        error = error + angle_error * 0.8;
        
        max_height = max(traj_y);
        if max_height > 3.5
            error = error + (max_height - 3.5) * 2.0;
        end
    else
        error = 1000;
    end
end

function plot_results(trajectory, target_distance, target_height)
    figure('Name', '篮球发射轨迹模拟');
    hold on;
    grid on;
    xlabel('距离 (m)');
    ylabel('高度 (m)');
    title('篮球发射轨迹模拟');
    
    % 绘制目标点
    plot(target_distance, target_height, 'r*', 'MarkerSize', 10);
    
    % 绘制篮筐
    rim_width = 0.45;  % 篮筐宽度
    rim_x = [target_distance-rim_width/2, target_distance+rim_width/2];
    rim_y = [target_height, target_height];
    plot(rim_x, rim_y, 'r-', 'LineWidth', 2);
    
    % 绘制轨迹
    plot(trajectory.x, trajectory.y, 'b-', 'LineWidth', 2);
    
    % 添加图例
    legend('目标点', '篮筐', '投篮轨迹');
    
    % 设置坐标轴范围
    xlim([0, target_distance+0.5]);
    ylim([0, max(trajectory.y)+0.5]);
end

% 测试1：1米距离
disp('测试1：1米距离');
[v1, theta1, h1, traj1] = basketball_simulation(1.0);

% 测试2：1.5米距离
disp('测试2：1.5米距离');
[v2, theta2, h2, traj2] = basketball_simulation(1.5);

% 测试3：1.8米距离
disp('测试3：1.8米距离');
[v3, theta3, h3, traj3] = basketball_simulation(1.8); 

