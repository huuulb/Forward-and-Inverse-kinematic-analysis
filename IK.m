clear; clc; close all;

% DH parameter
DH = [
    65,     pi/2,   135,  0;
    200,    0,       0,   0;
     0,     pi/2,   35,   0;
     0,     pi/2,   215,  0;
     0,     pi/2,    0,   0;
     0,      0,     55,   0;
];


%1.desires trajectory (square)
fprintf('=== Generating Square Trajectory ===\n');

sq_size = 100;
center = [400, 0, 300];
corners = [
    center + [-50, -50, 0];
    center + [ 50, -50, 0];
    center + [ 50,  50, 0];
    center + [-50,  50, 0];
];

% Generate square with 1mm spacing
traj_desired = [];
for i = 1:4
    c1 = corners(i, :);
    c2 = corners(mod(i,4)+1, :);
    for j = 0:99
        traj_desired = [traj_desired; c1 + j/100*(c2-c1)];
    end
end

% Desired orientation (fixed, tool pointing down)
R_desired = [1, 0, 0; 0, 1, 0; 0, 0, 1];

N = size(traj_desired, 1);
fprintf('Trajectory: %d waypoints\n\n', N);

% 2 NEWTON'S METHOD IK SOLVER
fprintf('=== Running Newton''s Method IK ===\n');

Q_solution = zeros(N, 6);
Q_current = [0; 20; -40; 0; 0; 0] * pi/180;  % Initial guess

ik_iterations = zeros(N, 1);
ik_errors = zeros(N, 1);

for wp = 1:N
    % Target pose
    p_target = traj_desired(wp, :)';
    T_target = [R_desired, p_target; 0 0 0 1];
    
    % Newton's method iteration
    q = Q_current;
    converged = false;
    
    for iter = 1:50  % Max iterations
        % Forward kinematics
        [T_current, J] = compute_fk_and_jacobian(q, DH);
        
        % Position and orientation error
        p_current = T_current(1:3, 4);
        R_current = T_current(1:3, 1:3);
        
        % Position error
        e_pos = p_target - p_current;
        
        % Orientation error (simplified - rotation vector)
        e_rot = 0.5 * [R_current(3,2) - R_current(2,3);
                       R_current(1,3) - R_current(3,1);
                       R_current(2,1) - R_current(1,2)];
        
        % Combined error (6x1)
        e = [e_pos; e_rot];
        
        % Check convergence
        if norm(e) < 1e-3
            converged = true;
            break;
        end
        
        % Newton's method update: q_new = q + J^+ * e
        % Using damped least squares for numerical stability
        lambda = 0.01;
        dq = (J' * J + lambda * eye(6)) \ (J' * e);
        
        q = q + dq;
    end
    
    Q_solution(wp, :) = q';
    Q_current = q;  % Use as initial guess for next waypoint
    
    ik_iterations(wp) = iter;
    ik_errors(wp) = norm(e);
    
    if mod(wp, 50) == 0
        fprintf('  Waypoint %d/%d: %d iterations, error=%.4f\n', ...
            wp, N, iter, norm(e));
    end
end

fprintf('IK complete!\n\n');

% 3 verify IK solution with FK
fprintf('=== Verifying IK Solution ===\n');

pos_actual = zeros(N, 3);
orient_actual = zeros(N, 3, 3);
joints = zeros(N, 7, 3);

for i = 1:N
    T = compute_fk(Q_solution(i, :)', DH);
    
    joints(i,1,:) = [0, 0, 0];
    T_accum = eye(4);
    for j = 1:6
        T_accum = T_accum * dh_transform(DH(j,:), Q_solution(i,j));
        joints(i,j+1,:) = T_accum(1:3,4)';
    end
    
    pos_actual(i,:) = T(1:3,4)';
    orient_actual(i,:,:) = T(1:3,1:3);
end

pos_error = sqrt(sum((traj_desired - pos_actual).^2, 2));

fprintf('Position error: mean=%.4fmm, max=%.4fmm\n', mean(pos_error), max(pos_error));
fprintf('IK iterations: mean=%.1f, max=%d\n\n', mean(ik_iterations), max(ik_iterations));

% 4 PLOTS
fprintf('=== Creating Analysis Plots ===\n');

figure('Position', [50,50,1400,800]);

% Desired vs Actual Trajectory
subplot(2,3,1);
plot3(traj_desired(:,1), traj_desired(:,2), traj_desired(:,3), 'r--', 'LineWidth', 2);
hold on;
plot3(pos_actual(:,1), pos_actual(:,2), pos_actual(:,3), 'b-', 'LineWidth', 1.5);
plot3(corners(:,1), corners(:,2), corners(:,3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('IK Solution: Desired vs Actual');
legend('Desired', 'Actual (IK+FK)', 'Location', 'best');
view(45,30);

% Position Error
subplot(2,3,2);
plot(pos_error, 'r', 'LineWidth', 2);
hold on;
yline(mean(pos_error), 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Waypoint'); ylabel('Error (mm)');
title(sprintf('Position Error (mean=%.3fmm)', mean(pos_error)));

% Joint Angles
subplot(2,3,3);
plot(Q_solution(:,1:3)*180/pi, 'LineWidth', 2);
grid on;
xlabel('Waypoint'); ylabel('Angle (deg)');
title('Joint Angles from IK');
legend('θ₁','θ₂','θ₃');

% IK Iterations
subplot(2,3,4);
plot(ik_iterations, 'b', 'LineWidth', 2);
hold on;
yline(mean(ik_iterations), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Waypoint'); ylabel('Iterations');
title(sprintf('Newton Method Convergence (mean=%.1f)', mean(ik_iterations)));

% Summary
subplot(2,3,5); axis off;
text(0.1, 0.8, sprintf(['INVERSE KINEMATICS\n' ...
    'Newton''s Method\n\n' ...
    'Waypoints: %d\n' ...
    'Converged: %d/%d\n\n' ...
    'Position Error:\n' ...
    '  Mean: %.4f mm\n' ...
    '  Max: %.4f mm\n\n' ...
    'Iterations:\n' ...
    '  Mean: %.1f\n' ...
    '  Max: %d'], ...
    N, sum(ik_errors < 1e-3), N, mean(pos_error), max(pos_error), ...
    mean(ik_iterations), max(ik_iterations)), ...
    'FontSize', 10, 'FontName', 'Courier');

% 5 ANIMATION WITH RGB FRAME
fprintf('=== Creating Animation with SE(3) Visualization ===\n');

fig = figure('Position', [100,100,1200,900]);
ax = axes('Parent', fig);

% Plot desired path
plot3(ax, traj_desired(:,1), traj_desired(:,2), traj_desired(:,3), ...
    'r--', 'LineWidth', 1.5);
hold(ax, 'on');
plot3(ax, corners(:,1), corners(:,2), corners(:,3), ...
    'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'X (mm)'); ylabel(ax, 'Y (mm)'); zlabel(ax, 'Z (mm)');
xlim(ax, [0 600]); ylim(ax, [-200 200]); zlim(ax, [0 500]);
view(ax, 45, 30);
title(ax, 'IK Solution - Robot Following Square Path');

% Animate
v = VideoWriter('robot_animation.mp4', 'MPEG-4');
v.FrameRate = 5;  
open(v);
for i = 1:5:N
    % Clear previous robot
    delete(findobj(ax, 'Tag', 'robot'));
    
    % Get joint positions
    j = squeeze(joints(i,:,:));
    
    % Draw robot stick figure
    plot3(ax, j(:,1), j(:,2), j(:,3), 'b-o', ...
        'LineWidth', 3, 'MarkerSize', 6, 'MarkerFaceColor', 'b', 'Tag', 'robot');
    
    % Base
    plot3(ax, 0, 0, 0, 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'Tag', 'robot');
    
    % End-effector position
    ee_pos = pos_actual(i,:);
    plot3(ax, ee_pos(1), ee_pos(2), ee_pos(3), 'yo', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'y', 'Tag', 'robot');
    
    % RGB FRAME at end-effector (SE(3) visualization)
    R_ee = squeeze(orient_actual(i,:,:));
    frame_scale = 50;  % 50mm axes
    
    % X-axis (RED)
    x_axis = ee_pos' + R_ee(:,1) * frame_scale;
    plot3(ax, [ee_pos(1), x_axis(1)], [ee_pos(2), x_axis(2)], [ee_pos(3), x_axis(3)], ...
        'r-', 'LineWidth', 3, 'Tag', 'robot');
    
    % Y-axis (GREEN)
    y_axis = ee_pos' + R_ee(:,2) * frame_scale;
    plot3(ax, [ee_pos(1), y_axis(1)], [ee_pos(2), y_axis(2)], [ee_pos(3), y_axis(3)], ...
        'g-', 'LineWidth', 3, 'Tag', 'robot');
    
    % Z-axis (BLUE)
    z_axis = ee_pos' + R_ee(:,3) * frame_scale;
    plot3(ax, [ee_pos(1), z_axis(1)], [ee_pos(2), z_axis(2)], [ee_pos(3), z_axis(3)], ...
        'b-', 'LineWidth', 3, 'Tag', 'robot');
    
    % Traced path
    plot3(ax, pos_actual(1:i,1), pos_actual(1:i,2), pos_actual(1:i,3), ...
        'g-', 'LineWidth', 2, 'Tag', 'robot');
    
    title(ax, sprintf('Waypoint %d/%d | Error: %.3fmm | RGB=XYZ Frame', ...
        i, N, pos_error(i)));
    
        % Capture frame
    frame = getframe(gcf);
    writeVideo(v, frame);
    drawnow;
    pause(0.03);
end
% Close video file
close(v);
fprintf('Video saved as robot_animation.mp4\n');
fprintf('\n=== COMPLETE ===\n');

fprintf('\n=== COMPLETE ===\n');

% functions

function T = compute_fk(q, DH)
    % Forward kinematics
    T = eye(4);
    for i = 1:length(q)
        T = T * dh_transform(DH(i,:), q(i));
    end
end

function [T, J] = compute_fk_and_jacobian(q, DH)
    % Forward kinematics with Jacobian
    n = length(q);
    T = eye(4);
    T_array = zeros(4, 4, n+1);
    T_array(:,:,1) = eye(4);
    
    % Forward pass
    for i = 1:n
        T = T * dh_transform(DH(i,:), q(i));
        T_array(:,:,i+1) = T;
    end
    
    % Compute Jacobian (geometric)
    J = zeros(6, n);
    p_ee = T(1:3, 4);
    
    for i = 1:n
        T_i = T_array(:,:,i);
        z_i = T_i(1:3, 3);  % Z-axis of frame i
        p_i = T_i(1:3, 4);  % Origin of frame i
        
        % Linear velocity contribution
        J(1:3, i) = cross(z_i, p_ee - p_i);
        
        % Angular velocity contribution
        J(4:6, i) = z_i;
    end
end

function T = dh_transform(dh_row, theta)
    % DH transformation matrix
    a = dh_row(1);
    alpha = dh_row(2);
    d = dh_row(3);
    
    c = cos(theta);
    s = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);
    
    T = [c, -s*ca,  s*sa, a*c;
         s,  c*ca, -c*sa, a*s;
         0,  sa,    ca,   d;
         0,  0,     0,    1];
end
