clear; clc; close all;
dtr = pi/180;

fprintf('=== 6-DOF Robot Forward Kinematics ===\n\n');

DH = [
    65,     pi/2,   135,  0;
    200,    0,       0,   0;
     0,     pi/2,   35,   0;
     0,     pi/2,   215,  0;
     0,     pi/2,    0,   0;
     0,      0,     55,   0;
];

fprintf('DH Parameters:\n');
fprintf('J |    a    | alpha |    d   \n');
for i=1:6
    fprintf('%d | %7.1f | %5.2f | %7.1f\n', i, DH(i,1), DH(i,2), DH(i,3));
end

% 2 define joint angle trajectory (Input)
fprintf('\n=== Defining Joint Space Trajectory ===\n');

N = 650; 
t = linspace(0, 2*pi, N);

% Joint angles that create square-like motion in task space
Q = zeros(N, 6);
Q(:,1) = 0 * ones(N,1);                 
Q(:,2) = 20 + 30*sin(t);                
Q(:,3) = -40 + 25*sin(2*t);               
Q(:,4) = zeros(N,1);                     
Q(:,5) = zeros(N,1);                  
Q(:,6) = zeros(N,1);                       

Q = Q * dtr;  % convert to radians

fprintf('Generated %d waypoints in joint space\n', N);

% 3 forward kinematic
fprintf('\n=== Computing Forward Kinematics ===\n');

pos = zeros(N, 3);           % End-effector positions
orient = zeros(N, 3, 3);     % End-effector orientations
joints = zeros(N, 7, 3);     % All joint positions

for i = 1:N
    T = eye(4);
    joints(i,1,:) = [0,0,0];  % Base at origin
    
    % Chain DH transformations
    for j = 1:6
        c = cos(Q(i,j));
        s = sin(Q(i,j));
        ca = cos(DH(j,2));
        sa = sin(DH(j,2));
        
        Tj = [c, -s*ca,  s*sa, DH(j,1)*c;
              s,  c*ca, -c*sa, DH(j,1)*s;
              0,  sa,    ca,   DH(j,3);
              0,  0,     0,    1];
        
        T = T * Tj;
        joints(i,j+1,:) = T(1:3,4)';
    end
    
    pos(i,:) = T(1:3,4)';
    orient(i,:,:) = T(1:3,1:3);
end

fprintf('FK computed for all waypoints\n');


% Verify 1mm spacing
spacing = sqrt(sum(diff(pos).^2, 2));
fprintf('End-effector spacing: mean=%.3fmm, std=%.4fmm\n', mean(spacing), std(spacing));

% Check fixed orientation
R0 = squeeze(orient(1,:,:));
orient_err = zeros(N,1);
for i = 1:N
    orient_err(i) = norm(squeeze(orient(i,:,:)) - R0, 'fro');
end
fprintf('Orientation error: mean=%.6f, max=%.6f\n\n', mean(orient_err), max(orient_err));

% 4 PLOTS
fprintf('=== Creating Plots ===\n');

figure('Position', [50,50,1400,700]);

% 3D Trajectory
subplot(2,3,1);
plot3(pos(:,1), pos(:,2), pos(:,3), 'b', 'LineWidth', 2);
hold on;
plot3(pos(1,1), pos(1,2), pos(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('End-Effector Trajectory (FK Result)'); view(45,30);

% Waypoint Spacing
subplot(2,3,2);
plot(spacing, 'b', 'LineWidth', 1.5);
hold on; yline(mean(spacing), 'r--', 'LineWidth', 2);
grid on;
xlabel('Waypoint Index'); ylabel('Distance (mm)');
title(sprintf('Waypoint Spacing (mean=%.2fmm)', mean(spacing)));

% Joint Angles
subplot(2,3,3);
plot(Q(:,1:3)/dtr, 'LineWidth', 2);
grid on;
xlabel('Waypoint Index'); ylabel('Angle (deg)');
title('Joint Angles (Input)');
legend('θ₁','θ₂','θ₃', 'Location', 'best');

% Orientation Error
subplot(2,3,4);
plot(orient_err, 'r', 'LineWidth', 2);
hold on; yline(mean(orient_err), 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Waypoint Index'); ylabel('Orientation Error');
title('Fixed End-Effector Orientation');

% Summary
subplot(2,3,5); axis off;
text(0.1, 0.8, sprintf(['FORWARD KINEMATICS SUMMARY\n\n' ...
    'Robot: UR3-like (6-DOF)\n' ...
    'Waypoints: %d\n\n' ...
    'End-Effector Path:\n' ...
    '  Spacing: %.2f ± %.3f mm\n\n' ...
    'Orientation:\n' ...
    '  Error: %.6f (fixed)'], ...
    N, mean(spacing), std(spacing), mean(orient_err)), ...
    'FontSize', 10, 'FontName', 'Courier');

% 5 Robot animation
fprintf('=== Animating Robot ===\n');
v = VideoWriter('robot_animation.mp4', 'MPEG-4');
v.FrameRate = 5;  
open(v);

figure('Position', [100,100,1000,800]);
plot3(pos(:,1), pos(:,2), pos(:,3), 'r--', 'LineWidth', 1.5);
hold on;
plot3(pos(1,1), pos(1,2), pos(1,3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
xlim([-100 500]); ylim([-400 400]); zlim([0 600]);
view(45,30);
title('6-DOF Robot Forward Kinematics');

for i = 1:15:N
    j = squeeze(joints(i,:,:));
    
    delete(findobj(gca, 'Tag', 'robot'));
    
    % Robot stick figure
    plot3(j(:,1), j(:,2), j(:,3), 'b-o', ...
        'LineWidth', 3, 'MarkerSize', 6, 'MarkerFaceColor', 'b', 'Tag', 'robot');
    
    % Base
    plot3(0, 0, 0, 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'Tag', 'robot');
    
    % End-effector
    plot3(j(end,1), j(end,2), j(end,3), 'ro', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'r', 'Tag', 'robot');
    
 
    plot3(pos(1:i,1), pos(1:i,2), pos(1:i,3), 'g-', 'LineWidth', 2, 'Tag', 'robot');
    
    title(sprintf('FK Animation - Waypoint %d/%d', i, N));

     % Capture frame
    frame = getframe(gcf);
    writeVideo(v, frame);
    drawnow; pause(0.03);
end

% Close video file
close(v);
fprintf('Video saved as robot_animation1.mp4\n');
fprintf('\n=== COMPLETE ===\n');


