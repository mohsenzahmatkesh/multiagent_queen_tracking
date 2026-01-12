close all

% --- PLOT 1: PATHS ---
for i = 1:numRobots
    plot(robotPaths{i}(:,1), robotPaths{i}(:,2), 'r', 'LineWidth', 0.2);
end
% title('Path Planning with Real Data Tracking');

% --- PLOT 2: ANGLE TRACKING ---
figure(2); 
hold on; box on
colors = lines(numRobots);
plot(psiHistory(:,1), psiHistory(:,2), 'k', 'LineWidth', 2, 'DisplayName', 'Queen \psi');
for i = 1:numRobots
    plot(psiHistory(:,1), psiHistory(:,2+i), 'Color', colors(i,:), 'LineWidth',1.7, 'DisplayName', sprintf('Agent %d', i));
end
xlabel('Time (s)');
ylabel('Heading Angle (deg)');
ylim([-180, 180]); 
yticks(-180:60:180);
% title('Orientation Tracking: Robots following Queen \psi in Sector');
lgd = legend('show', 'Location', 'northeast', 'NumColumns', 4);
lgd.FontSize = 8;        % try 6–9
lgd.ItemTokenSize = [10 8];
hold off;

% --- PREPARE DATA ---
simSteps = size(robotPaths{1}, 1); 
qX_plot = queenX(1:simSteps);
qY_plot = queenY(1:simSteps);
time_plot = queen_time(1:simSteps);
colors = lines(numRobots); 
% Standard Figure Size [left, bottom, width, height]
figSize = [100, 100, 600, 400];

% --- PLOT 3: 2D TRAJECTORIES (Map View) ---
figure(3);
set(gcf, 'Position', figSize);
hold on; box on
plot(qX_plot, qY_plot, 'k-', 'LineWidth', 0.5, 'DisplayName', 'Queen');
for i = 1:numRobots
    plot(robotPaths{i}(:,1), robotPaths{i}(:,2), 'Color', colors(i,:), 'LineWidth', 0.2, 'DisplayName', sprintf('Agent %d', i));
end
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
% title('2D Trajectories');
% 'NumColumns', 2 -> Creates 2 columns (2 items per row)
lgd = legend('show', 'Location', 'southeast');
lgd.FontSize = 8;        % try 6–9
lgd.ItemTokenSize = [10 8];
axis equal;

% --- PLOT 4: FORMATION ERROR ---
figure(4);
set(gcf, 'Position', figSize);
hold on; box on
for i = 1:numRobots
    distError = sqrt((robotPaths{i}(:,1) - qX_plot).^2 + (robotPaths{i}(:,2) - qY_plot).^2);
    plot(time_plot, distError, 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Agent %d', i));
end
xlabel('Time (s)');
ylabel('Distance to Queen (mm)');
% title('Formation Error');
lgd = legend('show', 'Location', 'northeast', 'NumColumns', 4);
lgd.FontSize = 8;        % try 6–9
lgd.ItemTokenSize = [10 8];

% --- PLOT 5: X & Y POSITIONS (Subplots) ---
figure(5);
set(gcf, 'Position', [100, 100, 600, 600]); 
% Subplot 1: X Positions
subplot(2,1,1);
hold on; box on
plot(time_plot, qX_plot, 'k-', 'LineWidth', 2, 'DisplayName', 'Queen X');
for i = 1:numRobots
    plot(time_plot, robotPaths{i}(:,1), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Agent %d', i));
end
ylabel('X Position (mm)');
% title('X Position Tracking');
lgd = legend('show', 'Location', 'northeast', 'NumColumns', 4);
lgd.FontSize = 8;        % try 6–9
lgd.ItemTokenSize = [10 8];

% Subplot 2: Y Positions
subplot(2,1,2);
hold on; box on
plot(time_plot, qY_plot, 'k-', 'LineWidth', 2, 'DisplayName', 'Queen Y');
for i = 1:numRobots
    plot(time_plot, robotPaths{i}(:,2), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Agent %d', i));
end
xlabel('Time (s)');
ylabel('Y Position (mm)');
% title('Y Position Tracking');
% legend('show', 'Location', 'northeast', 'NumColumns', 2);
% ==========================================================
%      ADD/REPLACE THIS BLOCK FOR FIGURE 6
% ==========================================================
% --- PLOT 6: SNAPSHOTS WITH ORIENTATION ARROWS ---
figure(6);
set(gcf, 'Position', [100, 100, 600, 800]); 

% --- SETTINGS FOR "MUCH BIGGER" VISUALS ---
arrowLen = 15;        % Set to 15 for visibility
queenSize = 400;      
robotSize = 250;      
lineWidth = 2.5;      

% --- SUBPLOT 1: TIME STEP 10 ---
subplot(2,1,1);
hold on;  box on;
axis equal;

% 1. Determine Index 
idx = min(150, simSteps);

% 2. Plot Queen Arrow (Outbound)
qX = qX_plot(idx);
qY = qY_plot(idx);
qPsi = psiHistory(idx, 2); 
u = arrowLen * cosd(qPsi);
v = arrowLen * sind(qPsi);
% Outbound: Start at (qX, qY)
quiver(qX, qY, u, v, 0, 'k', 'LineWidth', lineWidth, 'MaxHeadSize', 0.6, 'DisplayName', 'Queen');
scatter(qX, qY, queenSize, 'k', 'filled', 'HandleVisibility', 'off'); 

% 3. Plot Robot Arrows (Outbound)
for i = 1:numRobots
    rX = robotPaths{i}(idx, 1);
    rY = robotPaths{i}(idx, 2);
    rPsi = psiHistory(idx, 2+i); 
    
    u = arrowLen * cosd(rPsi);
    v = arrowLen * sind(rPsi);
    
    quiver(rX, rY, u, v, 0, 'Color', colors(i,:), 'LineWidth', lineWidth, 'MaxHeadSize', 0.6, 'DisplayName', sprintf('Agent %d', i));
    scatter(rX, rY, robotSize, colors(i,:), 'filled', 'HandleVisibility', 'off'); 
end

xlabel('X Position (mm)');
ylabel('Y Position (mm)');
% title(sprintf('Snapshot at Time Step (t = %.2fs)', time_plot(idx)));
lgd = legend('show', 'Location', 'southeast');
lgd.FontSize = 8;        % try 6–9
lgd.ItemTokenSize = [10 8];

% --- SUBPLOT 2: FINAL TIME STEP ---
subplot(2,1,2);
hold on;  box on;
axis equal;

% 1. Determine Index (Fixed at 999)
idx = 790; 

% 2. Plot Queen Arrow (Outbound)
qX = qX_plot(idx);
qY = qY_plot(idx);
qPsi = psiHistory(idx, 2); 
u = arrowLen * cosd(qPsi);
v = arrowLen * sind(qPsi);
quiver(qX, qY, u, v, 0, 'k', 'LineWidth', lineWidth, 'MaxHeadSize', 0.6, 'DisplayName', 'Queen');
scatter(qX, qY, queenSize, 'k', 'filled', 'HandleVisibility', 'off');

% 3. Plot Robot Arrows (Outbound)
for i = 1:numRobots
    rX = robotPaths{i}(idx, 1);
    rY = robotPaths{i}(idx, 2);
    rPsi = psiHistory(idx, 2+i); 
    
    u = arrowLen * cosd(rPsi);
    v = arrowLen * sind(rPsi);
    
    quiver(rX, rY, u, v, 0, 'Color', colors(i,:), 'LineWidth', lineWidth, 'MaxHeadSize', 0.6, 'DisplayName', sprintf('Robot %d', i));
    scatter(rX, rY, robotSize, colors(i,:), 'filled', 'HandleVisibility', 'off');
end

xlabel('X Position (mm)');
ylabel('Y Position (mm)');
% title(sprintf('Snapshot at time Step (t = %.2fs)', time_plot(idx)));
% legend('show', 'Location', 'northeast', 'NumColumns', 2);