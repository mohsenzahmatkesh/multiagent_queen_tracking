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
xlim([0, 107.764537600000]); 
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
figSize = [100, 100, 600, 400];

% --- PLOT 3: 2D TRAJECTORIES (Map View) ---
figure(3);
set(gcf, 'Position', figSize);
hold on; box on
plot(qX_plot, qY_plot, 'k-', 'LineWidth', 2.5, 'DisplayName', 'Queen');
for i = 1:numRobots
    plot(robotPaths{i}(:,1), robotPaths{i}(:,2), 'Color', colors(i,:), 'LineWidth', 1.2, 'DisplayName', sprintf('Agent %d', i));
end
xlabel('X Position (cm)');
ylabel('Y Position (cm)');
 
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
ylabel('Distance to Queen (cm)');
xlim([0, 107.764537600000]);
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
ylabel('X Position (cm)');
xlim([0, 107.764537600000]);
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
ylabel('Y Position (cm)');
xlim([0, 107.764537600000]);
% title('Y Position Tracking');
% legend('show', 'Location', 'northeast', 'NumColumns', 2);
% ==========================================================
%      ADD/REPLACE THIS BLOCK FOR FIGURE 6
% ==========================================================
% --- PLOT 6: SNAPSHOTS WITH OVALS (HORIZONTAL LAYOUT) ---
figure(6);
set(gcf, 'Position', [100, 100, 1200, 400]); 

% --- 1. CALCULATE GLOBAL LIMITS ---
all_X = qX_plot;
all_Y = qY_plot;
for i = 1:numRobots
    all_X = [all_X; robotPaths{i}(:,1)];
    all_Y = [all_Y; robotPaths{i}(:,2)];
end
globalXLim = [min(all_X)-5, max(all_X)+5];
globalYLim = [min(all_Y)-5, max(all_Y)+5];

% --- SETTINGS FOR VISUALS ---
q_a = 2.8; q_b = 1.5;     % Queen Dimensions
r_a = 2.5; r_b = 1.5;     % Robot Dimensions
t_param = linspace(0, 2*pi, 50); 
snapshotIndices = [min(680, simSteps), 800, 1850]; 

% Use TiledLayout for narrow borders
tlo = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

for k = 1:3
    nexttile;
    hold on; box on;
    axis equal;
    
    % Set Axes Font Size to match standard MATLAB defaults (usually 10-11)
    set(gca, 'FontSize', 10, 'LineWidth', 1); 
    
    idx = snapshotIndices(k);
    
    % --- PLOT QUEEN (Black Oval) ---
    qX = qX_plot(idx);
    qY = qY_plot(idx);
    qPsi = psiHistory(idx, 2); 
    
    X_ell = q_a * cos(t_param);
    Y_ell = q_b * sin(t_param);
    R = [cosd(qPsi), -sind(qPsi); sind(qPsi), cosd(qPsi)];
    coords = R * [X_ell; Y_ell];
    
    % LOGIC: Only show Queen in legend for the first plot (k=1)
    if k == 1
        fill(coords(1,:) + qX, coords(2,:) + qY, 'k', 'EdgeColor', 'none', 'DisplayName', 'Queen');
    else
        fill(coords(1,:) + qX, coords(2,:) + qY, 'k', 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
    
    % --- PLOT ROBOTS ---
    hasPlottedRed = false; 
    hasPlottedGold = false;
    
    for i = 1:numRobots
        rX = robotPaths{i}(idx, 1);
        rY = robotPaths{i}(idx, 2);
        rPsi = psiHistory(idx, 2+i); 
        
        % Check if Head-on (Active)
        angleDiff = mod(qPsi - robotFormationAngles(i) + 180, 360) - 180;
        isHeadOn = (abs(angleDiff) <= 40);
        
        % OVAL COORDINATES
        X_ell = r_a * cos(t_param);
        Y_ell = r_b * sin(t_param);
        R = [cosd(rPsi), -sind(rPsi); sind(rPsi), cosd(rPsi)];
        coords = R * [X_ell; Y_ell];
        
        if isHeadOn
            agentColor = [1, 0.84, 0]; % Gold
            legendName = sprintf('Head-on (Pos: %.0f^{\\circ})', robotFormationAngles(i));
            
            % LOGIC: Always show Gold legend for the specific agent in this snapshot
            if ~hasPlottedGold
                fill(coords(1,:) + rX, coords(2,:) + rY, agentColor, ...
                    'EdgeColor', 'none', 'DisplayName', legendName);
                hasPlottedGold = true;
            else
                fill(coords(1,:) + rX, coords(2,:) + rY, agentColor, ...
                    'EdgeColor', 'none', 'HandleVisibility', 'off');
            end
        else
            agentColor = 'r';          % Red
            
            % LOGIC: Only show "Agents" legend in the first plot (k=1)
            if k == 1 && ~hasPlottedRed
                fill(coords(1,:) + rX, coords(2,:) + rY, agentColor, ...
                    'EdgeColor', 'none', 'DisplayName', 'Agents');
                hasPlottedRed = true;
            else
                fill(coords(1,:) + rX, coords(2,:) + rY, agentColor, ...
                    'EdgeColor', 'none', 'HandleVisibility', 'off');
            end
        end
    end
    
    % --- APPLY LIMITS & LABELS ---
    xlim(globalXLim);
    ylim(globalYLim);
    xlabel('X Position (cm)');
    title(sprintf('t = %.2fs', time_plot(idx)));
    
    % Y-Axis Logic: Show label only on first plot
    if k == 1
        ylabel('Y Position (cm)');
    else
        set(gca, 'YTickLabel', []);
    end

    % --- LEGEND STYLING (MATCHING YOUR REFERENCE) ---
    lgd = legend('show', 'Location', 'northeast');
    lgd.FontSize = 8;          % Matches your reference
    lgd.ItemTokenSize = [10 8]; % Matches your reference
end

% --- REPORT TIMESTAMPS ---
fprintf('\n--- SNAPSHOT TIMESTAMPS ---\n');
for k = 1:3
    fprintf('Snapshot %d: %.4f seconds\n', k, time_plot(snapshotIndices(k)));
end
fprintf('---------------------------\n');




% ========= SAVE FIGURES (PAPER NAMES) =========
saveDir = fullfile(pwd, 'plots');
if ~exist(saveDir, 'dir')
    mkdir(saveDir);
end

% Figure 2 – Heading tracking
exportgraphics(figure(2), fullfile(saveDir,'headingtracking.png'), 'Resolution',300);
exportgraphics(figure(2), fullfile(saveDir,'headingtracking.eps'), 'ContentType','vector');

% Figure 3 – 2D trajectories
exportgraphics(figure(3), fullfile(saveDir,'2dtrajectories.png'), 'Resolution',300);
exportgraphics(figure(3), fullfile(saveDir,'2dtrajectories.eps'), 'ContentType','vector');

% Figure 4 – Trajectory error
exportgraphics(figure(4), fullfile(saveDir,'trajectoryerror.png'), 'Resolution',300);
exportgraphics(figure(4), fullfile(saveDir,'trajectoryerror.eps'), 'ContentType','vector');

% Figure 5 – 1D trajectories (X–Y vs time)
exportgraphics(figure(5), fullfile(saveDir,'1dtrajectories.png'), 'Resolution',300);
exportgraphics(figure(5), fullfile(saveDir,'1dtrajectories.eps'), 'ContentType','vector');

% Figure 6 – Sequential simulation snapshots
exportgraphics(figure(6), fullfile(saveDir,'sequentional_simulation.png'), 'Resolution',300);
exportgraphics(figure(6), fullfile(saveDir,'sequentional_simulation.eps'), 'ContentType','vector');


% 1. Create Headers dynamically based on numRobots
csvHeaders = {'Snapshot_Index', 'Time_s', 'Queen_X', 'Queen_Y', 'Queen_Psi_deg'};
for i = 1:numRobots
    csvHeaders = [csvHeaders, ...
                  {sprintf('Agent%d_X', i), sprintf('Agent%d_Y', i), sprintf('Agent%d_Psi_deg', i)}];
end

% 2. Extract Data
dataMatrix = [];

for k = 1:length(snapshotIndices)
    idx = snapshotIndices(k);
    
    % --- Get Time and Queen Data ---
    t_val = time_plot(idx);
    qX_val = qX_plot(idx);
    qY_val = qY_plot(idx);
    qPsi_val = psiHistory(idx, 2); % Matches your Figure 6 logic
    
    % Start the row
    row = [idx, t_val, qX_val, qY_val, qPsi_val];
    
    % --- Get Agent Data ---
    for i = 1:numRobots
        rX_val = robotPaths{i}(idx, 1);
        rY_val = robotPaths{i}(idx, 2);
        rPsi_val = psiHistory(idx, 2+i); % Matches your Figure 6 logic
        
        row = [row, rX_val, rY_val, rPsi_val];
    end
    
    % Append to matrix
    dataMatrix = [dataMatrix; row];
end

% 3. Convert to Table and Save
T = array2table(dataMatrix, 'VariableNames', csvHeaders);
csvFileName = fullfile(saveDir, 'snapshot_data.csv');
writetable(T, csvFileName);

fprintf('Snapshot data saved to: %s\n', csvFileName);
