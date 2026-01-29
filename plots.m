clc
close all

% ==========================================================
% 1. PREPARE DATA (MUST RUN FIRST)
% ==========================================================
if ~exist('robotPaths', 'var') || ~exist('queenX', 'var')
    error('Error: Simulation data (robotPaths, queenX, etc.) is missing from workspace.');
end

simSteps = size(robotPaths{1}, 1); 
qX_plot = queenX(1:simSteps);
qY_plot = queenY(1:simSteps);
time_plot = queen_time(1:simSteps);
colors = lines(numRobots); 

% Standard Size (Rectangular) for Figs 2 & 4
figSizeStandard = [100, 100, 600, 400]; 

% ==========================================================
% 2. GENERATE PLOTS
% ==========================================================

%% --- PLOT 2: ANGLE TRACKING ---
figure(2); 
set(gcf, 'Position', figSizeStandard); 
hold on; box on

% Set Border to Black/Thick
set(gca, 'LineWidth', 1, 'FontSize', 10); 

% --- 1. PLOT QUEEN (With Jump Removal) ---
qPsi = psiHistory(:,2);
jumpThreshold = 300; 
qPsi(abs([0; diff(qPsi)]) > jumpThreshold) = NaN; 
plot(psiHistory(:,1), qPsi, 'k', 'LineWidth', 2, 'DisplayName', 'Queen \psi');

% --- 2. PLOT AGENTS (With Jump Removal) ---
for i = 1:numRobots
    aPsi = psiHistory(:,2+i);
    aPsi(abs([0; diff(aPsi)]) > jumpThreshold) = NaN;
    plot(psiHistory(:,1), aPsi, 'Color', colors(i,:), 'LineWidth', 1.7, ...
        'DisplayName', sprintf('Agent %d', i));
end

xlabel('Time (s)');
ylabel('Heading Angle (deg)');
ylim([-180, 180]); 
xlim([0, 107.764537600000]); 
yticks(-180:60:180);

lgd = legend('show', 'Location', 'northeast', 'NumColumns', 4);
lgd.FontSize = 8;        
lgd.ItemTokenSize = [10 8];
hold off;

%% --- PLOT 3: 2D TRAJECTORIES (Map View - Evolution) ---
figure(3);
set(gcf, 'Position', [100, 100, 800, 600]); % Large Figure

% 1. CALCULATE GLOBAL LIMITS 
all_X = qX_plot;
all_Y = qY_plot;
for i = 1:numRobots
    all_X = [all_X; robotPaths{i}(:,1)];
    all_Y = [all_Y; robotPaths{i}(:,2)];
end
globalXLim = [min(all_X)-5, max(all_X)+5];
globalYLim = [min(all_Y)-5, max(all_Y)+5];

% 2. DEFINE STAGES
percentages = [0.25, 0.50, 0.75, 1.00];
titles = {'25% Evolution', '50% Evolution', '75% Evolution', '100% Evolution'};

% 3. CREATE TILED LAYOUT
tlo = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for k = 1:4
    nexttile;
    hold on; box on;
    axis equal;
    
    % Set Border to Black/Thick & Larger Font
    set(gca, 'LineWidth', 1, 'FontSize', 12); 
    
    % Determine the end index for this percentage
    limitIdx = floor(percentages(k) * simSteps);
    limitIdx = max(1, limitIdx); 
    
    % --- Plot Queen Trajectory ---
    plot(qX_plot(1:limitIdx), qY_plot(1:limitIdx), 'k-', ...
        'LineWidth', 2.0, 'DisplayName', 'Queen');
    
    % --- Plot Queen Current Point ---
    plot(qX_plot(limitIdx), qY_plot(limitIdx), 'ko', ...
        'MarkerFaceColor', 'k', 'MarkerSize', 5, 'HandleVisibility', 'off');

    % --- Plot Robots ---
    for i = 1:numRobots
        % Trajectory
        plot(robotPaths{i}(1:limitIdx, 1), robotPaths{i}(1:limitIdx, 2), ...
            'Color', colors(i,:), 'LineWidth', 1.2, 'DisplayName', sprintf('Agent %d', i));
        % Current Point
        plot(robotPaths{i}(limitIdx, 1), robotPaths{i}(limitIdx, 2), 'o', ...
            'Color', colors(i,:), 'MarkerFaceColor', colors(i,:), ...
            'MarkerSize', 5, 'HandleVisibility', 'off');
    end
    
    % --- Styling ---
    xlim(globalXLim);
    ylim(globalYLim);
    title(titles{k});
    
    % --- CONDITIONAL LABELS ---
    if mod(k, 2) ~= 0 
        ylabel('Y Position (cm)');
    end
    if k > 2
        xlabel('X Position (cm)');
    end
    
    % --- Legend Logic (First Plot Only) ---
    if k == 1
        lgd = legend('show', 'Location', 'southeast');
        lgd.FontSize = 10; 
        lgd.ItemTokenSize = [12 10];
    end
end

%% --- PLOT 4: FORMATION ERROR ---
figure(4);
set(gcf, 'Position', figSizeStandard); 
hold on; box on

% Set Border to Black/Thick
set(gca, 'LineWidth', 1, 'FontSize', 10);

for i = 1:numRobots
    distError = sqrt((robotPaths{i}(:,1) - qX_plot).^2 + (robotPaths{i}(:,2) - qY_plot).^2);
    plot(time_plot, distError, 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Agent %d', i));
end
xlabel('Time (s)');
ylabel('Euclidean Distance to Queen (cm)');
xlim([0, 107.764537600000]);
lgd = legend('show', 'Location', 'northeast', 'NumColumns', 4);
lgd.FontSize = 8;        
lgd.ItemTokenSize = [10 8];

%% --- PLOT 5: X & Y POSITIONS (Subplots) ---
figure(5);
set(gcf, 'Position', [100, 100, 600, 500]); 

% Subplot 1: X Positions
subplot(2,1,1);
hold on; box on
set(gca, 'LineWidth', 1, 'FontSize', 10); % Set Border to Black

plot(time_plot, qX_plot, 'k-', 'LineWidth', 2, 'DisplayName', 'Queen X');
for i = 1:numRobots
    plot(time_plot, robotPaths{i}(:,1), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Agent %d', i));
end
ylabel('X Position (cm)');
xlim([0, 107.764537600000]);
lgd = legend('show', 'Location', 'northeast', 'NumColumns', 4);
lgd.FontSize = 8;        
lgd.ItemTokenSize = [10 8];

% Subplot 2: Y Positions
subplot(2,1,2);
hold on; box on
set(gca, 'LineWidth', 1, 'FontSize', 10); % Set Border to Black

plot(time_plot, qY_plot, 'k-', 'LineWidth', 2, 'DisplayName', 'Queen Y');
for i = 1:numRobots
    plot(time_plot, robotPaths{i}(:,2), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Agent %d', i));
end
xlabel('Time (s)');
ylabel('Y Position (cm)');
xlim([0, 107.764537600000]);

%% --- PLOT 6: SNAPSHOTS WITH OVALS (HORIZONTAL LAYOUT) ---
figure(6);
set(gcf, 'Position', [100, 100, 1200, 400]); 
% --- Settings ---
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
    
    % Set Border to Black/Thick
    set(gca, 'FontSize', 10, 'LineWidth', 1); 
    
    idx = snapshotIndices(k);
    
    % --- PLOT QUEEN TRAJECTORY (History up to idx) ---
    plot(qX_plot(1:idx), qY_plot(1:idx), 'k-', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    
    % --- PLOT QUEEN (Black Oval) ---
    qX = qX_plot(idx);
    qY = qY_plot(idx);
    qPsi = psiHistory(idx, 2); 
    
    X_ell = q_a * cos(t_param);
    Y_ell = q_b * sin(t_param);
    R = [cosd(qPsi), -sind(qPsi); sind(qPsi), cosd(qPsi)];
    coords = R * [X_ell; Y_ell];
    
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
        
        X_ell = r_a * cos(t_param);
        Y_ell = r_b * sin(t_param);
        R = [cosd(rPsi), -sind(rPsi); sind(rPsi), cosd(rPsi)];
        coords = R * [X_ell; Y_ell];
        
        if isHeadOn
            agentColor = [1, 0.84, 0]; % Gold
            legendName = sprintf('Head-on (Pos: %.0f^{\\circ})', robotFormationAngles(i));
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
    
    if k == 1
        ylabel('Y Position (cm)');
    end
    
    lgd = legend('show', 'Location', 'northeast');
    lgd.FontSize = 8;          
    lgd.ItemTokenSize = [10 8]; 
end

fprintf('\n--- SNAPSHOT TIMESTAMPS ---\n');
for k = 1:3
    fprintf('Snapshot %d: %.4f seconds\n', k, time_plot(snapshotIndices(k)));
end
fprintf('---------------------------\n');

% ==========================================================
% 3. SAVE FIGURES AND DATA
% ==========================================================
saveDir = fullfile(pwd, 'plots');
if ~exist(saveDir, 'dir')
    mkdir(saveDir);
end

% Save Images
exportgraphics(figure(2), fullfile(saveDir,'headingtracking.png'), 'Resolution',300);
exportgraphics(figure(2), fullfile(saveDir,'headingtracking.eps'), 'ContentType','vector');
exportgraphics(figure(3), fullfile(saveDir,'2dtrajectories.png'), 'Resolution',300);
exportgraphics(figure(3), fullfile(saveDir,'2dtrajectories.eps'), 'ContentType','vector');
exportgraphics(figure(4), fullfile(saveDir,'trajectoryerror.png'), 'Resolution',300);
exportgraphics(figure(4), fullfile(saveDir,'trajectoryerror.eps'), 'ContentType','vector');
exportgraphics(figure(5), fullfile(saveDir,'1dtrajectories.png'), 'Resolution',300);
exportgraphics(figure(5), fullfile(saveDir,'1dtrajectories.eps'), 'ContentType','vector');
exportgraphics(figure(6), fullfile(saveDir,'sequentional_simulation.png'), 'Resolution',300);
exportgraphics(figure(6), fullfile(saveDir,'sequentional_simulation.eps'), 'ContentType','vector');

% Save Data to CSV
csvHeaders = {'Snapshot_Index', 'Time_s', 'Queen_X', 'Queen_Y', 'Queen_Psi_deg'};
for i = 1:numRobots
    csvHeaders = [csvHeaders, ...
                  {sprintf('Agent%d_X', i), sprintf('Agent%d_Y', i), sprintf('Agent%d_Psi_deg', i)}];
end

dataMatrix = [];
for k = 1:length(snapshotIndices)
    idx = snapshotIndices(k);
    
    t_val = time_plot(idx);
    qX_val = qX_plot(idx);
    qY_val = qY_plot(idx);
    qPsi_val = psiHistory(idx, 2); 
    
    row = [idx, t_val, qX_val, qY_val, qPsi_val];
    
    for i = 1:numRobots
        rX_val = robotPaths{i}(idx, 1);
        rY_val = robotPaths{i}(idx, 2);
        rPsi_val = psiHistory(idx, 2+i); 
        
        row = [row, rX_val, rY_val, rPsi_val];
    end
    dataMatrix = [dataMatrix; row];
end

T = array2table(dataMatrix, 'VariableNames', csvHeaders);
csvFileName = fullfile(saveDir, 'snapshot_data.csv');
writetable(T, csvFileName);
fprintf('Snapshot data saved to: %s\n', csvFileName);