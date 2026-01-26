clc
clear
close all

numRobots = 6;

% --- DATA LOADING ---
% Ensure bee_pose.txt exists in your folder
beeData = readmatrix('bee_pose.txt');
beeData2 = readtable('queen_position_extracted.csv');

% Using the first 1000 points
dataLen = 9900; 
queen_time = beeData2(1:dataLen,1);
queen_time = queen_time - queen_time(1);
queenX = beeData2(1:dataLen,3)*1000;
queenY = beeData2(1:dataLen,4)*1000;

% --- MAP TO -180 TO 180 DEGREES ---
rawPsi = beeData2(1:dataLen,2)*180/pi;
queen_psi = mod(rawPsi + 180, 360) - 180;

% --- Circular initial formation ---
centerX = queenX(1);      
centerY = queenY(1);      
radius = 20;       
theta = linspace(0, 2*pi, numRobots+1);
theta(end) = [];   
robotPositions = [centerX + radius*cos(theta)', centerY + radius*sin(theta)'];

% --- ROBOT ANGLE SETUP ---
rawFormation = theta * (180/pi);
robotFormationAngles = mod(rawFormation + 180, 360) - 180; 
robotPsi = robotFormationAngles'; 

% --- DYNAMIC TARGET SETUP ---
CurrentTarget = [queenX(1), queenY(1)];
TargetEnd = [queenX(end), queenY(end)]; 


% Record robot paths
robotPaths = cell(numRobots, 1);
for i = 1:numRobots
    robotPaths{i} = robotPositions(i, :);
end

% Record Angle Data
psiHistory = [];

%% Hyperparameter settings
Kaat = 0.3;     
Krep = 1.5;
P0 = 25;
StepRate = 0.2; 
de = 20;
QueenSafeRadius = 5;
%% Plotting Initial Setup
figure(1);
hold on

queenTrailX = [];
queenTrailY = [];
queenPathPlot = plot(nan, nan, 'k-', 'LineWidth', 1.5); 

% Initialize Robots
b = gobjects(1, numRobots);
for i = 1:numRobots
    % Initialize all as Red ('r')
    b(i) = scatter(robotPositions(i,1)', robotPositions(i,2)', 300, 'r', 'filled');
end

% Create the Target Object
tObj = scatter(CurrentTarget(1), CurrentTarget(2), 500, 'k', 'filled'); 

xlabel("X");
y = ylabel("Y");
set(y, 'Rotation', 0);
axis equal
xlim([min(queenX)-20, max(queenX)+20]);
ylim([min(queenY)-20, max(queenY)+20]);
title('Simulation View');

%% Calculation Loop
dataIdx = 1; 

while true
    tic;
    
    % --- UPDATE TARGET STATE ---
    targetMoveVec = [0, 0];
    
    if dataIdx < length(queenX)
        oldTarget = CurrentTarget;
        dataIdx = dataIdx + 1;
        
        CurrentTarget = [queenX(dataIdx), queenY(dataIdx)];
        CurrentPsi = queen_psi(dataIdx); 
        
        targetMoveVec = CurrentTarget - oldTarget; 
    else
        targetMoveVec = [0, 0];
        CurrentPsi = queen_psi(end);
    end
    
    % Update Target Plot (Circle)
    set(tObj, 'XData', CurrentTarget(1), 'YData', CurrentTarget(2));
    
    % --- NEW: Update Queen's Path Line ---
    queenTrailX = [queenTrailX, CurrentTarget(1)];
    queenTrailY = [queenTrailY, CurrentTarget(2)];
    set(queenPathPlot, 'XData', queenTrailX, 'YData', queenTrailY);
    
    % Update the Target arrays for APF
    DesX = repmat(CurrentTarget(1), numRobots, 1);
    DesY = repmat(CurrentTarget(2), numRobots, 1);
    
    % --- ROBOT MOVEMENT & ANGLE CALCULATION ---
    for i = 1:numRobots
        MyX = robotPositions(i,1);
        MyY = robotPositions(i,2);
        
        % 1. POSITION LOGIC
        % nextPoint = [DesX(i), DesY(i)];
        % [Fattx, Fatty] = AttractiveImprove(MyX, nextPoint(1), MyY, nextPoint(2), Kaat, de);

        nextPoint = [DesX(i), DesY(i)];
        
        % Calculate distance to the Queen (Target)
        dVectorX = nextPoint(1) - MyX;
        dVectorY = nextPoint(2) - MyY;
        distToQueen = sqrt(dVectorX^2 + dVectorY^2);
        
        if distToQueen < QueenSafeRadius
            % --- TOO CLOSE: REPEL ---
            % If inside the boundary, push AWAY from the queen.
            % We normalize the vector and multiply by a repulsion strength (e.g., 2.0)
            repulsionStrength = 2.0; 
            Fattx = -repulsionStrength * (dVectorX / distToQueen);
            Fatty = -repulsionStrength * (dVectorY / distToQueen);
        else
            % --- SAFE DISTANCE: ATTRACT ---
            % If outside the boundary, use your normal attraction logic
            [Fattx, Fatty] = AttractiveImprove(MyX, nextPoint(1), MyY, nextPoint(2), Kaat, de);
        end
        


        
        tempObs = [];
        for k = 1:numRobots
            if k ~= i
                tempObs = [tempObs; robotPositions(k, :)];
            end
        end
        
        Frepx = zeros(1, size(tempObs, 1));
        Frepy = zeros(1, size(tempObs, 1));
        for j = 1:size(tempObs, 1)
            [Frepx(1, j), Frepy(1, j)] = RepulsiveImprove(MyX, MyY, tempObs(j, 1), tempObs(j, 2), nextPoint(1), nextPoint(2), Krep, P0);
        end
        
        Fxsum = Fattx + sum(Frepx);
        Fysum = Fatty + sum(Frepy);
       

        Fxsum = Fxsum + (rand - 0.5) * 0.1;
        Fysum = Fysum + (rand - 0.5) * 0.1;
        
        MyX = MyX + (StepRate * Fxsum) + targetMoveVec(1);
        MyY = MyY + (StepRate * Fysum) + targetMoveVec(2);
        
        robotPositions(i,:) = [MyX, MyY];
        robotPaths{i} = [robotPaths{i}; MyX, MyY];
        
        % 2. ANGLE & COLOR LOGIC
        angleDiff = mod(CurrentPsi - robotFormationAngles(i) + 180, 360) - 180;
        
        % Check if robot is in the "Active" sector
        if abs(angleDiff) <= 40
            % ACTIVE: Follow Queen & Turn YELLOW
            targetAngle = CurrentPsi;
            set(b(i), 'CData', [1, 1, 0]); % [R, G, B] Yellow = [1, 1, 0]
        else
            % INACTIVE: Maintain Formation & Turn RED
            targetAngle = robotFormationAngles(i);
            set(b(i), 'CData', [1, 0, 0]); % [R, G, B] Red = [1, 0, 0]
        end
        
        % Move Robot Psi
        moveError = mod(targetAngle - robotPsi(i) + 180, 360) - 180;
        robotPsi(i) = robotPsi(i) + 0.2 * moveError;
        robotPsi(i) = mod(robotPsi(i) + 180, 360) - 180;
        
        if isvalid(b(i))
            set(b(i), 'XData', MyX, 'YData', MyY);
        end
    end
    
    % Store Data
    psiHistory = [psiHistory; queen_time(dataIdx), CurrentPsi, robotPsi'];
    
    drawnow;
    % pause(0.05); 
    
    % --- TERMINATION CONDITION ---
    if dataIdx >= length(queenX)
        distToTarget = sqrt((robotPositions(:,1) - CurrentTarget(1)).^2 + (robotPositions(:,2) - CurrentTarget(2)).^2);
        if mean(distToTarget) < 15 
             fprintf("End of Data Reached.\n");
             break;
        end
    end
end



% --- Helper functions ---
function [Fx, Fy] = AttractiveImprove(x, xdes, y, ydes, Kaat, de)
    dx = xdes - x; dy = ydes - y;
    dist = sqrt(dx^2 + dy^2);
    Fx = Kaat * dx / (dist + de);
    Fy = Kaat * dy / (dist + de);
end
function [Fx, Fy] = RepulsiveImprove(x, y, ox, oy, xdes, ydes, Krep, P0)
    d = sqrt((x - ox)^2 + (y - oy)^2);
    if d < P0
        Fx = Krep * (1/d - 1/P0) * (1/d^2) * (x - ox);
        Fy = Krep * (1/d - 1/P0) * (1/d^2) * (y - oy);
    else
        Fx = 0; Fy = 0;
    end
end



% % ==========================================================
% %      ADD/REPLACE THIS SECTION AT THE END OF YOUR CODE
% % ==========================================================
% 
% % --- PLOT 1: PATHS ---
% for i = 1:numRobots
%     plot(robotPaths{i}(:,1), robotPaths{i}(:,2), 'r--', 'LineWidth', 0.2);
% end
% % title('Path Planning with Real Data Tracking');
% 
% % --- PLOT 2: ANGLE TRACKING ---
% figure(2); 
% hold on;
% colors = lines(numRobots);
% plot(psiHistory(:,1), psiHistory(:,2), 'k', 'LineWidth', 1.5, 'DisplayName', 'Queen \psi');
% for i = 1:numRobots
%     plot(psiHistory(:,1), psiHistory(:,2+i), '--', 'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', sprintf('Robot %d', i));
% end
% xlabel('Time (s)');
% ylabel('Heading Angle (deg)');
% ylim([-180, 180]); 
% yticks(-180:60:180);
% % title('Orientation Tracking: Robots following Queen \psi in Sector');
% legend('show', 'Location', 'northeast', 'NumColumns', 4);
% hold off;
% 
% % --- PREPARE DATA ---
% simSteps = size(robotPaths{1}, 1); 
% qX_plot = queenX(1:simSteps);
% qY_plot = queenY(1:simSteps);
% time_plot = queen_time(1:simSteps);
% colors = lines(numRobots); 
% % Standard Figure Size [left, bottom, width, height]
% figSize = [100, 100, 600, 400];
% 
% % --- PLOT 3: 2D TRAJECTORIES (Map View) ---
% figure(3);
% set(gcf, 'Position', figSize);
% hold on; 
% plot(qX_plot, qY_plot, 'k-', 'LineWidth', 0.5, 'DisplayName', 'Queen');
% for i = 1:numRobots
%     plot(robotPaths{i}(:,1), robotPaths{i}(:,2), '--', 'Color', colors(i,:), 'LineWidth', 0.2, 'DisplayName', sprintf('Robot %d', i));
% end
% xlabel('X Position (mm)');
% ylabel('Y Position (mm)');
% % title('2D Trajectories');
% % 'NumColumns', 2 -> Creates 2 columns (2 items per row)
% legend('show', 'Location', 'northeast'); 
% axis equal;
% 
% % --- PLOT 4: FORMATION ERROR ---
% figure(4);
% set(gcf, 'Position', figSize);
% hold on; 
% for i = 1:numRobots
%     distError = sqrt((robotPaths{i}(:,1) - qX_plot).^2 + (robotPaths{i}(:,2) - qY_plot).^2);
%     plot(time_plot, distError, 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Robot %d', i));
% end
% xlabel('Time (s)');
% ylabel('Distance to Queen (mm)');
% % title('Formation Error');
% legend('show', 'Location', 'northeast', 'NumColumns', 2);
% 
% % --- PLOT 5: X & Y POSITIONS (Subplots) ---
% figure(5);
% set(gcf, 'Position', [100, 100, 600, 600]); 
% % Subplot 1: X Positions
% subplot(2,1,1);
% hold on; 
% plot(time_plot, qX_plot, 'k-', 'LineWidth', 2, 'DisplayName', 'Queen X');
% for i = 1:numRobots
%     plot(time_plot, robotPaths{i}(:,1), '--', 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Robot %d', i));
% end
% ylabel('X Position (mm)');
% % title('X Position Tracking');
% legend('show', 'Location', 'northeast', 'NumColumns', 2);
% 
% % Subplot 2: Y Positions
% subplot(2,1,2);
% hold on; 
% plot(time_plot, qY_plot, 'k-', 'LineWidth', 2, 'DisplayName', 'Queen Y');
% for i = 1:numRobots
%     plot(time_plot, robotPaths{i}(:,2), '--', 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Robot %d', i));
% end
% xlabel('Time (s)');
% ylabel('Y Position (mm)');
% % title('Y Position Tracking');
% % legend('show', 'Location', 'northeast', 'NumColumns', 2);
% 
% % ==========================================================
% %      ADD/REPLACE THIS BLOCK FOR FIGURE 6
% % ==========================================================
% % --- PLOT 6: SNAPSHOTS WITH ORIENTATION ARROWS ---
% figure(6);
% set(gcf, 'Position', [100, 100, 600, 800]); 
% 
% % --- SETTINGS FOR "MUCH BIGGER" VISUALS ---
% arrowLen = 15;        % Set to 15 for visibility
% queenSize = 400;      
% robotSize = 250;      
% lineWidth = 2.5;      
% 
% % --- SUBPLOT 1: TIME STEP 10 ---
% subplot(2,1,1);
% hold on; grid on; box on;
% axis equal;
% 
% % 1. Determine Index 
% idx = min(10, simSteps);
% 
% % 2. Plot Queen Arrow (Outbound)
% qX = qX_plot(idx);
% qY = qY_plot(idx);
% qPsi = psiHistory(idx, 2); 
% u = arrowLen * cosd(qPsi);
% v = arrowLen * sind(qPsi);
% % Outbound: Start at (qX, qY)
% quiver(qX, qY, u, v, 0, 'k', 'LineWidth', lineWidth, 'MaxHeadSize', 0.6, 'DisplayName', 'Queen');
% scatter(qX, qY, queenSize, 'k', 'filled', 'HandleVisibility', 'off'); 
% 
% % 3. Plot Robot Arrows (Outbound)
% for i = 1:numRobots
%     rX = robotPaths{i}(idx, 1);
%     rY = robotPaths{i}(idx, 2);
%     rPsi = psiHistory(idx, 2+i); 
% 
%     u = arrowLen * cosd(rPsi);
%     v = arrowLen * sind(rPsi);
% 
%     quiver(rX, rY, u, v, 0, 'Color', colors(i,:), 'LineWidth', lineWidth, 'MaxHeadSize', 0.6, 'DisplayName', sprintf('Robot %d', i));
%     scatter(rX, rY, robotSize, colors(i,:), 'filled', 'HandleVisibility', 'off'); 
% end
% 
% xlabel('X Position (mm)');
% ylabel('Y Position (mm)');
% title(sprintf('Snapshot at Time Step %d (t = %.2fs)', idx, time_plot(idx)));
% legend('show', 'Location', 'northeast', 'NumColumns', 2);
% 
% % --- SUBPLOT 2: FINAL TIME STEP ---
% subplot(2,1,2);
% hold on; grid on; box on;
% axis equal;
% 
% % 1. Determine Index (Fixed at 999)
% idx = 999; 
% 
% % 2. Plot Queen Arrow (Outbound)
% qX = qX_plot(idx);
% qY = qY_plot(idx);
% qPsi = psiHistory(idx, 2); 
% u = arrowLen * cosd(qPsi);
% v = arrowLen * sind(qPsi);
% quiver(qX, qY, u, v, 0, 'k', 'LineWidth', lineWidth, 'MaxHeadSize', 0.6, 'DisplayName', 'Queen');
% scatter(qX, qY, queenSize, 'k', 'filled', 'HandleVisibility', 'off');
% 
% % 3. Plot Robot Arrows (Outbound)
% for i = 1:numRobots
%     rX = robotPaths{i}(idx, 1);
%     rY = robotPaths{i}(idx, 2);
%     rPsi = psiHistory(idx, 2+i); 
% 
%     u = arrowLen * cosd(rPsi);
%     v = arrowLen * sind(rPsi);
% 
%     quiver(rX, rY, u, v, 0, 'Color', colors(i,:), 'LineWidth', lineWidth, 'MaxHeadSize', 0.6, 'DisplayName', sprintf('Robot %d', i));
%     scatter(rX, rY, robotSize, colors(i,:), 'filled', 'HandleVisibility', 'off');
% end
% 
% xlabel('X Position (mm)');
% ylabel('Y Position (mm)');
% title(sprintf('Snapshot at Final Step (t = %.2fs)', time_plot(idx)));
% legend('show', 'Location', 'northeast', 'NumColumns', 2);