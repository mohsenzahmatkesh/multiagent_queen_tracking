clc
clear
close all

numRobots = 6;

% --- DATA LOADING ---
% Ensure bee_pose.txt exists in your folder
beeData = readmatrix('bee_pose.txt');

% Using the first 1000 points
dataLen = 1000; 
queen_time = beeData(1:dataLen,1);
queenX = beeData(1:dataLen,2)*1000;
queenY = beeData(1:dataLen,3)*1000;

% --- MAP TO -180 TO 180 DEGREES ---
rawPsi = beeData(1:dataLen,4)*180/pi;
queen_psi = mod(rawPsi + 180, 360) - 180;

% --- Circular initial formation ---
centerX = queenX(1);      
centerY = queenY(1);      
radius = 10;       
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

% SPEED SETTINGS
TargetSpeed = 0.0005;     
stepSize = 1;

% Record robot paths
robotPaths = cell(numRobots, 1);
for i = 1:numRobots
    robotPaths{i} = robotPositions(i, :);
end

% Record Angle Data
psiHistory = [];

%% Hyperparameter settings
Kaat = 0.5;     
Krep = 1.5;
P0 = 25;
StepRate = 0.1; 
de = 20;

%% Plotting Initial Setup
figure(1);
hold on

% --- NEW: Initialize the Queen's Trail (Empty for now) ---
queenTrailX = [];
queenTrailY = [];
% 'k-' means black solid line. LineWidth 1.5 makes it visible.
queenPathPlot = plot(nan, nan, 'k-', 'LineWidth', 1.5); 

% Initialize Robots
b = gobjects(1, numRobots);
for i = 1:numRobots
    % Initialize all as Red ('r')
    b(i) = scatter(robotPositions(i,1)', robotPositions(i,2)', 500, 'r', 'filled');
end

% Create the Target Object
tObj = scatter(CurrentTarget(1), CurrentTarget(2), 700, 'k', 'filled'); 

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
        nextPoint = [DesX(i), DesY(i)];
        [Fattx, Fatty] = AttractiveImprove(MyX, nextPoint(1), MyY, nextPoint(2), Kaat, de);
        
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
        
        % Force Overrides
        Fxsum = 0;
        Fysum = 0;
        
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
    pause(0.05); 
    
    % --- TERMINATION CONDITION ---
    if dataIdx >= length(queenX)
        distToTarget = sqrt((robotPositions(:,1) - CurrentTarget(1)).^2 + (robotPositions(:,2) - CurrentTarget(2)).^2);
        if mean(distToTarget) < 15 
             fprintf("End of Data Reached.\n");
             break;
        end
    end
end

% --- PLOT 1: PATHS ---
for i = 1:numRobots
    plot(robotPaths{i}(:,1), robotPaths{i}(:,2), 'r--', 'LineWidth', 0.2);
end
title('Path Planning with Real Data Tracking');

% --- PLOT 2: ANGLE TRACKING ---
figure(2); 
hold on;
plot(psiHistory(:,1), psiHistory(:,2), 'k', 'LineWidth', 1.5, 'DisplayName', 'Queen \psi');
for i = 1:numRobots
    plot(psiHistory(:,1), psiHistory(:,2+i), '--', 'LineWidth', 2, 'DisplayName', sprintf('Robot %d', i));
end
xlabel('Time (s)');
ylabel('Heading Angle (deg)');
ylim([-180, 180]); 
yticks(-180:60:180);
title('Orientation Tracking: Robots following Queen \psi in Sector');
legend('show');
hold off;

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