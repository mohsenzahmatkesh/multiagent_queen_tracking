close all

numRobots = 6;

beeData = readmatrix('bee_pose.txt');
bag = rosbag('h0xy_queen_2024-07-03-09-00-02.bag');
% disp(bag.AvailableTopics);
bSel = select(bag, 'Topic', '/queen_position');

% --- Circular initial formation ---
centerX = 20;      % circle center (x)
centerY = 20;      % circle center (y)
radius = 10;       % circle radius
theta = linspace(0, 2*pi, numRobots+1);
theta(end) = [];   % remove duplicate point
robotPositions = [centerX + radius*cos(theta)', centerY + radius*sin(theta)'];


% --- DYNAMIC TARGET SETUP ---
% TargetStart = [15, 55]; 
% TargetEnd = [70, 55];   

TargetStart = [20, 20]; 
TargetEnd = [80, 80];  
CurrentTarget = TargetStart;

% SPEED SETTINGS
% We can now increase TargetSpeed because robots have Feedforward logic
TargetSpeed = 0.05;     

flag = 0;                             
maxIterations = 1000;
stepSize = 1;

% Robot colors
robotColors = ['r', 'r', 'r', 'r', 'r', 'r'];

% Record robot paths
robotPaths = cell(numRobots, 1);
for i = 1:numRobots
    robotPaths{i} = robotPositions(i, :);
end

% Tracking data arrays
timeDistanceData = [];
computationalEfficiencyData = [];

%% Hyperparameter settings
if flag == 1
    Kaat = 0.1;
    Krep = 100;
    P0 = 25;
    StepRate = 0.1;
    Epoch = 3000;
    de = 20;
else
    Kaat = 0.5;     % Increased Attraction slightly for tighter formation
    Krep = 1.5;
    P0 = 25;
    StepRate = 0.1; 
    Epoch = 5000;   
    de = 20;
end

%% Plotting Initial Setup
figure;
hold on
b = gobjects(1, numRobots);
for i = 1:numRobots
    b(i) = scatter(robotPositions(i,1)', robotPositions(i,2)', 100, robotColors(i), 'filled');
end

% Create the Target Object
tObj = scatter(CurrentTarget(1), CurrentTarget(2), 200, 'k', 'filled'); 

xlabel("X");
y = ylabel("Y");
set(y, 'Rotation', 0);
xlim([0, 100]);
ylim([0, 100]);

%% Calculation Loop
CountFlag = 0;
forceThreshold = 0.035;

while true
    tic;
    
    % --- UPDATE TARGET POSITION ---
    % Calculate the vector the target moved in this step
    targetMoveVec = [0, 0];
    
    if CurrentTarget(1) < TargetEnd(1)
        oldTarget = CurrentTarget;
        CurrentTarget = CurrentTarget + TargetSpeed;
        targetMoveVec = CurrentTarget - oldTarget; % This is the "Feedforward" vector
    end
    
    % Update Target Plot
    set(tObj, 'XData', CurrentTarget(1), 'YData', CurrentTarget(2));
    
    % Update the Target arrays
    DesX = repmat(CurrentTarget(1), numRobots, 1);
    DesY = repmat(CurrentTarget(2), numRobots, 1);
    
    % --- ROBOT MOVEMENT CALCULATION ---
    totalForce = zeros(numRobots, 1);
    for i = 1:numRobots
        MyX = robotPositions(i,1);
        MyY = robotPositions(i,2);
        
        % Goal is the CURRENT dynamic target position
        nextPoint = [DesX(i), DesY(i)];
        
        % Attractive Force
        [Fattx, Fatty] = AttractiveImprove(MyX, nextPoint(1), MyY, nextPoint(2), Kaat, de);
        
        % Repulsive Force
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
        
        % Add random disturbance
        Fxsum = Fxsum + (rand - 0.5) * 0.1;
        Fysum = Fysum + (rand - 0.5) * 0.1;
        
        % --- KEY CHANGE: ADD FEEDFORWARD TARGET VELOCITY ---
        % We add 'targetMoveVec' directly to the position.
        % The APF forces (Fxsum) now only handle "Correction" (keeping formation),
        % while targetMoveVec handles "Tracking" (moving forward).
        MyX = MyX + (StepRate * Fxsum) + targetMoveVec(1);
        MyY = MyY + (StepRate * Fysum) + targetMoveVec(2);
        
        robotPositions(i,:) = [MyX, MyY];
        robotPaths{i} = [robotPaths{i}; MyX, MyY];
        
        if isvalid(b(i))
            set(b(i), 'XData', MyX, 'YData', MyY);
        end
        totalForce(i) = sqrt(Fxsum^2 + Fysum^2);
    end
    
    drawnow;
    
    % Check for termination
    CountFlag = CountFlag + 1;
    if CountFlag >= Epoch
        fprintf("Timeout\n");
        break;
    end
    
    % Condition: Stop if target reached destination AND robots are stable
    targetReached = CurrentTarget(1) >= TargetEnd(1);
    distToTarget = sqrt((robotPositions(:,1) - TargetEnd(1)).^2 + (robotPositions(:,2) - TargetEnd(2)).^2);
    
    if targetReached && all(totalForce < forceThreshold) && mean(distToTarget) < 5
        fprintf("Target reached and robots stabilized.\n");
        break;
    end
end

% Draw final paths
for i = 1:numRobots
    plot(robotPaths{i}(:,1), robotPaths{i}(:,2), robotColors(i), 'LineWidth', 2);
end
title('Path Planning with Feedforward Tracking');
saveas(gcf, 'dynamic_target_feedforward.png');
hold off
display(CountFlag);

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