% 1. Load the bag and select the specific topic
bag = rosbag('h0xy_queen_2024-07-03-09-00-02.bag');
bSel = select(bag, 'Topic', '/hive_0/xy_0/queen_position');

% 2. Read all messages as structures (this handles the custom message type)
msgs = readMessages(bSel, 'DataFormat', 'struct');
numMsgs = length(msgs);

% 3. Pre-allocate a matrix for speed (Size: N x 3)
% Columns: [X, Y, Phi]
queen_data = nan(numMsgs, 3);

% 4. Loop through and extract the data
for i = 1:numMsgs
    % Check if the 'Positions' array is not empty
    if ~isempty(msgs{i}.Positions)
        % We assume the queen is the 1st element in the Positions array
        queen_data(i, 1) = msgs{i}.Positions(1).X;
        queen_data(i, 2) = msgs{i}.Positions(1).Y;
        queen_data(i, 3) = msgs{i}.Positions(1).Phi;
    end
end

% 5. Remove any rows where data wasn't found (NaNs)
queen_data(any(isnan(queen_data), 2), :) = [];

% % 6. Display or use the data
% disp('First 5 rows of Queen Data (X, Y, Phi):');
% disp(queen_data(1:5, :));
% 
% % Optional: Plot the path
% figure;
% plot(queen_data(:,1), queen_data(:,2));
% title('Queen Position (X vs Y)');
% xlabel('X'); ylabel('Y');
% axis equal;