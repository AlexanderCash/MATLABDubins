function PP3_PlotSolvedDubins(q0,vt,radius,windVector,uavSpeed)
%PP3_PlotSolvedDubins; utility function for PP3_FindDubinsWithVT
% Copies the functionality of PP2_PlotDubinsAndWind

%% Input handling
 
% All we need to do is define stepSize
stepSize = 0.1;

%% Calculate no-wind path

% Call dubins to generate no-wind path
dubinsPath = dubins(q0,vt,radius,stepSize);


%% Calculate n

% From dubins_interface.m, output path is (3xn) matrix. We need to know n
% to work out a time val so as to calculate offset by wind.

% Call n 'numReadings'
% Find number of entried in top row to get n
numReadings = numel(dubinsPath(1,:));

%% Calculate timer interval between steps (from stepSize val)

% Time delta between each reading. We use fixed stepSize of 0.1m so to get
% time between each reading:
timeDelta = 1/(uavSpeed/stepSize);
% For uav going 18ms our timeDelta will be 1/180 s

%% Create new windy x values

% Counter variable for loop
counter = 0;

% We need to add time values and new x values for our path. y values are
% unaffected by wind because wind is only in the x direction. Path array
% now becomes:

    %   path =  | no-wind x vals    |
    %           | no-wind y vals    |
    %           | theta (unused)    |
    %           | time value        |
    %           | windy x vals      |

% Loop through all columns
for i = 1:1:numReadings
    % Row 4 is time value, updated last in loop
    dubinsPath(4,i) = counter;
    
    % Calculate new x value from no-wind x value + time value*wind vector
    dubinsPath(5,i) = dubinsPath(1,i) + counter*windVector;
    
    % Update time value using counter variable
    counter = counter + timeDelta;
end

%% Plot paths

% Plot no-wind x against no-wind y as in PP1
plot(dubinsPath(1,:), dubinsPath(2,:))
hold % Hold plot to add second path
% Plot windy x against no-wind y, in different plot colour
plot(dubinsPath(5,:), dubinsPath(2,:),'r')
% Scale the axis as equal so as not to warp the shape
axis equal

%% Add wind indicator

% Add a wind arrow, direction based on sign of wind val
if windVector < 0
    arrowX = [0.5 0.4];
else 
    arrowX = [0.4 0.5];
end

arrowY = [0.2 0.2];

%% Add wind speed value to arrow
arrowString = sprintf('Wind = %dm/s',windVector);
annotation('textarrow',arrowX,arrowY,'String',arrowString)

%% Create legend
legend('Air relative','Groud relative')
hold

%% Print out important path points

fprintf('\n\nOrigin(%.01f, %.01f, %.02f)\n',dubinsPath(1,1),dubinsPath(2,1),q0(3));
fprintf('Airpath End(%.01f, %.01f, %.02f)\n',dubinsPath(1,end),dubinsPath(2,end),vt(3));
fprintf('Groundpath End(%.01f, %.01f, %.02f)\n',dubinsPath(5,end),dubinsPath(2,end),vt(3));
fprintf('Flight duration: %.02f\n',dubinsPath(4,end));
fprintf('x offset should be: %.02f\n',dubinsPath(4,end)*windVector);
fprintf('\tx actual offset: %.02f\n\n',dubinsPath(5,end)-dubinsPath(1,end));

end

