function PP2_PlotDubinsAndWind
%PP2_PlotDubinsAndWind; the second stage of the path planning work
%   PathPlanning2; use path plotted in PP1 with wind input to display a
%   path offset by wind. Normalise inputs to assume wind always blows along
%   the x axis; direction specified by sign. Doing this means we dont need
%   to do any trig with the wind, only to work out initial positions.

%   Offset by wind is basically time*wind speed in that direction

%% Input params
% Locations use co-ords as if they are meter values
xStart = 0;
yStart = 0;

xEnd = 10;
yEnd = 10;

% Orientations are in radians starting at due East increasing counter
% clockwise
    % North = pi/2
    % East = 0
    % South = 3*pi/2
    % West = pi
startOrientation = pi/2; 
endOrientation = 3*pi/2;

% UAV turn radius
radius = 25;

% stepSize is resolution for dubins wrapper, just leave at 0.1 to get
% accurate plots
stepSize = 0.1; 

%% Speed params

% Speed value isnt too important right now, as long as it is reasonable
uavSpeed = 18;

% -ve is wind blowing in the -ve direction along the x axis
% +ve is wind blowing in the +ve direction along the x axis
windVector = -10; 

%% Calculate no-wind path

% Create q0 and q1 points from input params
q0 = [xStart,yStart,startOrientation];
q1 = [xEnd,yEnd,endOrientation];

% Call dubins to generate no-wind path
dubinsPath = dubins(q0,q1,radius,stepSize);


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


end

