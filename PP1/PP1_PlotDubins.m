function PP1_PlotDubins
%PP1_PlotDubins function; the first stage of path planning work
%   PathPlanning1; use the mex wrapper to just plot a Dubins path. Set
%   values for start and endpoints within function, parameters can be
%   annoying to enter as args. Copying in info from dubins_interface.m:

    %[ path ] = dubins(q0, q1, r, stepSize)
    %   dubins calculates an interpolated dubins cureve between two points
    %   q0 and q1 with circles of radius r at the inputed stepSize.
    %
    %   q0 = [x1 (1x1), y1 (1x1), theta1 (1x1)]; 
    %   q1 = [x2 (1x1), y2 (1x1), theta2 (1x1)]; 
    %   r  = (1x1); stepSize = (1x1);
    %
    %   path = [x;y;theta] (3xn) :  n is deteremined within the code but is
    %   a minimum of 1
    %
    %   The original code is from
    %   https://github.com/AndrewWalker/Dubins-Curves

% We need to plot just the x and y values

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

%% Calculate path

% Create q0 and q1 points from input params
q0 = [xStart,yStart,startOrientation];
q1 = [xEnd,yEnd,endOrientation];

% Call dubins to generate path
dubinsPath = dubins(q0,q1,radius,stepSize);

% Plot path x and y vals
plot(dubinsPath(1,:), dubinsPath(2,:))

end

