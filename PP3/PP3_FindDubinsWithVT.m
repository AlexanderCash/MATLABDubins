function PP3_FindDubinsWithVT
%PP3_FindDubinsWithVT; the third stage of the path planning work
%   PathPlanning3; find a suitable air relative Dubins path that will fly
%   the UAV to a ground relative point d. 

%   We start at a ground relative point q0 
%   We want to end up at a ground relative point d
%   We get to d by commanding the UAV to fly a dubins path to an air
%   relative point denoted by the virtual target point vt

%   Calculation starts with the UAV at point q0
%   At time 0, vt = d
%   As time progresses, vt moves equal and opposite to wind vector

%   Tvt is time taken for vt to move to its location, i.e. the calculation
%   time
%   Ta is time taken for UAV to fly dubins path to point vt
%   Solution is found when Tvt roughly equals Ta

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

% Virtual target vector is equal and opposite to wind vector so
vtVector = -windVector;

%% Define relevant locations

% Create q0 as usual
% d is the value we used to use for q1
q0 = [xStart,yStart,startOrientation];
d = [xEnd,yEnd,endOrientation];

% At time 0, vt = d
vt = d;

%% Calculate start conditions

% Ta starts as the time taken for the UAV to travel from q0 to d as if
% there was no wind, so
noWindPath = dubins(q0,d,radius,stepSize);

% Calculate Ta using the stepsize (distance between each datapoint
% generated by dubins function), and speed of uav.
% Number of x position readings * stepsize = length of path
Ta =  (numel(noWindPath(1,:))*stepSize)/uavSpeed; 

% Calculation time is 0 when turn starts
% At time 0 UAV is at q0
% At time 0 vt = d, therefore Tvt = 0  too;
calculationTime = 0;
Tvt = calculationTime;


%% Search algorithm

while ((Ta-Tvt)>0.1)||((Ta-Tvt)<(-0.1))
    % Firstly increment our calculation time
    calculationTime = calculationTime + 0.1;
    
    % Then update the location of the vt. vt(1) is the x co-ordinate value,
    % this works because wind is always in only the x direction
    % Note: *0.1 because we are evaluating this 10 times for every second
    % of calculationTime
    vt(1) = vt(1) + 0.1*vtVector;
    
    % Tvt is the time it has taken for the vt to move from d to its current
    % location, so is equal to calculationTime
    Tvt = calculationTime;
    
    % Create a candidate dubins path, to see if this position of vt is a
    % suitable solution for the search
    candidatePath = dubins(q0,vt,radius,stepSize);
    
    % Recalculate Ta using the new candidatePath
    Ta = (numel(candidatePath(1,:))*stepSize)/uavSpeed;
end

%% Call plot function

% Simply call the PP3_PlotSolvedDubins helper function to print the results

PP3_PlotSolvedDubins(q0,vt,radius,windVector,uavSpeed);
end

















