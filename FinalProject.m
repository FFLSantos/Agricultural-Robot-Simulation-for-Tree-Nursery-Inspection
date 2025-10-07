%% Main code
% Final Project
% Students: Achala Rao & Fernando Ferreira Lima dos Santos
% Professor: Stavros Vougioukas

clear all;
% Defining some variables as global
global DT dT STEER_INPUT SPEED_INPUT MAX_STEER MAX_SPEED MIN_STEER MIN_SPEED;
global ROBOT_POS_X ROBOT_POS_Y ROBOT_THETA ROBOT_STEER ROBOT_SPEED W N RS Rmin;
global previous rangeMax angleSpan angleStep probmap oddsmap sens step_ang;
global bitmap probmap2 nursery;
addpath(genpath('geom2d'));

Xmax = 50; Ymax = 50; % Dimensions of the physical space (m)
R = 500; C = 500; % Numbers of rows and columns of bitmap
bitmap = zeros(R, C); % Initialize as empty
probmap = 0.0*ones(R, C); %initialize the probability map
probmap2 = 0.0*ones(R, C); %initialize the probability map
oddsmap = ones(R, C); %initialize the pixels in Odds map to be equal to one (probability = 50%)

DT = 0.1; % DT = control step interval (s)
dT = 0.01; % dt = Euler integration interval (s)
T = 40; % Time interval for the task (s)
Ld = 2; % Look ahead distance (m)
N = 4; % Number of rows
RL = 12; % Row length
RS = 3; % Row spacing (m)

step_ang = 0.1;
sens = 20;
HUGE = 10^(10);

% Lidar settings
rangeMax = 20; angleSpan = pi; angleStep = deg2rad(0.125); 

%Vehicle Parameters
W = 2; % The width of the vehicle (m)
L = 3; % The wheelbase of the vehicle (m)
Vmax = 5;  %maximum vehicle speed (m/s)
gammaMax = deg2rad(55); 

% Defining the time response (tau) of the system, in seconds.
tau_gamma = 0.1; % Basically, it says how quickly the system will respond to a change in the steering angle
tau_v = 0.2; % Basically, it says how quickly the system will respond to a change in the velocity

% Defining the input variables as constant values
% u(1) -> desired steering angle (rad)
% u(2) -> desired linear velocity (m/s)
u = [0 3]; 
STEER_INPUT = 1; SPEED_INPUT=2;

% Maximum and minimum values for the input variables 
umax = [gammaMax Vmax];
MAX_STEER = 1; MAX_SPEED=2;
umin = - umax;
MIN_STEER = 1; MIN_SPEED=2;

Rmin = L/tan(gammaMax);

% Defining the initial state of the vehicle
% q(1) -> position in x axis
% q(2) -> position in y axis
% q(3) -> theta (orientation in world frame)
% q(4) -> phi (steering angle)
% q(5) -> linear velocity
q = [0 0 pi/2 u(STEER_INPUT) u(SPEED_INPUT)];
ROBOT_POS_X = 1; ROBOT_POS_Y = 2; ROBOT_THETA = 3; ROBOT_STEER = 4; ROBOT_SPEED = 5; 

% Defining the state constraints
Qmax(ROBOT_POS_X) = Inf; Qmax(ROBOT_POS_Y) = Inf; Qmax(ROBOT_THETA) = Inf; 
Qmax(ROBOT_STEER) = umax(MAX_STEER);Qmax(ROBOT_SPEED) = umax(MAX_SPEED);
Qmin = -Qmax; % symmetrical negative constraints for minimum values

%% Computing node coordinates

% This function creates a randon nursery and returns a plot illustrating its real location 
% and the coordinates of the bottom-left tree center (m)
ng = NurseryGen(R,C,Xmax,Ymax);

for i = 2:N+1
    Y(i) = ng(2);
    if i == 2
        X(i) = ng(1)-RS/2;
    else
        X(i) = X(i-1)+RS;
    end
end
for i = N+2:2*N+1
    Y(i) = 32;
    X(i) = X(i-N);
end
% Adding the robot's initial and final coordinates to the vector
X(1) = q(ROBOT_POS_X); Y(1) = q(ROBOT_POS_Y);
X(2*N+2) = q(ROBOT_POS_X); Y(2*N+2) = q(ROBOT_POS_Y);
XY = [X;Y].';

%% Computing cost of nodes in the cost matrix
for i = 2:N+1
    for j = N+2:2*N+1
        if j-i == N
            % The cost to go straight in the row should be minimum
            M_cost(i,j) = -HUGE/2; 
            M_cost(j,i) = -HUGE/2; 
        else
            M_cost(i,j) = HUGE; 
            M_cost(j,i) = HUGE;        
        end
    end
end
%Penalizing diagonal maneuvers
for i = 1:2*N+2
    M_cost(i,i) = HUGE;
end
% Assigning cost to headland turning
for i = 2:N
    for j = i+1:N+1
        d = abs(i-j);
        if (Rmin <= d*RS/2) % Make a pi turn
           Len = d*RS+ ((pi-2)*Rmin); % Length of the turn
           M_cost(i,j) = Len;
        else % Make an omega turn
           gamma = acos(1-(2*Rmin+d*RS)^(2)/(8*Rmin^(2)));
           alpha = (pi - gamma)/2;
           Len = 2*Rmin*alpha + 2*Rmin*pi - gamma*Rmin; % Length of the turn
           M_cost(i,j) = Len;          
        end
        M_cost(i,j) = M_cost(j,i); % Symmetry of cost
        M_cost(i+N,j+N) = M_cost(i,j); % Same cost to top nodes
        M_cost(j+N,i+N) = M_cost(i,j);
    end
end
% Assigning cost involving the start and end nodes
% Cost of going from the start and end to the other nodes
for i = 2:2*N+1 
    % Manhattan distance between this node and start node
    M_cost(1,i) = abs(X(1)-X(i)) + abs(Y(1)-Y(i));
    M_cost(i,1) = M_cost(1,i); 
    M_cost(2*N+2,i) = abs(X(2*N+2)-X(i)) + abs(Y(2*N+2)-Y(i));
    M_cost(i,2*N+2) = M_cost(2*N+2,i); 
end
% Cost between start and end nodes
M_cost(1,2*N+2) = HUGE;
M_cost(2*N+2, 1) = HUGE; 

%% Compute optimal row traversal sequence 
t = cputime;
resultStruct = tspof_ga('XY',XY,'DMAT',M_cost,'SHOWRESULT',false,'SHOWWAITBAR',false,'SHOWPROG',false);
E = cputime-t;
route = [1 resultStruct.optRoute 2*N+2];
resultStruct.minDist;
% Path Generation
path = path_gen(route,XY);
Points = [path; ones(size(path(1,:)))];

%% Initialize the Covariance matrices
% The covariances matrices were calculated using the code
% "Covariance.matrix.m".
V = [0.0025 0;
        0 0.0001]; % Odometry covariance matrix
M_gps = 10^(-3)*[0.9144 0 0;
                    0 0.9047 0;
                    0 0 0.3966]; % GPS covariance matrix
P = zeros(3); % The initial pose of the robot is known (zero covariance matrix)

%% Initialize the true state, gps and corrected pose arrays

trueState = zeros(5,T/DT); % Initialize a vector to store the true state
trueState(ROBOT_POS_X,1) = q(ROBOT_POS_X); trueState(ROBOT_POS_Y,1) = q(ROBOT_POS_Y);
trueState(ROBOT_THETA,1) = q(ROBOT_THETA); trueState(ROBOT_STEER,1) = q(ROBOT_STEER);
trueState(ROBOT_SPEED,1) = q(ROBOT_SPEED);

gpsOutp = zeros(3,1); % Initialize a vector to store the GPS ouput
gpsOutp(1) = trueState(ROBOT_POS_X,1); gpsOutp(2) = trueState(ROBOT_POS_Y,1); 
gpsOutp(3) = trueState(ROBOT_THETA,1);
correctedPose = zeros(3,T/DT); % Initialize a vector to store the EKF ouput
correctedPose(:,1) = [trueState(ROBOT_POS_X,1);trueState(ROBOT_POS_Y,1);trueState(ROBOT_THETA,1)];

%% Move the robot

i = 1; j = 1;
previous = 0;
for t = 0:DT:T-DT
    % Feed data of current state to the pure pursuit controller
    ppController(i,:) = purePursuitController(correctedPose(:,i), L, Ld, Points);
    % Change the steer input based on the pure pursuit controller output
    u (STEER_INPUT) = ppController(i,1);   
    % Feed the updated data to the model  
    [trueState(:,i+1), odo] = robot_odo(trueState(:,i), u, umin, umax,Qmin, Qmax, L, tau_gamma, tau_v);  
    if previous == length(path) 
        break 
    end   
    % GPS readings are available once every 100 control intervals
    if rem(j,100) == 0 && j ~= 0
        [gpsOutp(1),gpsOutp(2),gpsOutp(3)] = GPS_CompassNoisy(trueState(1,i),trueState(2,i),trueState(3,i));
        j = 0;
    end
    % Use the Extended Kalman Filter
    [correctedPose(:,i+1), PNext] = EKF(correctedPose(:,i), P, odo, V, M_gps, gpsOutp, j); 
    P = PNext;
    
    if correctedPose(1,i+1) > 0 && correctedPose(2,i+1) > 0
        % Start scanning
        Tl = SE2([correctedPose(1,i+1) correctedPose(2,i+1) correctedPose(3,i+1)]);
        p = laserScannerNoisy(angleSpan, angleStep, rangeMax, Tl.T, bitmap, Xmax, Ymax);
        p_noisyRange = p(:,2);
        % Applying a median filter of order M 
        p_Rangefiltered = medianFilter(p_noisyRange,length(p),5);
        p_filtered = [p(:,1) p_Rangefiltered];
        for s=1:length(p)
            angle = p_filtered(s,1); 
            range = p_filtered(s,2);
            if(isinf(range)) 
                range = rangeMax+1;
            end
            n = updateLaserBeamGrid(angle, range, Tl.T, R, C, Xmax, Ymax);
        end
    end
    j = j + 1;
    i = i + 1;
end
%% Estimate map of the real world
for i = 1:R
    for j = 1:C
        % Generating the probability map
        probmap(i,j) = oddsmap(i,j)/(1+oddsmap(i,j));
        probmap2(i,j) = oddsmap(i,j)/(1+oddsmap(i,j));
    end 
end

figure(2);
imagesc([0 Xmax], [Ymax 0],probmap); title('Estimate map of the perceived environment')
colorbar
caxis([0 1])

%% Run the function to generate the final report

fReport = treeDetector(XY,Xmax,Ymax,R,C,RL,N,RS);

%% ERROR ANALYSIS
k = 1;

for i = 1:length(fReport)
    dx = (fReport(i,3)-nursery(i,1))^2;
    dy = (fReport(i,4)-nursery(i,2))^2;
    diff = sqrt(dx+dy);
    error_dist(k) = diff;
    diamater_error(k) = abs(fReport(i,5) - 2*nursery(i,3)); 
    k = k+1;
end


%% Error Plots

figure(4)
X_p = correctedPose(1,:);
Y_p = correctedPose(2,:);
plot(X_p,Y_p,'-'); 
hold on
plot(path(1,:),path(2,:),'-'); 
title('Odometry-based path vs. True path')
xlabel('Position in X (m)')
ylabel('Position in Y (m)')
legend('Odometry-based path','True path');

figure()
histogram(diamater_error,5)
xlabel('Error (m)')
title('Tree diameter error'); 


figure()
histogram(error_dist,5)
xlabel('Error (m)')
title('Tree Position error'); 

rmse_error_d = rms(diamater_error);
max_error_d = max(diamater_error);
percentile_error_d = prctile(diamater_error,95);

fprintf('The maximum diameter error is %.3f m\n', max_error_d);
fprintf('The 95th percentile diameter error is %.3f m\n', percentile_error_d);
fprintf('The RMSE of diameter error is %.3f m\n', rmse_error_d);



rmse_error_p = rms(error_dist);
max_error_p = max(error_dist);
percentile_error_p = prctile(error_dist,95);

fprintf('The maximum Tree Position error is %.3f m\n', max_error_p);
fprintf('The 95th percentile Tree Position error is %.3f m\n', percentile_error_p);
fprintf('The RMSE of Tree Position error is %.3f m\n', rmse_error_p);
