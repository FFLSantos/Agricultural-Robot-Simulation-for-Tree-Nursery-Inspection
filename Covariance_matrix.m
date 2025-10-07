%% Covariance matrix calculation

% This code works as an experiment, where we know the true position of the robot.
% Here, we run the odometry for small interval of time, several times.
% We will command the robot to move straight to a random location, and we will compare the output of the odometry
% with the previous state of the robot.
% Finally, we will calculate the error statistics. The variance of the
% error is our sigma.

clear all;

% Experiment Parameters

T = 10000; % Experiment time duration (s). It needs to be long to produce large sample sizes
DT = 0.01; % Controller sampling rate
T_steps = T/DT;

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
u = [0 0]; 
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
% q(4) -> linear velocity
% q(5) -> phi (steering angle)
q = [0 0 0 u(SPEED_INPUT) u(STEER_INPUT)];
ROBOT_POS_X = 1; ROBOT_POS_Y = 2; ROBOT_THETA = 3; ROBOT_SPEED = 4; ROBOT_STEER = 5;

% Defining the state constraints
Qmax(ROBOT_POS_X) = Inf; Qmax(ROBOT_POS_Y) = Inf; Qmax(ROBOT_THETA) = Inf; 
Qmax(ROBOT_STEER) = umax(MAX_STEER);Qmax(ROBOT_SPEED) = umax(MAX_SPEED);
Qmin = -Qmax; % symmetrical negative constraints for minimum values

% Initialize State Arrays

M_true = zeros(5,T_steps);
M_true(:,1) = q'; 
q_previous = M_true(:,1);
M_gps = zeros(3,T_steps/100);
odo = [0 0];

% Run Simulation
j = 1; k = 1;
for i = 1:T_steps-1
    % Command the Robot to run on a straight path with randomn velocity and steer angle
    u=[(randn*(90*pi/180)) Vmax*randn];
   
    % Feed Input to the Vehicle model
    [q_next, odo] = robot_odo(q_previous,u,umin,umax,Qmin,Qmax,L,tau_gamma,tau_v);
    
    %Calculate Odometry Error
    d_theta_true = q_next(3)-q_previous(3);
    d_dist_true = sqrt((q_next(1)-q_previous(1))^2+(q_next(2)-q_previous(2))^2);
    e_dist(i) = d_dist_true - odo(1);
    e_theta(i) = d_theta_true - odo(2);
    
    % GPS readings are available once every 100 control intervals
    if rem(j,100) == 0 && j ~= 0
        [M_gps(1,k),M_gps(2,k),M_gps(3,k)] = GPS_CompassNoisy(M_true(1,i),M_true(2,i),M_true(3,i));
        e_x(k) = M_true(1,i) - M_gps(1,k);
        e_y(k) = M_true(2,i) - M_gps(2,k);
        e_theta_n(k) = M_true(3,i) - M_gps(3,k); 
        k = k+1;
        j = 0;
    end
     
    q_previous = q_next;
    X_true(:,i+1) = q_next;
    j = j+1;
end

% Creating the Covariance Matrix

% Odometry Covariance Matrix
sigma_theta = var(e_theta);
sigma_d = var(e_dist);
CovM_odo = [sigma_d 0; 0 sigma_theta];

% GPS and compass error Covariance Matrix
sigma_x = var(e_x);
sigma_y = var(e_y);
sigma_theta_n = var(e_theta_n);
CovM_gps = [sigma_x 0 0; 0 sigma_y 0; 0 0 sigma_theta_n];

