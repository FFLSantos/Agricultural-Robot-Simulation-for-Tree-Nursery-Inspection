% Function of the Pure Pursuit Controller
function ppController = purePursuitController(q, L, Ld, Points)
global ROBOT_POS_X ROBOT_POS_Y ROBOT_THETA
global previous;

if isempty(previous)
    previous = 1; 
end

W_T_R = transl2(q(ROBOT_POS_X), q(ROBOT_POS_Y))*trot2(q(ROBOT_THETA)); % Function to transform the world frame to robot frame
W_T_R_inv = inv(W_T_R);
% Now, we multiply the points by the WTR to obtain the path in the robot
% frame
Points_RF = W_T_R_inv*Points;

% Find the waypoint on the path that is closest to robot
nP = length(Points); % Find the number of points in Path
window = 15;

WindowStart = max(previous-window, 1); %window is the current - 15 points
WindowEnd = min(nP, previous+window); %window is the current + 15 points
nextpoint = 0;
Di_diff = zeros(1,nP);
Di = zeros(1,nP);

for i = WindowStart:WindowEnd % Reduced the range for the search of goal point  
    Di(i) = sqrt(Points_RF(1,i)^(2) + Points_RF(2,i)^(2));
    Di_diff(i) = abs(Di(i)-Ld);
    if (Points_RF(1,i) > 0) && (Di(i) < Ld)  
        nextpoint = i; %look ahead goal
        Target_points(:,i) = Points_RF(1:2,nextpoint);
    end
end

if nextpoint == 0
    e_y = min(Di(Di>0)); %replacing idx with this
    Target_points(:,1) = [q(ROBOT_POS_X), q(ROBOT_POS_Y)];
else     
    e_y = Points_RF(2,nextpoint);   
end

% Calculating the steer angle
R = Ld^(2)/(2*e_y);
steer_ang = atan(L/R);
% Calculating the cross-track error (e)
e=min(Di(Di>0));

First_target_point = Target_points(:,end);

ppController = [steer_ang; e; First_target_point];

previous = nextpoint;
