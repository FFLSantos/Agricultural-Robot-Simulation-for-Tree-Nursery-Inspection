% Function to perform a pi turn
function p_t = piTurn(P_i, P_j, Rmin, d)
global step_ang RS;

threshold = 26;

% First we will decide if our robot is going to turn top or bottom; and
% left or right

% Choosing between top or bottom turn
if P_i(2) > threshold % Perform a pi turn on a top node
    if P_i(1) > P_j(1) % Setting the direction for the turn (left or right)
    % In this case, our final goal is to turn left. So, we will perform a
    % left turn
        a_if = 0; a_ff = pi/2; % Initial and final angles of the first turn
        a_is = pi/2; a_fs = pi; % Initial and final angles of the second turn
    else
    % In this case, our final goal is to turn right. So, we will perform a
    % right turn
        a_if = pi/2; a_ff = pi;
        a_is = 0; a_fs = pi/2;
    end
else % Perform a pi turn on a bottom node
    if P_i(1) > P_j(1) % Setting the direction for the turn (left or right)
    % In this case, our final goal is to turn left. So, we will perform a
    % left turn
        a_if = pi+pi/2; a_ff = 2*pi; % Initial and final angles of the first turn
        a_is = pi; a_fs = pi+pi/2; % Initial and final angles of the second turn
    else
    % In this case, our final goal is to turn right. So, we will perform a
    % right turn
        a_if = pi; a_ff = pi+pi/2;
        a_is = pi+pi/2; a_fs = 2*pi;
    end
end

% Generating the first turn
a = a_if : step_ang : a_ff; % angle step for points on circular path
x_ft = P_i(1) + (-1)*(P_i(1) > P_j(1))*2*Rmin + Rmin + Rmin*cos(a); 
y_ft = P_i(2) + Rmin*sin(a);
Points_first_turn = [x_ft; y_ft];
%plot(Points_first_turn(1,:),Points_first_turn(2,:),'o')

if P_i(2) > threshold
    ind = (P_i(1) > P_j(1))*(length(Points_first_turn) - 1) + 1; % When going top
else
    ind = (P_i(1) < P_j(1))*(length(Points_first_turn) - 1) + 1; % When going bottom
end

X_s = abs(d*RS-2*Rmin);

% Generating the line segment
i = 0;
step_dist = step_ang;
for dx = step_dist: step_dist : X_s
    i = i + 1;
    y = Points_first_turn(2,ind);
    x_ls(i) = Points_first_turn(1,ind) + (-1)*(P_i(1) > P_j(1))*2*dx + dx; y_ls(i) = y;
end
Points_X_segment = [x_ls; y_ls];

% Generating the second turn
a = a_is : step_ang : a_fs; % angle step for points on circular path
x_st = Points_X_segment(1,end) + Rmin*cos(a); 
y_st = -2*Rmin*(P_i(2) > threshold)+ Rmin + Points_X_segment(2,end) + Rmin*sin(a);
Points_second_turn = [x_st; y_st];

% Inverting the sequence of the turns, so that the actual point sequence is
% in the right order
if P_i(2) > threshold && P_i(1) < P_j(1)
    Points_first_turn_X = flip(Points_first_turn(1,:));
    Points_first_turn_Y = flip(Points_first_turn(2,:));
    Points_first_turn = [Points_first_turn_X ;Points_first_turn_Y];
    Points_second_turn_X = flip(Points_second_turn(1,:));
    Points_second_turn_Y = flip(Points_second_turn(2,:));
    Points_second_turn = [Points_second_turn_X ;Points_second_turn_Y]; 
elseif P_i(2) < threshold && P_i(1) > P_j(1)
    Points_first_turn_X = flip(Points_first_turn(1,:));
    Points_first_turn_Y = flip(Points_first_turn(2,:));
    Points_first_turn = [Points_first_turn_X ;Points_first_turn_Y];
    Points_second_turn_X = flip(Points_second_turn(1,:));
    Points_second_turn_Y = flip(Points_second_turn(2,:));
    Points_second_turn = [Points_second_turn_X ;Points_second_turn_Y];    
end

Points_turn = cat(2,Points_first_turn, Points_X_segment, Points_second_turn);

p_t = Points_turn; 
end