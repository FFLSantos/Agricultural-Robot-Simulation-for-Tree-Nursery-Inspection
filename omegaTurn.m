% Function to perform a omega turn
function o_t = omegaTurn(P_i, P_j, Rmin, d, alpha)
global step_ang RS;

threshold = 26;

% First we will decide if our robot is going to turn top or bottom; and
% left or right

% Choosing between top or bottom turn
if P_i(2) > threshold % Perform an omega turn on a top node
    if P_i(1) > P_j(1) % Setting the direction for the turn (left or right)
    % In this case, our final goal is to turn left. So, we will perform a
    % RLR turn
        a_if = pi-alpha; a_ff = pi; % Initial and final angles of the first turn
        a_is = -alpha; a_fs = pi+alpha; % Initial and final angles of the second turn
        a_it = 0; a_ft = alpha; % Initial and final angles of the third turn
    else
    % In this case, our final goal is to turn right. So, we will perform a
    % LRL turn
        a_if = 0; a_ff = alpha;
        a_is = -alpha; a_fs = pi+alpha;
        a_it = pi-alpha; a_ft = pi;
    end
else % Perform an omega turn on a bottom node
    if P_i(1) > P_j(1) % Setting the direction for the turn (left or right)
    % In this case, our final goal is to turn left. So, we will perform a
    % RLR turn
        a_if = pi; a_ff = alpha + pi; 
        a_is = -alpha + pi; a_fs = 2*pi+alpha; 
        a_it = 2*pi-alpha; a_ft = 2*pi; 
    else
    % In this case, our final goal is to turn right. So, we will perform a
    % LRL turn
        a_if = 2*pi-alpha; a_ff = 2*pi; 
        a_is = -alpha + pi; a_fs = 2*pi+alpha;
        a_it = pi; a_ft = alpha + pi;
    end
end

% Generating the first turn
a = a_if : step_ang : a_ff; % angle step for points on circular path
x_ft = P_i(1) + (-1)*(P_i(1) < P_j(1))*2*Rmin + Rmin + Rmin*cos(a); 
y_ft = P_i(2) + Rmin*sin(a);
Points_first_turn = [x_ft; y_ft];

% Generating the third turn
a = a_it : step_ang : a_ft ; % angle step for points on circular path
x_tt = P_j(1) + (-1)*(P_i(1) > P_j(1))*2*Rmin + Rmin + Rmin*cos(a); 
y_tt = P_j(2) + Rmin*sin(a);
Points_third_turn = [x_tt; y_tt];

if P_i(1) > P_j(1)&& P_i(2) > threshold
    ind_1 = 1;
    ind_2 = length(Points_third_turn);
elseif P_i(1) < P_j(1)&& P_i(2) > threshold
    ind_1 = length(Points_first_turn);
    ind_2 = 1;
elseif P_i(1) > P_j(1)&& P_i(2) < threshold  
    ind_1 = length(Points_first_turn);
    ind_2 = 1;
else P_i(1) < P_j(1)&& P_i(2) < threshold 
    ind_1 = 1;
    ind_2 = length(Points_third_turn);
end

dist_y = sqrt((2*Rmin)^(2)-(Rmin+d*RS/2)^(2));
% Generating the second turn
a = a_is: step_ang : a_fs; % angle step for points on circular path
x_st = (Points_first_turn(1,ind_1) + Points_third_turn(1,ind_2))/2 + Rmin*cos(a);
y_st = (Points_first_turn(2,ind_1) + Points_third_turn(2,ind_2))/2 + dist_y - Rmin/2 +(-2*dist_y + Rmin)*(P_i(2) < threshold)*(P_i(2) < threshold) + Rmin*sin(a);
Points_second_turn = [x_st; y_st];

% Inverting the sequence of the turns, so that the actual point sequence is
% in the right order
if P_i(2) < threshold && P_i(1) < P_j(1)
    Points_first_turn_X = flip(Points_first_turn(1,:));
    Points_first_turn_Y = flip(Points_first_turn(2,:));
    Points_first_turn = [Points_first_turn_X ;Points_first_turn_Y];
    Points_third_turn_X = flip(Points_third_turn(1,:));
    Points_third_turn_Y = flip(Points_third_turn(2,:));
    Points_third_turn = [Points_third_turn_X ;Points_third_turn_Y]; 
elseif P_i(2) < threshold && P_i(1) > P_j(1)
    Points_second_turn_X = flip(Points_second_turn(1,:));
    Points_second_turn_Y = flip(Points_second_turn(2,:));
    Points_second_turn = [Points_second_turn_X ;Points_second_turn_Y];
elseif P_i(2) > threshold && P_i(1) < P_j(1)
    Points_second_turn_X = flip(Points_second_turn(1,:));
    Points_second_turn_Y = flip(Points_second_turn(2,:));
    Points_second_turn = [Points_second_turn_X ;Points_second_turn_Y];
else P_i(2) > threshold && P_i(1) > P_j(1)   
    Points_first_turn_X = flip(Points_first_turn(1,:));
    Points_first_turn_Y = flip(Points_first_turn(2,:));
    Points_first_turn = [Points_first_turn_X ;Points_first_turn_Y];
    Points_third_turn_X = flip(Points_third_turn(1,:));
    Points_third_turn_Y = flip(Points_third_turn(2,:));
    Points_third_turn = [Points_third_turn_X ;Points_third_turn_Y];     
end

Points_turn = cat(2,Points_first_turn, Points_second_turn, Points_third_turn);

o_t = Points_turn; 
end