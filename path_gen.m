function path = path_gen(route,nodes)
global N Rmin sens RS;
path = [];
 %The inputs to the pi, omega, and straight functions require node
 %coordinates. 
 %d*w = total distance needed to be covered
for i = 1:length(route)-1   
    if i<length(route)
        d = abs(route(i)-route(i+1));%d = differnce between rows --> how many rows apart.
        P_i = nodes(route(i),:); % Current node coordinates == XY(route(1),:)
        P_j = nodes(route(i+1),:);% Next node coordinates
        d_1 = P_j(1)/RS; % First node
        d_2 = P_i(1)/RS;
        gamma = acos(1-(2*Rmin+d*RS)^(2)/(8*Rmin^(2)));
        alpha = (pi - gamma)/2;
        gamma_1 = acos(1-(2*Rmin+d_1*RS)^(2)/(8*Rmin^(2)));
        alpha_1 = (pi - gamma_1)/2;
        gamma_2 = acos(1-(2*Rmin+d_2*RS)^(2)/(8*Rmin^(2)));
        alpha_2 = (pi - gamma_2)/2;
    else % we need a case for last node because there is no next position, but it has to return to the original positions
        
        d = 0;
        P_i = nodes(route(i),:);
        P_j = 0; 
    end
    % So now there are three ways the robot moves: pi, omega, straight
    %d*w = total distance needed to be covered
    % Movement 1:  d*w>2*rmin --->  pi turn
    % Movement 2: d*w<2*rmin ---> Omega turn 
    % Movement 3: If the distance between node i and node j equal length of
    % row, then go stright. 
    
    %But we need to define movement seperately for the first node and the
    %last node. This is because the length of the first row is less than N.
    %and the last node has to return to the initial starting position. 
    
    if (i == 1) % First node
        straight = [P_i(1)*ones(1,sens);linspace(P_i(2),P_j(2),sens)];
        if (d_1*RS<2*Rmin)
            turn = omegaTurn([P_i(1), P_j(2)], P_j, Rmin, d_1,alpha_1);
        elseif (d_1*RS>=2*Rmin)
            turn = piTurn([P_i(1), P_j(2)], P_j, Rmin, d_1);
        end
        path = [straight turn];
    elseif i == length(route)-1
        if (d_2*RS<2*Rmin)
            turn = omegaTurn(P_i, [P_j(1), P_i(2)], Rmin, d_2,alpha_2);
        elseif (d_2*RS>=2*Rmin)
            turn = piTurn(P_i, [P_j(1), P_i(2)], Rmin, d_2);
        end
        straight = [P_j(1)*ones(1,sens);linspace(P_i(2),P_j(2),sens)];
        path_new = [turn straight];
        path = [path path_new];      
    else
        if (d==N) %if differnce between row coordinates is 10
           path_new = [P_i(1)*ones(1,sens);linspace(P_i(2),P_j(2),sens)]; 
        else 
            if (d*RS<2*Rmin)% only one row apart
                path_new = omegaTurn(P_i, P_j, Rmin, d,alpha);
            
            elseif (d*RS>=2*Rmin)
                 path_new = piTurn(P_i, P_j, Rmin, d);
            end
        
        end
        path = [path path_new];
    end
  
end
end