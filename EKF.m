function [qEstNext, PNext] = EKF(qEst, P, odo, V, W, q_gps, j)
%% Description
% This function gives the best estimate of pose based upon GPS data and
% odometry model
%% Main Variables
% qEst -> Current estimated state
%   qEst(1) -> X Coordinate (m)
%   qEst(2) -> Y Coordinate (m)
%   qEst(3) -> Theta angle 
% V -> Odometry covariance matrix
% P -> Current estimate of the state uncertainty
% W -> GPS covariance matrix
% q_gps -> Output of the GPS+Compass device (noise included)
%   q_gps(1) -> X Coordinate (m)
%   q_gps(2) -> Y Coordinate (m)
%   q_gps(3) -> Theta angle 
% odo -> Odometry of the robot
%   odo(1) -> Change in distance between qNext and qPrev
%   odo(2) -> Change in theta angle between qNext and qPrev
%% Odometry-based pose estimation
% Estimating the next state
qEstNext = zeros(1,3); % Initializing the vector
qEstNext(1) = qEst(1)+odo(1)*cos(qEst(3)); % X Coordinate (m)
qEstNext(2) = qEst(2)+odo(1)*sin(qEst(3)); % Y Coordinate (m)
qEstNext(3) = qEst(3)+odo(2); % Theta angle

% Jacobians for linearization
Fx = [1 0 -odo(1)*sin(qEst(3));
        0 1 odo(1)*cos(qEst(3));
        0 0 1];
Fv = [cos(qEst(3)) 0;
        sin(qEst(3)) 0;
        0 1];
%% Corke EKF Prediction
H = [1 1 1]; % Measurement matrix
% Estimate of the next state uncertainty
P = Fx*P*Fx'+Fv*V*Fv';    
if rem(j,100) == 0 && j ~= 0 % If GPS readings are available 
    % Innovation (v)
    % It's the discrepancy between the reality (GPS measurement) and the prediction 
    v = q_gps - qEstNext; 
    % Compute Kalman Gain
    K = P*H'/(H*P*H' + W);
    %Update State Estimate
    qEstNext =  qEstNext + K*v;
    % Update error covariance estimation
    P = P-K*H*P;
end
PNext = P; 
end