close all; clear all; clc;

g = 9.81;

q0 = [0;
      0;
      2;];

dq0 = [0;
       0;
       0;];

qp0 = [0;
      0;
      1.5;];

dqp0 = [0;
       0;
       0;];



% Constants
params.mq = 1;
params.mp = 0.2;
params.L = 0.5;
params.e3 = [0;0;1];
params.g = -9.81;

% Controller Constants
params.maxProperAcc = 13;
params.minVerticalProperAcceleration = -1;

% Initial conditions
rhat = [0;0;-1];

s_initial = [q0;dq0];
s0 = s_initial;

sp_initial = [qp0; dqp0];
sp0 = sp_initial;

% T = params.mp * 9.81;

dt = 0.002;
t_steps = 0:dt:5;

s_arr = zeros(6, length(t_steps));
s_arr(:, 1) = s0; % Initialize the first state in the array

sp_arr = zeros(6, length(t_steps));
sp_arr(:, 1) = sp0; % Initialize the first state in the array

T_arr = zeros(1, length(t_steps));
% T_arr(1,1) = T;

R_arr = zeros(3, length(t_steps));
R_arr(:,1) = rhat;

for i = 2:length(t_steps)
    t = t_steps(i);
    [s1, sp1, T1, rhat] = simulate(t, dt, s0, sp0, rhat, params);
    s_arr(:, i) = s1; % Store the state in the array for each time step
    sp_arr(:, i) = sp1; % Store the state in the array for each time step

    T_arr(1, i) = T1;
    T = T1;

    R_arr(:, i) = rhat;

    s0 = s1; % Update the state for the next iteration
    sp0 = sp1;

end

figure(1); hold on;
title('Drone Position over time')
subplot(3,1,1);
plot(t_steps, s_arr(1,:), 'r', 'LineWidth', 2)
subplot(3,1,2);
plot(t_steps, s_arr(2,:), 'g', 'LineWidth', 2)
subplot(3,1,3);
plot(t_steps, s_arr(3,:), 'b', 'LineWidth', 2)


figure(2); hold on;
title('Payload Position over time')
subplot(3,1,1);
plot(t_steps, sp_arr(1,:), 'r', 'LineWidth', 2)
subplot(3,1,2);
plot(t_steps, sp_arr(2,:), 'g', 'LineWidth', 2)
subplot(3,1,3);
plot(t_steps, sp_arr(3,:), 'b', 'LineWidth', 2)


figure(4); hold on;
plot(t_steps(2:end), T_arr(1,2:end), 'r', 'LineWidth', 2)
title('T over time')

figure(5); hold on;
title('rhat over time')
subplot(3,1,1);
plot(t_steps, R_arr(1,:), 'g', 'LineWidth', 2)
subplot(3,1,2);
plot(t_steps, R_arr(2,:), 'g', 'LineWidth', 2)
subplot(3,1,3);
plot(t_steps, R_arr(3,:), 'g', 'LineWidth', 2)

% animate([s_arr;sp_arr], dt)

% figure(2)
% plot3(s_arr(1,:), s_arr(2,:), s_arr(3,:), '.')

%% Helper functions

function [s1, sp1, T1, new_rhat] = simulate(t, dt, s0, sp0, rhat, params)

mq = params.mq;
mp = params.mp;
e3 = params.e3;
g = params.g;
L = params.L;

q = s0(1:3);
dq = s0(4:6);

p = sp0(1:3);
dp = sp0(4:6);

% X = ddq, ddp, T

f = u_input(params, t, p, dp);

A = [mq * eye(3), zeros([3,3]), -rhat;
     zeros([3,3]), mp * eye(3), +rhat;
     -rhat.', rhat.', 0];

B = [f + e3 * mq * g;
     e3 * mp * g;
     -norm(dp - dq)^2/L];

X = A\B;

ddq = X(1:3);
ddp = X(4:6);
T1 = X(7);

new_dq = dq + dt*ddq;
new_dp = dp + dt*ddp;

new_q = q + dt*new_dq;
new_p = p + dt*new_dp;

new_rhat = (new_p-new_q)/norm(new_p-new_q);

sp1 = [new_p; new_dp];
s1 = [new_q; new_dq];

end

% function f = u_input(t)
% 
%     thrust_eq = 9.81*1.2;
% 
%     if t < 0.2
%         f = [1;0;thrust_eq];
%     elseif t > 1 && t < 1.2
%         f = [-1;0;thrust_eq];
%     % elseif t < 2.5
%     %     f = [0.1;0;thrust_eq*0.9];
%     else
%         f = [0;0;thrust_eq];
%     end
% 
%     % f = [0.5;0;thrust_eq];
% 
% end

function f = u_input(params, t, p, pdot)
    m = 1.2;
    g = 9.81;

    pd = desired_position(t);
    pdot_d = [0;0;0];

    Kp = 1*eye(3);
    % Kd = 0.2*eye(3);
    Kd = zeros([3,3]);

    a_d = Kp*(pd - p) + Kd*(pdot_d - pdot) + [0;0;g];

    if norm(a_d) > params.maxProperAcc
        a_d = a_d * params.maxProperAcc / norm(a_d);
    end

    if a_d(3) < params.minVerticalProperAcceleration
        a_d(3) = params.minVerticalProperAcceleration;
    end

    if a_d(1) > 0.5
        a_d(1) = 0.5;
    end

    if a_d(2) > 0.5
        a_d(2) = 0.5;
    end

    % desired roll + pitch
    % phi_d   = a_d(2) / g;
    % theta_d = a_d(1) / g;

    % full thrust magnitude
    T = m*norm(a_d);

    % Rotation matrix from desired orientation
    % R = eul2rotm([0, theta_d, phi_d]); % yaw = 0 for simplicity

    ax = cross([1;1;1], a_d);
    ang = atan2(norm(ax), dot([1;1;1], a_d));

    R = axang2rotm([ax; ang]');

    f = R * [0;0;T];   % force in world frame
end

function pd = desired_position(t)
    if t < 1
        pd = [0; 0; 2];     % hover at (0,0)
    elseif t < 2
        pd = [1; 0; 2];     % slide to x = 1
    else
        pd = [0; 0; 2];     % return to hover
    end
end
