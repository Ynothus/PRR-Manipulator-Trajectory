function [theta, theta_dot, theta_ddot, ts] = trajectory_planning(q0, q0_dot, q1, q1_dot, q_via, t1)
% Trajectory planning based on intial position and velocity, target position and velocity, and given time duration.
% Inputs:
% q0, q0_dot: intial point position and velocity
% q1, q1_dot: goal point position and velocity
% t1: time to achieve goal

% Output:
% theta: time series position trajectory
% theta_dot: time series velocity trajectory
% theta_ddot: time series acceleration trajectory
% ts: time steps

    %  Cubic polynomial coefficients solver function
    %  q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    %  given q(0), q_dot(0), q(Tf), q_dot(Tf)

    function [a0, a1, a2, a3] = cubic_coeffs(q_start, v_start, q_end, v_end, T)
        a0 = q_start;
        a1 = v_start;
        a2 = ( 3*(q_end - q_start) - T*(2*v_start + v_end) ) / T^2;
        a3 = (-2*(q_end - q_start) + T*(v_start + v_end)   ) / T^3;
    end

    % Time parameters 
    dt    = 0.01;                     % time step resolution
    t_via = t1 / 2;                   % time at via point (half of total)
    
    ts1 = 0     : dt : t_via;         % time vector for segment 1
    ts2 = t_via : dt : t1;            % time vector for segment 2
    
    N1 = length(ts1);
    N2 = length(ts2);
    
    % Preallocate outputs time series
    N_total    = N1 + N2 - 1;        % -1 to avoid duplicate at via point
    theta      = zeros(1, N_total);
    theta_dot  = zeros(1, N_total);
    theta_ddot = zeros(1, N_total);
    q_via_dot = 0;                   % Velocity at via point = 0
    
    % Segment 1: q0 -> q_via 
    [a0, a1, a2, a3] = cubic_coeffs(q0, q0_dot, q_via, q_via_dot, t_via);
    
    tau1 = ts1;  % local time for segment 1
    theta(1:N1)      = a0 + a1*tau1 + a2*tau1.^2 + a3*tau1.^3;
    theta_dot(1:N1)  = a1 + 2*a2*tau1 + 3*a3*tau1.^2;
    theta_ddot(1:N1) = 2*a2 + 6*a3*tau1;
    
    % Segment 2: q_via -> q1
    T2 = t1 - t_via;                 % duration of segment 2
    
    [b0, b1, b2, b3] = cubic_coeffs(q_via, q_via_dot, q1, q1_dot, T2);
    
    tau2 = ts2(2:end) - t_via;       % local time (skip first to avoid duplicate)
    idx  = N1+1 : N_total;
    
    theta(idx)      = b0 + b1*tau2 + b2*tau2.^2 + b3*tau2.^3;
    theta_dot(idx)  = b1 + 2*b2*tau2 + 3*b3*tau2.^2;
    theta_ddot(idx) = 2*b2 + 6*b3*tau2;
    
    % Time vector
    ts = [ts1, ts2(2:end)];
    



% plot planned trajectory
figure();
subplot(3,1,1)
plot(ts, theta);
ylabel('angle');
xlabel('Time (s)');
subplot(3,1,2)
plot(ts, theta_dot);
ylabel('velocity');
xlabel('Time (s)');
subplot(3,1,3)
plot(ts, theta_ddot);
ylabel('acceleration');
xlabel('Time (s)');
sgtitle('Planned trajectory');
end