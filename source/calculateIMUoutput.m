function [linear_acceleration, angular_rate] = calculateIMUoutput(prev_position, current_position, next_position, prev_quaternion, current_quaternion, next_quaternion, delta_time)
    % Input: prev_position: [x, y, z]
    %        current_position: [x, y, z]
    %        next_position: [x, y, z]
    %        prev_quaternion: [p, q, r, s] (assuming [x, y, z, w] format for quaternion)
    %        current_quaternion: [p, q, r, s] (assuming [x, y, z, w] format for quaternion)
    %        next_quaternion: [p, q, r, s] (assuming [x, y, z, w] format for quaternion)
    %        delta_time: time interval between consecutive positions/quaternions
    % Output: linear_acceleration: [ax, ay, az]
    %         angular_rate: [gx, gy, gz]

    % Constants
    g = 9.81;  % gravitational acceleration in m/s^2

    % Calculate velocity at current and next time steps
    velocity_t = (current_position - prev_position) / delta_time;
    velocity_t_plus_1 = (next_position - current_position) / delta_time;
    
    % Calculate raw linear acceleration using central difference method
    raw_linear_acceleration = (velocity_t_plus_1 - velocity_t) / delta_time;

    % Normalize the quaternion
    q = current_quaternion / norm(current_quaternion);

    % Calculate the gravity vector from the quaternion
    gravity = [2*(q(2)*q(4) - q(1)*q(3)), ...
               2*(q(3)*q(4) + q(1)*q(2)), ...
               q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2] * g;

    % Subtract gravity from the raw linear acceleration
    linear_acceleration = raw_linear_acceleration - gravity;

    % Quaternion derivative (using central difference method)
    q_dot = (next_quaternion - prev_quaternion) / (2 * delta_time);
    
    % Convert quaternion derivative to angular velocity
    raw_angular_rate = 2 * quatmultiply(q_dot, quatconj(current_quaternion));
    angular_rate = raw_angular_rate(1:3); % Extract only the vector part

    % Nested function to multiply two quaternions
    function q = quatmultiply(q1, q2)
        q = [ q1(4)*q2(1) + q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2), ...
              q1(4)*q2(2) - q1(1)*q2(3) + q1(2)*q2(4) + q1(3)*q2(1), ...
              q1(4)*q2(3) + q1(1)*q2(2) - q1(2)*q2(1) + q1(3)*q2(4), ...
              q1(4)*q2(4) - q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) ];
    end

    % Nested function to calculate the conjugate of a quaternion
    function q_conj = quatconj(q)
        q_conj = [-q(1), -q(2), -q(3), q(4)];
    end
end