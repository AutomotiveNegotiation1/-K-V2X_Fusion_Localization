function theta_adjusted = quaternion_to_angle(q1, q2)
    % Calculate the quaternion difference
    q_diff = quatmultiply(quatinv(q1), q2);
    
    % Convert quaternion difference to angle
    theta = 2 * acos(q_diff(1));
    
    % Adjust the angle to be within 0 to 2pi
    if theta < 0
        theta = theta + 2 * pi;
    end
    
    theta_adjusted = theta;
end
