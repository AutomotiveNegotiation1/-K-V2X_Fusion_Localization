function [linear_acceleration, angular_rate] = slam_to_imu(current_position, current_quaternion, previous_position, previous_quaternion, delta_t)
    % 현재 위치와 쿼터니언
    x_t = current_position(1);
    y_t = current_position(2);
    z_t = current_position(3);
    q_t = current_quaternion;
    
    % 이전 위치와 쿼터니언
    x_t_minus = previous_position(1);
    y_t_minus = previous_position(2);
    z_t_minus = previous_position(3);
    q_t_minus = previous_quaternion;
    
    % 위치로부터 속도 계산 (중앙 차분)
    velocity_t = (current_position - previous_position) / delta_t;
    
    % 이전 속도 계산을 위해 한 번 더 위치 입력 필요 (이전 시간의 위치)
    % 여기서는 예제로 이전 위치를 current_position에 사용합니다.
    velocity_t_minus = (previous_position - current_position) / delta_t;
    
    % 속도로부터 가속도 계산 (중앙 차분)
    linear_acceleration = (velocity_t - velocity_t_minus) / delta_t;
    
    % 쿼터니언 변화율 계산
    dq = (q_t - q_t_minus) / delta_t;
    
    % 각속도 계산 (쿼터니언 변화율 이용)
    q_t_conj = [q_t(1), -q_t(2), -q_t(3), -q_t(4)];  % 쿼터니언의 켤레
    omega = 2 * quatmultiply(dq, q_t_conj);
    angular_rate = omega(2:4);  % 스칼라 부분 무시
    
    % 결과 출력
    linear_acceleration = linear_acceleration';
    angular_rate = angular_rate';
end

function result = quatmultiply(q, r)
    % 쿼터니언 곱셈 함수
    w0 = q(1); x0 = q(2); y0 = q(3); z0 = q(4);
    w1 = r(1); x1 = r(2); y1 = r(3); z1 = r(4);
    result = [
        -x0 * x1 - y0 * y1 - z0 * z1 + w0 * w1
         x0 * w1 + y0 * z1 - z0 * y1 + w0 * x1
        -x0 * z1 + y0 * w1 + z0 * x1 + w0 * y1
         x0 * y1 - y0 * x1 + z0 * w1 + w0 * z1
    ];
end