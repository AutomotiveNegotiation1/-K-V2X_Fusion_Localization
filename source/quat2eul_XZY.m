function [eulr_rad] = quat2eul_XZY(q)
% 쿼터니언 정의 (예시)
% q = [a, b, c, d]; % q = a + bi + cj + dk
% q = [q(2) q(3) q(4) q(1)];
q = [q(1) q(2) q(3) q(4)];

% 쿼터니언 정규화
q = q / norm(q);

% 오일러 각 계산 (XZY 순서)
roll = atan2(2*(q(1)*q(3) + q(2)*q(4)), 1 - 2*(q(2)^2 + q(3)^2));
pitch = asin(2*(q(1)*q(4) - q(2)*q(3)));
yaw = atan2(2*(q(1)*q(2) + q(3)*q(4)), 1 - 2*(q(3)^2 + q(4)^2));

eulr_rad = [roll;pitch;yaw];

% 라디안을 도(degree)로 변환
% roll_deg = rad2deg(roll);
% pitch_deg = rad2deg(pitch);
% yaw_deg = rad2deg(yaw);
% 
% % 결과 출력
% fprintf('Roll: %.2f°\n', roll_deg);
% fprintf('Pitch: %.2f°\n', pitch_deg);
% fprintf('Yaw: %.2f°\n', yaw_deg);