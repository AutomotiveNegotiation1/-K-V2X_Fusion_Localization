function [eulr_rad] = quaternionToEulerYXZ(q, XX)
% Extract the values from quaternion
[M,N] = size(q);
if M == 4
    Cnt = N;
    q = transpose(q);
else
    Cnt = M;
    
end
eulr_rad = zeros(Cnt,3);

for gg = 1 : Cnt
    w = q(gg,1);
    x = q(gg,2);
    y = q(gg,3);
    z = q(gg,4);
    %
    % w = q(4);
    % x = q(1);
    % y = q(2);
    % z = q(3);

    roll = atan2(2 * (w * x + y * z), 1 - 2 * (y^2 + z^2));
    yaw = atan2(2 * (w * z + x * y), 1 - 2 * (x^2 + y^2));
    pitch = asin(2 * (w * y - x * z));

    Temp = [roll pitch yaw];

        
        TT = dcm2eulr(eulr2dcm(Temp)*XX);
        if abs(TT(3))<pi/2
            TT(3) = pi+TT(3);
            TT(2) = pi-TT(2);
            TT(1) = pi-TT(1);
        end

        TTs = dcm2eulr(eulr2dcm(Temp)*XX*eulr2dcm([0 pi/2 0]))+[0; pi/2; 0];
        if abs(TTs(3))<pi/2
            TTs(3) = pi-TTs(3);
            TTs(2) = 2*pi-TTs(2);
            TTs(1) = pi-TTs(1);
        % elseif abs(TTs(gd,3))>7*pi/4
        % 
        %     TTs(gd,3) = pi+TTs(gd,3);
        end

        TTo = zeros(3,1);
        if (abs(mod(TTs(2),2*pi))<pi/4) || ((abs(mod(TTs(2),2*pi))>3*pi/4)&&(abs(mod(TTs(2),2*pi))<=5*pi/4)) ||(abs(mod(TTs(2),2*pi))>=7*pi/4)
            TTo(:) = TT(:);
        else
            TTo(:) = TTs(:);
        end
            
    eulr_rad(gg,1) = TTo(1);
    eulr_rad(gg,2) = TTo(2);
    eulr_rad(gg,3) = TTo(3);
end
