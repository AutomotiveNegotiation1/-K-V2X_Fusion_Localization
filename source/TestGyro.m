
HHroll = 0;
HHpitch = 0;
HHyaw = 0;
s_time_prev = IMU_tot(1,1);
MatIMU = eye(3);
for dfd = 2 : size(IMU_tot,1)
    IMUAccT = IMU_tot(dfd,2:4);
    IMUgyroT = IMU_tot(dfd,5:7);
    
    if IMUSel == 2
        RotDiff = eulr2dcm(IMUgyroT*(IMU_tot(dfd,1)-s_time_prev)/(180/pi));
    else
        RotDiff = eulr2dcm(IMUgyroT*(IMU_tot(dfd,1)-s_time_prev));
    end

    MatIMU = MatIMU*RotDiff;

    EulIMU = dcm2eulr(MatIMU);

    % TempBB = MatIMU*[1;0;0];
    % HHyaw(dfd) = atan2(TempBB(1),TempBB(2));
    
    HHroll(dfd) = EulIMU(1);
    HHpitch(dfd) = EulIMU(2);
    HHyaw(dfd) = EulIMU(3);

    Ndf = round(abs(HHyaw(dfd-1)-HHyaw(dfd))/(2*pi));
    if abs(HHyaw(dfd-1)-HHyaw(dfd)) > pi
        if (HHyaw(dfd-1)-HHyaw(dfd))>0
            HHyaw(dfd) = HHyaw(dfd) + Ndf*2*pi;
        else
            HHyaw(dfd) = HHyaw(dfd) - Ndf*2*pi;
        end
    end
    % TempBB = dcm2eulr(MatIMU);
    % HHyaw(dfd) = TempBB(2); 
    s_time_prev = IMU_tot(dfd,1);
end

if IMUSel == 1
figure(19);plot(IMU_tot(:,1),-HHyaw,'.')
figure(20);plot(IMU_tot(:,1),HHpitch,'.')
figure(21);plot(IMU_tot(:,1),HHroll,'.')
else
figure(1019);plot(IMU_tot(:,1),-HHyaw,'.')
figure(1020);plot(IMU_tot(:,1),HHpitch,'.')
figure(1021);plot(IMU_tot(:,1),HHroll,'.')
end    
TT = find(HeadingHUWB~=0);

figure(22);hold off;plot(PosHUWBtime(TT)-PosHUWBtime(TT(1)),mod(-HeadingHUWB(TT),2*pi),'.');hold on;plot(IMU_tot(:,1)-PosHUWBtime(TT(1)),mod(-HHyaw,2*pi),'r.')