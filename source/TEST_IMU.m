
IMUtimePrev = IMU_tot(1,1);
IMUposPrev = [0;0;0];
EulerSaved = zeros(1,3);
Pos = 0;
Vel = 0;
for dg = 2 : size(IMU_tot,1)
    
    IMUtime = IMU_tot(dg,1);
    IMUTimeDiff(dg)=IMUtime-IMUtimePrev;
    IMUtimePrev = IMUtime;
    ax = IMU_tot(dg,2)+0.38;
    ay = IMU_tot(dg,3)-0.073;
    az = IMU_tot(dg,4);

    p = IMU_tot(dg,5);
    q = IMU_tot(dg,6);
    r = IMU_tot(dg,7);
    
    [phi_a, theta_a] = EulerAccel2(ax, ay, az); 

    [phi, theta, psi] = EulerEKF([phi_a theta_a]', [p q r], IMUTimeDiff(dg));
  
    % ay = tan(phi)*az;
    % ax = tan(theta)*sqrt(ay^2+az^2);

    EulerSaved(dg, :) = [ mod(phi,2*pi) mod(theta,2*pi) mod(psi,2*pi) ];
    
    Vel(dg) = Vel(dg-1) + (ax+j*ay)*exp(-j*psi)*IMUTimeDiff(dg);
    Pos(dg) = Pos(dg-1) + Vel(dg-1)*IMUTimeDiff(dg) + 1/2*(ax+j*ay)*exp(-j*psi)*IMUTimeDiff(dg)^2;
    Acc(dg) = ax+j*ay;
end

figure(501);hold off;plot(IMU_tot(:,1),EulerSaved(:,1),'.r');hold on;plot(IMU_tot(:,1),EulerSaved(:,2),'.g');plot(IMU_tot(:,1),EulerSaved(:,3),'.b')
% figure(500);plot(IMUTimeDiff,'.')
figure(502);plot(Pos,'.');
figure(503);hold off;plot(real(Vel),'.');hold on;plot(imag(Vel),'r.');
figure(504);hold off;plot(real(Acc),'.');hold on;plot(imag(Acc),'r.');
