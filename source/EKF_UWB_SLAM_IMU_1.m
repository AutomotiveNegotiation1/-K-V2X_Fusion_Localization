function [PosHF, HeadingHF, GammHF, BetaHF] = EKF_UWB_SLAM_IMU_1(dT, acc, GyroD, IMUtime, PosUWB, HeadUWB, UWBtime, grav)

persistent XhatUWB
persistent firstRun
persistent P Q R H
persistent IMUtimePrev

if isempty(firstRun)
    firstRun = 1;
    XhatUWB = zeros(15,1);
    Q = eye(15)*1e-0;
    Q(1:6,1:6) = eye(6)*1e-0;
    Q(10:12,10:12) = eye(3)*1e-0;
    Q = Q * 1;
    P = eye(15);
    R = eye(6)*1e0;
    H = zeros(6,15);
    H(1,10) = 1;
    H(2,11) = 1;
    H(3,12) = 1;

    H(4,1) = 1;
    H(5,2) = 1;
    H(6,3) = 1;

    IMUtimePrev = 0;
end


XhatIMU = transpose(XhatUWB);
XhatIMU = PredEKF_3D_Simple(XhatIMU,acc,GyroD,dT);

if (IMUtimePrev - UWBtime) < 0 
    A = makePredA_3D_Simple(XhatUWB,acc,GyroD,dT);
    % [Zv,H] = EstEKF_Center_3D_Simple_1(Xbar);

    Xbar = XhatIMU;
    game = atan((acc(2)-Xbar(8))/(acc(3)-grav-Xbar(9)));
    bete = atan((acc(1)-Xbar(7))/sqrt((acc(2)-Xbar(8))^2+(acc(3)-grav-Xbar(9))^2));

    game = AngleMatching(Xbar(12),game);
    bete = AngleMatching(Xbar(11),bete);
    HeadUWB = AngleMatching(Xbar(10),HeadUWB);
    % game = AngleMatching(Xbar(3),game);
    % bete = AngleMatching(Xbar(2),bete);

    Zv = [HeadUWB;bete;game;real(PosUWB);imag(PosUWB);0];


    P = A*P*transpose(A)+Q;

    K = P*transpose(H)/(H*P*transpose(H)+R);

    XhatUWB = transpose(XhatIMU) + K*(Zv-H*transpose(XhatIMU));
    P = P - K*H*P;

    if isnan(K)
        PosHF = XhatIMU(1)+XhatIMU(2)*j;
        HeadingHF = XhatIMU(10);
        GammHF = XhatIMU(12);
        BetaHF = XhatIMU(11);
    else
        PosHF = XhatUWB(1)+XhatUWB(2)*j;
        HeadingHF = XhatUWB(10);
        GammHF = XhatUWB(12);
        BetaHF = XhatUWB(11);
    end
else
    PosHF = XhatIMU(1)+XhatIMU(2)*j;
    HeadingHF = XhatIMU(10);
    GammHF = XhatIMU(12);
    BetaHF = XhatIMU(11);
    XhatUWB = transpose(XhatIMU);
end

IMUtimePrev = IMUtime;
