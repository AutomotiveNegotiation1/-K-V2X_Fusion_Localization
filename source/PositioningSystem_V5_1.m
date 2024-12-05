function [PositionOut] = PositioningSystem_V5_1(PositionVector)

persistent s_time_prev  s_time_vec xt_b yt_b P FiFoUWBpos FiFoUWBhead FiFoUWBtime FiFoSLAMpos FiFoSLAMhead FiFoSLAMtime FiFoSLAMposN FiFoSLAMheadN FiFoSLAMEulDiff
persistent  PrevPosHF PrevHeadingHF firstV RotMatn SLAMposPrev SLAMinitPos SLAMinitHead UWBPosAcc UWBPosInitAcc SLAMPosAcc sAFlag sA RotMatInit QuatInit SLAMposInit s SLAMSet
persistent FiFoIMUacc FiFoIMUgyro FiFoIMUtime
persistent PosUWBIMU HeadUWBIMU GammUWBIMU BetaUWBIMU
persistent grav FiFoSLAMUWBpos FiFoSLAMUWBhead FiFoSLAMUWBtime CAMPos
Front = 0;

if isempty (s_time_prev)
    s_time_prev = 0;
    PrevPosHF = 0+0*j;
    PrevHeadingHF = 0;

    s_time_vec = zeros(1,4);
    P = eye(15);

    FiFoUWBpos = zeros(1,100)+0*j;
    FiFoUWBhead = zeros(1,100);
    FiFoUWBtime = zeros(1,100);

    FiFoSLAMpos = zeros(3,100);
    FiFoSLAMhead = zeros(4,100);
    FiFoSLAMtime = zeros(1,100);

    FiFoSLAMUWBpos = zeros(1,100)+0*j;
    FiFoSLAMUWBhead = zeros(1,100);
    FiFoSLAMUWBtime = zeros(1,100);

    FiFoIMUacc = zeros(3,4);
    FiFoIMUgyro = zeros(3,4);
    FiFoIMUtime = zeros(1,4);

    FiFoSLAMposN = zeros(3,100);
    FiFoSLAMEulDiff = zeros(3,100);
    FiFoSLAMheadN = zeros(1,100);

    SLAMinitPos = [0;0;0];
    SLAMinitHead = 0;
    firstV = 0;

    UWBPosAcc = 0;
    UWBPosInitAcc = 0;
    SLAMPosAcc = 0;

    sAFlag = 0;
    % load("MapParams2.mat");

    QuatInit = zeros(1,4);
    SLAMSet = 0;

    grav = 9.85;
    if Front == 1
        CAMPos = (-0.065+0.77*j);
    else
        CAMPos = (-0.06-0.77*j);

    end
    if Front == 1
        s = 4.5839;
        roll =  -0.0060;
        pitch = -4.4647;
        yaw =  -0.0458;

    else
        s = 3.3027;
        roll = 0.0340;
        pitch = 1.5508;
        yaw = 0.0076;
        sA = [-0.0661102073281800	3.30239075120085	-0.112716735233562; -3.30859080761202	-0.213460995745502	-0.0178754852105826; -0.00759043157741070	0.0339592364423471	0.999394394425305];
        SLAMposInit = [48.2402110895784;     5.22590479979198; 0];
    end

end

% dT = 0.001;
kk = 1;
s_time = PositionVector(kk); kk = kk + 1;
SensorNum = PositionVector(kk); kk = kk + 1;

if SensorNum == 5
    PosHF = 0;
    HeadingHF = 0;
    PosHF5 = 0;
    HeadingHF5 = 0;
    GammHF = 0;
    BetaHF = 0;
    PosH = 0;
    HeadingH = 0;
    dt = 5e-3;
    % dt = 10e-3;
    IMUtime = s_time;
    IMUacc_c = PositionVector(3:5);
    IMUgyro_c = PositionVector(6:8);

    if IMUacc_c(1)~=0

        FiFoIMUacc = [FiFoIMUacc(:,2:end) transpose(IMUacc_c)];
        FiFoIMUgyro = [FiFoIMUgyro(:,2:end) transpose(IMUgyro_c)];

        % GyroD = mean(FiFoIMUgyro)*pi/180;
        % acc = mean(FiFoIMUacc);
        % GyroD = (FiFoIMUgyro(:,end))*pi/180;
        GyroD = (FiFoIMUgyro(:,end));
        acc = (FiFoIMUacc(:,end));
        acc(3) = acc(3) + grav;

        % [PosHF5, HeadingHF5, GammHF5, BetaHF5] = EKF_UWB_IMU_1(dt, acc, GyroD, IMUtime, FiFoUWBpos(end), -FiFoUWBhead(end), FiFoUWBtime(end), grav);
        [PosHF5, HeadingHF5, GammHF5, BetaHF5] = EKF_UWB_SLAM_IMU_1(dt, acc, GyroD, IMUtime, FiFoSLAMUWBpos(end), FiFoSLAMUWBhead(end), FiFoSLAMUWBtime(end), grav);
        % [PosHF5, HeadingHF5, GammHF5, BetaHF5] = EKF_UWB_SLAM_IMU_1(dt, acc, GyroD, IMUtime, FiFoSLAMUWBpos(end)-(-0.065+2.5*j)*exp(j*(FiFoSLAMUWBhead(end))), FiFoSLAMUWBhead(end), FiFoSLAMUWBtime(end), grav);

%         PosUWBIMU = [PosUWBIMU PosHF5];
%         HeadUWBIMU = [HeadUWBIMU HeadingHF5];
%         GammUWBIMU = [GammUWBIMU GammHF5];
%         BetaUWBIMU = [BetaUWBIMU BetaHF5];

    end
elseif SensorNum == 6 %% GPS
    PosHF = 0;
    HeadingHF = 0;
    GammHF = 0;
    BetaHF = 0;
    PosH = 0;
    HeadingH = 0;
    PosHF5 = 0;
    HeadingHF5 = 0;

elseif SensorNum == 7 %% SLAM
    PosHF = 0;
    HeadingHF = 0;
    GammHF = 0;
    BetaHF = 0;
    PosH = 0;
    HeadingH = 0;
    PosHF5 = 0;
    HeadingHF5 = 0;

    SLAMpos = PositionVector(3:5);
    SLAMorient = PositionVector(6:9);
    % SLAMorient = SLAMorient([4 1 2 3]);

    if (SLAMpos(1)~=0) && ((FiFoSLAMtime(end)-s_time)<-1e-3)
        FiFoSLAMpos = [FiFoSLAMpos(:,2:end) transpose(SLAMpos)];
        FiFoSLAMhead = [FiFoSLAMhead(:,2:end) transpose(SLAMorient)];
        FiFoSLAMtime = [FiFoSLAMtime(2:end) s_time];


        L = length(FiFoSLAMtime(FiFoSLAMtime~=0));

        lagT = 50e-3;

        if (FiFoSLAMtime(end-L+1)< (s_time-lagT)) && (L>2)
            curr_pos = zeros(1,3);
            prev_pos = zeros(1,3);

            curr_pos(1) = interp1(FiFoSLAMtime(end-L+1:end),FiFoSLAMpos(1,end-L+1:end),FiFoSLAMtime(end)-lagT,'spline');
            curr_pos(2) = interp1(FiFoSLAMtime(end-L+1:end),FiFoSLAMpos(2,end-L+1:end),FiFoSLAMtime(end)-lagT,'spline');
            curr_pos(3) = interp1(FiFoSLAMtime(end-L+1:end),FiFoSLAMpos(3,end-L+1:end),FiFoSLAMtime(end)-lagT,'spline');
            if (FiFoSLAMtime(end-L+1)< (FiFoSLAMtime(end-1)-lagT))
                prev_pos(1) = interp1(FiFoSLAMtime(end-L+1:end),FiFoSLAMpos(1,end-L+1:end),FiFoSLAMtime(end-1)-lagT,'spline');
                prev_pos(2) = interp1(FiFoSLAMtime(end-L+1:end),FiFoSLAMpos(2,end-L+1:end),FiFoSLAMtime(end-1)-lagT,'spline');
                prev_pos(3) = interp1(FiFoSLAMtime(end-L+1:end),FiFoSLAMpos(3,end-L+1:end),FiFoSLAMtime(end-1)-lagT,'spline');
            else
                prev_pos = curr_pos;
            end

        else
            curr_pos = transpose(FiFoSLAMpos(:,end));
            prev_pos = curr_pos;
        end

        SLAMtime = s_time;

        if  (SLAMpos(1)~=0)

            if firstV == 0
                if FiFoUWBpos(end)~=0
                    RotMatInit = eye(3);
                    QuatInit = SLAMorient;
                    firstV = 1;
                end
            else

                curr_quat = SLAMorient;

                prev_quat = transpose(FiFoSLAMhead(:,end-1));

                prev_pos_org = transpose(prev_pos);
                curr_pos_org = transpose(curr_pos);
                pos_diff_org = curr_pos_org - prev_pos_org;

                if sqrt(sum(abs(pos_diff_org).^2))<1
                    % RotMat_diff = (qua2dcm(curr_quat))/(qua2dcm(prev_quat));
                    RotMat_diff = (quat2rotm(curr_quat))/(quat2rotm(prev_quat));
                    EulCurr = equPlane(quat2eul(curr_quat));
                    EulPrev = quat2eul(Prev_quat);

                    TT = dcm2eulr(RotMat_diff);
                    if abs(TT(3)) > pi/4
                        OffsetEulr = [pi/2; pi/2; 0];
                    else
                        OffsetEulr = [0; 0; 0];
                    end
                    TempOffDcm = eulr2dcm(OffsetEulr);
                    RotEul_diff = dcm2eulr(RotMat_diff*TempOffDcm)-OffsetEulr;

                else
                    RotMat_diff = eye(3);
                    RotEul_diff = dcm2eulr(RotMat_diff);
                    pos_diff_org = zeros(3,1);
                    QuatInit = SLAMorient;
                    firstV = 2;
                end

                if Front == 1
                    FiFoSLAMEulDiff = [FiFoSLAMEulDiff(:,2:end) (RotEul_diff)];
                else
                    FiFoSLAMEulDiff = [FiFoSLAMEulDiff(:,2:end) (-RotEul_diff)];
                end

                if (FiFoSLAMtime(end-1)<(FiFoUWBtime(end)+lagT))

                    TT = quat2eul(SLAMorient);

                    TempOri = (qua2dcm(SLAMorient));
                    if abs(TT(1)) > pi/4
                        TempOriRot = qua2dcm(SLAMorient)*eulr2dcm([0; pi/2; pi/2]);
                        [tempEulr] = dcm2eulr(real(sA)*real(TempOriRot))-[0; pi/2; pi/2];
%                         [tempEulr] = dcm2eulr(TempOriRot*real(sA))-[0; pi/2; pi/2];

                        % [tempEulr] = dcm2eulr(TempOriRot*real(sA/s))-[0; pi/2; pi/2];
                        
                    elseif abs(TT(3)) > pi/4
                        TempOriRot = qua2dcm(SLAMorient)*eulr2dcm([pi/2; pi/2; 0]);
                        [tempEulr] = dcm2eulr(real(TempOriRot)*real(sA/s))-[pi/2; pi/2; 0];
                    else
                        tempEulr = dcm2eulr(TempOri*(sA/s));
                    end
                    if Front ~= 1
                        tempEulr(2) = tempEulr(2)+pi;
                    end

                    PosSLAM = sA*transpose(SLAMpos)+SLAMposInit;

                    % [EKFpos,EKFhead] = EKF_UWB_SLAM_4(FiFoUWBpos(end)+(-0.065+0.74*j)*exp(j*(FiFoUWBhead(end))), -FiFoUWBhead(end), pos_diff_org, RotEul_diff, QuatInit, PosSLAM, tempEulr);
                    if Front == 1
                        [EKFpos,EKFhead] = EKF_UWB_SLAM_4(FiFoUWBpos(end)+CAMPos*exp(j*(FiFoUWBhead(end))), -FiFoUWBhead(end), pos_diff_org, RotEul_diff, QuatInit, PosSLAM, tempEulr);
                    else
                        [EKFpos,EKFhead] = EKF_UWB_SLAM_4(FiFoUWBpos(end)+CAMPos*exp(j*(FiFoUWBhead(end))), -FiFoUWBhead(end), pos_diff_org, -RotEul_diff, QuatInit, PosSLAM, tempEulr);
                    end


                else
                    TempOri = qua2dcm(SLAMorient);
                    tempEulr = dcm2eulr(TempOri*(sA/s));
                    if Front ~= 1
                        tempEulr(2) = tempEulr(2)+pi;
                    end

                    PosSLAM = sA*transpose(SLAMpos)+SLAMposInit;
                    QuatInit = SLAMorient;
                    if Front == 1
                        [EKFpos,EKFhead] = EKF_UWB_SLAM_4(0,0, pos_diff_org, RotEul_diff, QuatInit, PosSLAM, tempEulr);
                    else
                        [EKFpos,EKFhead] = EKF_UWB_SLAM_4(0,0, pos_diff_org, -RotEul_diff, QuatInit, PosSLAM, tempEulr);
                    end

                end

                if (SLAMSet==0)
                    if (EKFpos~=0) && (abs(EKFpos-(FiFoUWBpos(end)+CAMPos*exp(j*(FiFoUWBhead(end)))))<0.5) && (FiFoUWBpos(end)~=0) % && (min(abs(mod((FiFoUWBhead(end)-EKFhead(2)),2*pi)),abs(mod((FiFoUWBhead(end)-EKFhead(2)),2*pi)-2*pi))<0.1)
                        % if (EKFpos~=0) && (abs(EKFpos-(FiFoUWBpos(end)+(-0.065+0.74*j)*exp(j*(FiFoUWBhead(end)))))<0.5) && (FiFoUWBpos(end)~=0) % && (min(abs(mod((FiFoUWBhead(end)-EKFhead(2)),2*pi)),abs(mod((FiFoUWBhead(end)-EKFhead(2)),2*pi)-2*pi))<0.1)
                        SLAMSet = 1;
                        % PosHF = EKFpos;
                        % HeadingHF = EKFhead(2);

                        PosHF = EKFpos-CAMPos*exp(-j*EKFhead(2));
                        HeadingHF = EKFhead(2);
                        FiFoSLAMUWBpos = [FiFoSLAMUWBpos(2:end) PosHF];
                        FiFoSLAMUWBhead = [FiFoSLAMUWBhead(2:end) HeadingHF];
                        FiFoSLAMUWBtime = [FiFoSLAMUWBtime(2:end) s_time];

                        PrevHeading = EKFhead;
                        PrevHeadingHF = EKFhead(2);
                    else
                        PosHF = 0;
                        HeadingHF = 0;
                        PrevHeading = 0;
                        PrevHeadingHF = 0;
                    end
                else
                    % PosHF = EKFpos;
                    % HeadingHF = EKFhead(2);
                    PosHF = EKFpos-CAMPos*exp(-j*EKFhead(2));
                    HeadingHF = EKFhead(2);
                        

                    FiFoSLAMUWBpos = [FiFoSLAMUWBpos(2:end) PosHF];
                    FiFoSLAMUWBhead = [FiFoSLAMUWBhead(2:end) HeadingHF];
                    FiFoSLAMUWBtime = [FiFoSLAMUWBtime(2:end) s_time];
                    PrevHeading = EKFhead;
                    PrevHeadingHF = EKFhead(2);
                end
            end
        end
    end

else %% UWB
    PosHF5 = 0;
    HeadingHF5 = 0;

    d_uwb = s_time - s_time_prev ;

    s_time_vec = [s_time_vec(2:4) s_time];

    Ln = PositionVector(kk); kk = kk + 1;
    LnC = PositionVector(kk); kk = kk + 1;
    Nanchor = PositionVector(kk); kk = kk + 1;
    RxIDUWB = PositionVector(kk:kk+Nanchor-1); kk = kk + Ln;
    RxDistOrig = PositionVector(kk:kk+Nanchor-1); kk = kk + 4;

    xain = PositionVector(kk:kk+LnC-1);  kk = kk + LnC;
    yain = PositionVector(kk:kk+LnC-1);  kk = kk + LnC;
    zain = PositionVector(kk:kk+LnC-1);  kk = kk + LnC;

    xt_b = PositionVector(kk:kk+4-1);  kk = kk + 4;
    yt_b = PositionVector(kk:kk+4-1);  kk = kk + 4;
    zt_b = PositionVector(kk); kk = kk + 1;

    % [PosH, Heading, UWBAnc_Full] = UWBPosition_V3(s_time, Ln, LnC, Nanchor, SensorNum, RxIDUWB, RxDistOrig, xain, yain, zain, xt_b, yt_b, zt_b);
    [PosH, HeadingH, UWBAnc_Full] = UWBPosition_V4_1(s_time, Ln, LnC, Nanchor, SensorNum, RxIDUWB, RxDistOrig, xain, yain, zain, xt_b, yt_b, zt_b, PrevPosHF, PrevHeadingHF);

    if PosH~=0

        FiFoUWBpos = [FiFoUWBpos(2:end) PosH];
        FiFoUWBhead = [FiFoUWBhead(2:end) HeadingH];
        FiFoUWBtime = [FiFoUWBtime(2:end) s_time];
    end

    if SLAMSet == 0
        PosHF = PosH;
        HeadingHF = -HeadingH;
        if PosHF~=0
            FiFoSLAMUWBpos = [FiFoSLAMUWBpos(2:end) PosHF];
            FiFoSLAMUWBhead = [FiFoSLAMUWBhead(2:end) HeadingHF];
            FiFoSLAMUWBtime = [FiFoSLAMUWBtime(2:end) s_time];
        end

        % PosHF = PosH + (-0.065+0.74*j)*exp(j*HeadingH);
    else
        PosHF = 0;
        HeadingHF = 0;
    end
    GammHF = 0;
    BetaHF = 0;

    %
    s_time_prev = s_time;
end

% figure(123412);hold on;plot(PosHF,'ro');

Uncertainty = P;
% PositionOut = [real(PosHF), imag(PosHF), 0, HeadingHF, GammHF, BetaHF, Uncertainty(1,1), real(PosHF5), imag(PosHF5), HeadingHF5  ];
PositionOut = [real(PosHF), imag(PosHF), 0, HeadingHF, GammHF, BetaHF, Uncertainty(1,1), real(PosH), imag(PosH), HeadingH  ];

% PositionOut = [real(PosHF5), imag(PosHF5), 0, -HeadingHF5, GammHF, BetaHF, Uncertainty(1,1), real(PosH), imag(PosH), HeadingH  ];
%
PrevPosHF = PosHF;
PrevHeadingHF = HeadingHF;





