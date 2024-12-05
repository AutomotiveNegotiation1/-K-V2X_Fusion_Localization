function [PositionOut] = PositioningSystem_V4_1(PositionVector)

persistent s_time_prev  s_time_vec xt_b yt_b P FiFoUWBpos FiFoUWBhead FiFoUWBtime FiFoSLAMpos FiFoSLAMhead FiFoSLAMtime FiFoSLAMposN FiFoSLAMheadN 
persistent  PrevPosHF PrevHeadingHF firstV RotMatn SLAMposPrev SLAMinitPos SLAMinitHead UWBPosAcc UWBPosInitAcc SLAMPosAcc sAFlag sA RotMatInit QuatInit SLAMposInit


N = 10;
if isempty (s_time_prev)
    % load('MapParams.mat','sA','SLAMposInit');
    s_time_prev = 0;
    PrevPosHF = 0+0*j;
    % PrevHeadingHF = [0 0 0];
    PrevHeadingHF = 0;

    s_time_vec = zeros(1,4);
    P = eye(15);

    FiFoUWBpos = zeros(1,100);
    FiFoUWBhead = zeros(1,100);
    FiFoUWBtime = zeros(1,100);

    FiFoSLAMpos = zeros(3,100);
    FiFoSLAMhead = zeros(4,100);
    FiFoSLAMtime = zeros(1,100);

    FiFoSLAMposN = zeros(3,100);
    FiFoSLAMheadN = zeros(1,100);

    SLAMinitPos = [0;0;0];
    SLAMinitHead = 0;
    firstV = 0;

    UWBPosAcc = 0;
    UWBPosInitAcc = 0;
    SLAMPosAcc = 0;

    sAFlag = 0;
    load("MapParams2.mat");
end

% dT = 0.001;
kk = 1;
s_time = PositionVector(kk); kk = kk + 1;
SensorNum = PositionVector(kk); kk = kk + 1;

if SensorNum == 5
    PosHF = 0;
    HeadingHF = 0;
    GammHF = 0;
    BetaHF = 0;
    PosH = 0;
    HeadingH = 0;

elseif SensorNum == 6 %% GPS
    PosHF = 0;
    HeadingHF = 0;
    GammHF = 0;
    BetaHF = 0;
    PosH = 0;
    HeadingH = 0;
elseif SensorNum == 7 %% SLAM
    PosHF = 0;
    HeadingHF = 0;
    GammHF = 0;
    BetaHF = 0;
    PosH = 0;
    HeadingH = 0;

    SLAMpos = PositionVector(3:5);
    SLAMorient = PositionVector(6:9);
    % SLAMorient = SLAMorient([4 1 2 3]);

    if SLAMpos(1)~=0
        FiFoSLAMpos = [FiFoSLAMpos(:,2:end) transpose(SLAMpos)];
        FiFoSLAMhead = [FiFoSLAMhead(:,2:end) transpose(SLAMorient)];
        FiFoSLAMtime = [FiFoSLAMtime(2:end) s_time];

        L = length(FiFoSLAMtime(FiFoSLAMtime~=0));

        lagT = 0e-3;
        % lagT = 0;
        if (FiFoSLAMtime(end-L+1)< (s_time-lagT)) && (L>2)
            curr_pos(1) = interp1(FiFoSLAMtime(end-L+1:end),FiFoSLAMpos(1,end-L+1:end),FiFoUWBtime(end)+lagT);
            curr_pos(2) = interp1(FiFoSLAMtime(end-L+1:end),FiFoSLAMpos(2,end-L+1:end),FiFoUWBtime(end)+lagT);
            curr_pos(3) = interp1(FiFoSLAMtime(end-L+1:end),FiFoSLAMpos(3,end-L+1:end),FiFoUWBtime(end)+lagT);
            prev_pos(1) = interp1(FiFoSLAMtime(end-L+1:end),FiFoSLAMpos(1,end-L+1:end),FiFoUWBtime(end-1)+lagT);
            prev_pos(2) = interp1(FiFoSLAMtime(end-L+1:end),FiFoSLAMpos(2,end-L+1:end),FiFoUWBtime(end-1)+lagT);
            prev_pos(3) = interp1(FiFoSLAMtime(end-L+1:end),FiFoSLAMpos(3,end-L+1:end),FiFoUWBtime(end-1)+lagT);

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
                % curr_pos = transpose(sA*transpose(SLAMpos)+SLAMposInit);
                % curr_pos = SLAMpos;
                curr_time = SLAMtime;
                curr_quat = SLAMorient;

                % prev_pos = transpose(sA*FiFoSLAMpos(:,end-1)+SLAMposInit);
                % prev_pos = transpose(FiFoSLAMpos(:,end-1));
                prev_time = FiFoSLAMtime(end-1);
                prev_quat = transpose(FiFoSLAMhead(:,end-1));

                prev_pos_org = transpose(prev_pos);
                curr_pos_org = transpose(curr_pos);
                pos_diff_org = curr_pos_org - prev_pos_org;

                if sqrt(mean(abs(pos_diff_org).^2))<0.5
                    RotMat_diff = (qua2dcm(curr_quat))/(qua2dcm(prev_quat));
                    % RotEul_diff = quatmultiply(curr_quat,quatconj(prev_quat));

                    TempPP = RotMat_diff*[1;0;0];

                    OffsetEulr = [pi/2; 0; 0];
                    TempOffDcm = eulr2dcm(OffsetEulr);
                    RotEul_diff = dcm2eulr(RotMat_diff*TempOffDcm)-OffsetEulr;
                else
                    RotMat_diff = eye(3);
                    RotEul_diff = dcm2eulr(RotMat_diff);
                    pos_diff_org = zeros(3,1);
                    QuatInit = SLAMorient;
                    
                end
                    TempOri = quat2eul(SLAMorient);
                    % eulerAnglesRandians = quat2eul(SLAMorient);
                    % 
                    % TempOriDcm = qua2dcm(SLAMorient);
                    % OffsetEulr = [pi/2; 0; 0];
                    % TempOffDcm = eulr2dcm(OffsetEulr);
                    % TempRotDcm = TempOriDcm*TempOffDcm;
                    % TempRotEulr = dcm2eulr(TempRotDcm)-OffsetEulr;
                    % 
                    % 
                    % Tme = TempRotDcm*[1;0;0]
                    % Tmp = TempOriDcm*[1;0;0]
                    % Tmq = TempOffDcm*[1;0;0]
                    % figure(5);hold on;plot(SLAMtime,FiFoUWBhead(end),'b.');
                    % plot(SLAMtime,mod(TempOri(1)+pi,2*pi)-pi,'r.');plot(TempOri,mod(RotEul_diff(2)+pi,2*pi)-pi,'g.');plot(TempOri,mod(RotEul_diff(3)+pi,2*pi)-pi,'k.');
                    % 
                    % figure(6);hold on;plot(SLAMtime,FiFoUWBhead(end),'b.');
                    % plot(SLAMtime,mod(TempRotEulr(1)+pi,2*pi)-pi,'r.');plot(SLAMtime,mod(TempRotEulr(2)+pi,2*pi)-pi,'g.');plot(SLAMtime,mod(TempRotEulr(3)+pi,2*pi)-pi,'k.');
                    % 
                    % figure(7);hold on;plot(SLAMtime,FiFoUWBhead(end),'b.');
                    % plot(SLAMtime,mod(eulerAnglesRandians(1)+pi,2*pi)-pi,'r.');plot(SLAMtime,mod(eulerAnglesRandians(2)+pi,2*pi)-pi,'g.');plot(SLAMtime,mod(eulerAnglesRandians(3)+pi,2*pi)-pi,'k.');
                    % 
                    
                    % figure(6);hold on;plot(SLAMtime,TempOriVec(1),'b.');
                    % plot(SLAMtime,TempOriVec(2),'g.');plot(SLAMtime,TempOriVec(3),'k.');




                if FiFoSLAMtime(end-1)<FiFoUWBtime(end)
                    

                    % figure(4);hold off;plot(FiFoUWBpos(end),'ro');hold on;plot(FiFoUWBpos(end)+(-0.065-j*0.74)*exp(j*(FiFoUWBhead(end))),'bo');axis equal
                    % figure(5);hold on;plot(SLAMtime,FiFoUWBhead(end),'b.');plot(SLAMtime,TempOri(1),'r.');
                    % plot(SLAMtime,TempOri(2),'g.');plot(SLAMtime,TempOri(3),'k.');

                    [EKFpos,EKFhead] = EKF_UWB_SLAM_3(FiFoUWBpos(end)+(-0.065+0.74*j)*exp(j*(-FiFoUWBhead(end))), -FiFoUWBhead(end), pos_diff_org, RotEul_diff, QuatInit);
                    % [EKFpos,EKFhead] = EKF_UWB_SLAM_2(FiFoUWBpos(end)+(-0.065+j*0.74)*exp(j*(-FiFoUWBhead(end))), -FiFoUWBhead(end), pos_diff_org, dcm2eulr(RotMat_diff), QuatInit);
                    % [EKFpos,EKFhead] = EKF_UWB_SLAM(FiFoUWBpos(end)+1*exp(j*(pi/2+FiFoUWBhead(end))), -FiFoUWBhead(end), pos_diff_org, dcm2eulr(RotMat_diff), QuatInit);

                else
                    TempOri = quat2eul(SLAMorient);
                    % figure(5);hold on;plot(SLAMtime,TempOri(2),'r.');
                    %                     plot(SLAMtime,TempOri(2),'g.');plot(SLAMtime,TempOri(3),'k.');

                    [EKFpos,EKFhead] = EKF_UWB_SLAM_3(0, 0, pos_diff_org, RotEul_diff, QuatInit);
                    % [EKFpos,EKFhead] = EKF_UWB_SLAM(0, 0, pos_diff_org, dcm2eulr(RotMat_diff), QuatInit);
                end
                PosHF = EKFpos;
                HeadingHF = EKFhead(2);
                PrevHeading = EKFhead;
                PrevHeadingHF = EKFhead(2);
            end
        end
    end


else %% UWB
    
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

    PosHF = 0;
    HeadingHF = 0;
    GammHF = 0;
    BetaHF = 0;

    % 
    s_time_prev = s_time;
end


Uncertainty = P;    
PositionOut = [real(PosHF), imag(PosHF), 0, HeadingHF, GammHF, BetaHF, Uncertainty(1,1), real(PosH), imag(PosH), HeadingH  ];
PrevPosHF = PosHF;
PrevHeadingHF = HeadingHF;





