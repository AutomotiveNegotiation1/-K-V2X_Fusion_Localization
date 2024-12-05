function [PositionOut] = PositioningSystem_V4_2(PositionVector)

persistent s_time_prev  s_time_vec xt_b yt_b P FiFoUWBpos FiFoUWBhead FiFoUWBtime FiFoSLAMpos FiFoSLAMhead FiFoSLAMtime FiFoSLAMposN FiFoSLAMheadN FiFoSLAMEulDiff
persistent  PrevPosHF PrevHeadingHF firstV RotMatn SLAMposPrev SLAMinitPos SLAMinitHead UWBPosAcc UWBPosInitAcc SLAMPosAcc sAFlag sA RotMatInit QuatInit SLAMposInit s SLAMSet


N = 10;
if isempty (s_time_prev)
    % load('MapParams.mat','sA','SLAMposInit');
    s_time_prev = 0;
    PrevPosHF = 0+0*j;
    % PrevHeadingHF = [0 0 0];
    PrevHeadingHF = 0;

    s_time_vec = zeros(1,4);
    P = eye(15);

    FiFoUWBpos = zeros(1,100)+0*j;
    FiFoUWBhead = zeros(1,100);
    FiFoUWBtime = zeros(1,100);

    FiFoSLAMpos = zeros(3,100);
    FiFoSLAMhead = zeros(4,100);
    FiFoSLAMtime = zeros(1,100);

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
    s = 4.6;
    sA = s*eulr2dcm([pi*0 -pi*0.57 pi*0]);

    SLAMposInit = [40.572;0;4.2114];

    SLAMSet = 0;
    % roll = 0;
    % pitch = -pi*0.57;
    % yaw = 0;
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

    if (SLAMpos(1)~=0) && ((FiFoSLAMtime(end)-s_time)<-1e-3)
        FiFoSLAMpos = [FiFoSLAMpos(:,2:end) transpose(SLAMpos)];
        FiFoSLAMhead = [FiFoSLAMhead(:,2:end) transpose(SLAMorient)];
        FiFoSLAMtime = [FiFoSLAMtime(2:end) s_time];


        L = length(FiFoSLAMtime(FiFoSLAMtime~=0));

        lagT = 50e-3;
        % lagT = 250e-3;

        % if L > 1
        %     curr_pos = SLAMpos;
        %     prev_pos = transpose(FiFoSLAMpos(:,end-1));
        % else
        %     curr_pos = SLAMpos;
        %     prev_pos = curr_pos;
        % end
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
                    RotMat_diff = (qua2dcm(curr_quat))/(qua2dcm(prev_quat));
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

                FiFoSLAMEulDiff = [FiFoSLAMEulDiff(:,2:end) (RotEul_diff)];

                % RotEul_diff_interp = zeros(1,3);
                % RotEul_diff_interp(1) = interp1(FiFoSLAMtime(end-L+1:end),FiFoSLAMEulDiff(1,end-L+1:end),FiFoUWBtime(end)+lagT);
                % RotEul_diff_interp(2) = interp1(FiFoSLAMtime(end-L+1:end),FiFoSLAMEulDiff(2,end-L+1:end),FiFoUWBtime(end)+lagT);
                % RotEul_diff_interp(3) = interp1(FiFoSLAMtime(end-L+1:end),FiFoSLAMEulDiff(3,end-L+1:end),FiFoUWBtime(end)+lagT);



                % if (FiFoSLAMtime(end)<(FiFoUWBtime(end)+lagT)) && (FiFoSLAMtime(end-1)<(FiFoUWBtime(end)-lagT))
                if (FiFoSLAMtime(end-1)<(FiFoUWBtime(end)+lagT))

                    % Luwb = length(FiFoSLAMpos(1,FiFoSLAMpos(1,:)~=0));
                    % if (Luwb > 1) & (FiFoUWBtime(end-Luwb+1)<(FiFoSLAMtime(end)-lagT))
                    %     UWBrePos = interp1(FiFoUWBtime(end-Luwb+1:end),real(FiFoUWBpos(1,end-Luwb+1:end)),FiFoSLAMtime(end)-lagT,'cubic','extrap');
                    %     UWBrePos = UWBrePos+j*(interp1(FiFoUWBtime(end-Luwb+1:end),imag(FiFoUWBpos(end-Luwb+1:end)),FiFoSLAMtime(end)-lagT,'cubic','extrap'));
                    %
                    %
                    %     HeadingHUWBcont(1) = FiFoUWBhead(1);
                    %     for dge = 2 : 100
                    %         teta = (HeadingHUWBcont(dge-1)-FiFoUWBhead(dge));
                    %         if teta >= 2*pi
                    %             ncount = floor(teta/(2*pi));
                    %             HeadingHUWBcont(dge) = FiFoUWBhead(dge)+2*pi*ncount;
                    %         elseif teta <= -2*pi
                    %             ncount = floor(-teta/(2*pi));
                    %             HeadingHUWBcont(dge) = FiFoUWBhead(dge)-2*pi*ncount;
                    %         else
                    %             HeadingHUWBcont(dge) = FiFoUWBhead(dge);
                    %         end
                    %         if (HeadingHUWBcont(dge)-HeadingHUWBcont(dge-1))>pi
                    %             HeadingHUWBcont(dge) = HeadingHUWBcont(dge)-2*pi;
                    %         elseif (HeadingHUWBcont(dge)-HeadingHUWBcont(dge-1))<-pi
                    %             HeadingHUWBcont(dge) = HeadingHUWBcont(dge)+2*pi;
                    %         end
                    %     end
                    %
                    %     UWBreHead = interp1(FiFoUWBtime(end-Luwb+1:end),HeadingHUWBcont(end-Luwb+1:end),FiFoSLAMtime(end)-lagT,'cubic','extrap');
                    %     % UWBreHead = FiFoUWBhead(end);
                    %
                    % else
                    %     UWBrePos = FiFoUWBpos(end);
                    %     UWBreHead = FiFoUWBhead(end);
                    % end
                    TT = quat2eul(SLAMorient);
                    % quat = quaternion(SLAMorient);
                    % TT2 = euler(quat,"YXZ","point")

                    TempOri = (qua2dcm(SLAMorient));
                    if abs(TT(1)) > pi/4
                        TempOriRot = qua2dcm(SLAMorient)*eulr2dcm([0; pi/2; pi/2]);
                        [tempEulr] = dcm2eulr(TempOriRot*real(sA/s))-[0; pi/2; pi/2];

                    elseif abs(TT(3)) > pi/4
                        TempOriRot = qua2dcm(SLAMorient)*eulr2dcm([pi/2; pi/2; 0]);
                        [tempEulr] = dcm2eulr(TempOriRot*real(sA/s))-[pi/2; pi/2; 0];
                    else
                        tempEulr = dcm2eulr(TempOri*(sA/s));
                    end
                    PosSLAM = sA*transpose(SLAMpos)+SLAMposInit;

                    % [EKFpos,EKFhead] = EKF_UWB_SLAM_4(FiFoUWBpos(end)+(-0.065+0.74*j)*exp(j*(FiFoUWBhead(end))), -FiFoUWBhead(end), pos_diff_org, RotEul_diff_interp, QuatInit, PosSLAM, tempEulr);
                    [EKFpos,EKFhead] = EKF_UWB_SLAM_4(FiFoUWBpos(end)+(-0.065+0.74*j)*exp(j*(FiFoUWBhead(end))), -FiFoUWBhead(end), pos_diff_org, RotEul_diff, QuatInit, PosSLAM, tempEulr);

                    % if abs(EKFpos-FiFoUWBpos(end))>0.2
                    %     % EKFpos = FiFoUWBpos(end);
                    %     % EKFhead = [0;-FiFoUWBhead(end);0];
                    %     [EKFpos,EKFhead] = EKF_UWB_SLAM_4(PosSLAM(1)+j*PosSLAM(3), tempEulr(2), pos_diff_org, RotEul_diff, QuatInit, PosSLAM, tempEulr);
                    % end

                    % [EKFpos,EKFhead] = EKF_UWB_SLAM_3(UWBrePos+(-0.065+0.74*j)*exp(j*(UWBreHead)), -UWBreHead, pos_diff_org, RotEul_diff, QuatInit);



                    % [EKFpos,EKFhead] = EKF_UWB_SLAM_2(FiFoUWBpos(end)+(-0.065+j*0.74)*exp(j*(-FiFoUWBhead(end))), -FiFoUWBhead(end), pos_diff_org, dcm2eulr(RotMat_diff), QuatInit);
                    % [EKFpos,EKFhead] = EKF_UWB_SLAM(FiFoUWBpos(end)+1*exp(j*(pi/2+FiFoUWBhead(end))), -FiFoUWBhead(end), pos_diff_org, dcm2eulr(RotMat_diff), QuatInit);

                else
                    TempOri = qua2dcm(SLAMorient);
                    % if abs(TT(1)) > pi/4
                    %     TempOriRot = qua2dcm(SLAMorient)*eulr2dcm([0; pi/2; pi/2]);
                    %     [tempEulr] = dcm2eulr(TempOriRot*real(sA/s))-[0; pi/2; pi/2];
                    %
                    % elseif abs(TT(3)) > pi/4
                    %     TempOriRot = qua2dcm(SLAMorient)*eulr2dcm([pi/2; pi/2; 0]);
                    %     [tempEulr] = dcm2eulr(TempOriRot*real(sA/s))-[pi/2; pi/2; 0];
                    % else
                    tempEulr = dcm2eulr(TempOri*(sA/s));
                    % end

                    PosSLAM = sA*transpose(SLAMpos)+SLAMposInit;
                    QuatInit = SLAMorient;
                    [EKFpos,EKFhead] = EKF_UWB_SLAM_4(0,0, pos_diff_org, RotEul_diff, QuatInit, PosSLAM, tempEulr);
                    % [EKFpos,EKFhead] = EKF_UWB_SLAM_4(0,0, pos_diff_org, RotEul_diff_interp, QuatInit, PosSLAM, tempEulr);

                    % [EKFpos,EKFhead] = EKF_UWB_SLAM_3(PosSLAM(1)+j*PosSLAM(3),tempEulr(2), pos_diff_org, RotEul_diff, QuatInit);

                    % [EKFpos,EKFhead] = EKF_UWB_SLAM(0, 0, pos_diff_org, dcm2eulr(RotMat_diff), QuatInit);
                end
                TempPos = FiFoUWBpos(end)+1*exp(j*(pi/2+FiFoUWBhead(end)));
                % if (abs(EKFpos-TempPos)>0.1) && (FiFoUWBpos(end)~=0) && (firstV<2)
                %     PosHF = FiFoUWBpos(end);
                %     HeadingHF = -FiFoUWBhead(end);

                % end

                if (SLAMSet==0)
                    if (EKFpos~=0) && (abs(EKFpos-FiFoUWBpos(end))<0.5) && (FiFoUWBpos(end)~=0) % && (min(abs(mod((FiFoUWBhead(end)-EKFhead(2)),2*pi)),abs(mod((FiFoUWBhead(end)-EKFhead(2)),2*pi)-2*pi))<0.1)
                        SLAMSet = 1;
                        PosHF = EKFpos;
                        HeadingHF = EKFhead(2);
                        PrevHeading = EKFhead;
                        PrevHeadingHF = EKFhead(2);
                    else
                        PosHF = 0;
                        HeadingHF = 0;
                        PrevHeading = 0;
                        PrevHeadingHF = 0;
                    end
                else
                    PosHF = EKFpos;
                    HeadingHF = EKFhead(2);
                    PrevHeading = EKFhead;
                    PrevHeadingHF = EKFhead(2);
                end
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

    if SLAMSet == 0
        PosHF = PosH;
        HeadingHF = -HeadingH;
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
PositionOut = [real(PosHF), imag(PosHF), 0, HeadingHF, GammHF, BetaHF, Uncertainty(1,1), real(PosH), imag(PosH), HeadingH  ];
PrevPosHF = PosHF;
PrevHeadingHF = HeadingHF;





