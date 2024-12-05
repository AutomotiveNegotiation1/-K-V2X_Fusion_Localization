function [PositionOut] = PositioningSystem_V4_1(PositionVector)

persistent s_time_prev  s_time_vec xt_b yt_b P FiFoUWBpos FiFoUWBhead FiFoUWBtime FiFoSLAMpos FiFoSLAMhead FiFoSLAMtime FiFoSLAMposN FiFoSLAMheadN 
persistent  PrevPosHF PrevHeadingHF firstV RotMatn SLAMposPrev SLAMinitPos SLAMinitHead UWBPosAcc UWBPosInitAcc SLAMPosAcc sAFlag sA RotMatInit

N = 10;
if isempty (s_time_prev)
    s_time_prev = 0;
    PrevPosHF = 0+0*j;
    PrevHeadingHF = 0;

    s_time_vec = zeros(1,4);
    P = eye(15);

    FiFoUWBpos = zeros(1,100);
    FiFoUWBhead = zeros(1,100);
    FiFoUWBtime = zeros(1,100);

    FiFoSLAMpos = zeros(3,2);
    FiFoSLAMhead = zeros(4,2);
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
    sA  = 0;
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

    if SLAMpos(1)~=0
        FiFoSLAMpos = [FiFoSLAMpos(:,2:end) transpose(SLAMpos)];
        FiFoSLAMhead = [FiFoSLAMhead(:,2:end) transpose(SLAMorient)];
        FiFoSLAMtime = [FiFoSLAMtime(2:end) s_time];
        SLAMtime = s_time;
        if  (SLAMpos(1)~=0)

            if firstV == 0
                if FiFoUWBpos(end)~=0
                    % FiFoUWBposEnd = FiFoUWBpos(end);
                    % FiFoUWBheadEnd = FiFoUWBhead(end);
                    % RotMatn = eye(3);
                    % RotMatn =[cos(FiFoUWBheadEnd) 0 sin(FiFoUWBheadEnd);0 1 0;-sin(FiFoUWBheadEnd) 0 cos(FiFoUWBheadEnd)];
                    
                    % SLAMposPrev = [real(FiFoUWBposEnd);imag(FiFoUWBposEnd);0];
                    % SLAMposPrev = zeros(3,1);
                    % SLAMinitPos = FiFoUWBposEnd
                    % SLAMinitHead = FiFoUWBheadEnd;
                    RotMatInit = inv(qua2dcm(SLAMorient));
                    RotMatInit = eye(3);
                    % RotMatn = qua2dcm(SLAMorient);
                    firstV = 1;
                end
            else
                curr_pos = SLAMpos;
                curr_time = SLAMtime;
                curr_quat = SLAMorient;

                prev_pos = transpose(FiFoSLAMpos(:,end-1));
                prev_time = FiFoSLAMtime(end-1);
                prev_quat = transpose(FiFoSLAMhead(:,end-1));

                RotMat = RotMatInit*qua2dcm(prev_quat);
                prev_pos_org = RotMat'*transpose(prev_pos);
                curr_pos_org = RotMat'*transpose(curr_pos);
                pos_diff_org = curr_pos_org - prev_pos_org;

                if sqrt(mean(abs(pos_diff_org).^2))<0.5
                    RotMat_diff = (qua2dcm(curr_quat))/(qua2dcm(prev_quat));
                    
                    % SLAMposPrev = SLAMposPrev + RotMatn*pos_diff_org;
                    % RotMatn = RotMat_diff*RotMatn;
                else
                    RotMat_diff = eye(3);
                    pos_diff_org = zeros(3,1);
                    % RotMatn = RotMatn;

                    % RotMatInit = inv(qua2dcm(SLAMorient));
                    % SLAMposPrev = SLAMposPrev;
                    % TE = quat2eul(SLAMorient);
                    % TD = dcm2eulr(RotMatn);
                    % TE(2) = TD(2);
                    % RotMatn = eulr2dcm(TE);
            
                end
                % SLAMposN = SLAMposPrev;
                % dcmSLAM = RotMatn;
                % temp = dcmSLAM*[1;0;0];
                % edl = angle(temp(1)+j*temp(3));
                % HeadingSLAMN = edl;
                
                % FiFoSLAMposN = [FiFoSLAMposN(:,2:end) (SLAMposN)];
                % FiFoSLAMheadN = [FiFoSLAMheadN(2:end) (HeadingSLAMN)];
                
                if FiFoSLAMtime(end-1)<FiFoUWBtime(end)
                    % SLAMposInterp(1,:) = interp1(FiFoSLAMtime(end-1:end),FiFoSLAMposN(1,(end-1:end)),FiFoUWBtime(end));
                    % SLAMposInterp(2,:) = interp1(FiFoSLAMtime(end-1:end),FiFoSLAMposN(3,(end-1:end)),FiFoUWBtime(end));
                    % SLAMposInterp(3,:) = interp1(FiFoSLAMtime(end-1:end),FiFoSLAMposN(2,(end-1:end)),FiFoUWBtime(end));
                    
                    % if sqrt(mean(abs(pos_diff_org).*2)) > 0.5
                    %     pos_diff_org = zeros(3,1);
                    %     RotMat_diff = eye(3);
                    % end

                    [EKFpos,EKFhead] = EKF_UWB_SLAM(FiFoUWBpos(end)+1.2*exp(j*(pi/2+FiFoUWBhead(end))), -FiFoUWBhead(end), pos_diff_org, dcm2eulr(RotMat_diff));

                    % UWBPosAcc = UWBPosAcc+ FiFoUWBpos(end);
                    % UWBPosInitAcc = UWBPosInitAcc + SLAMinitPos;
                    % SLAMPosAcc = SLAMPosAcc + SLAMposInterp(1)+j*SLAMposInterp(2);
                    % if (abs(SLAMPosAcc)>100)
                    %     sA = (UWBPosAcc-UWBPosInitAcc)/SLAMPosAcc;
                    %     % sA = 8.3*exp(j*pi);
                    %     sAFlag = 1;
                    % end
                else
                    [EKFpos,EKFhead] = EKF_UWB_SLAM(0, 0, pos_diff_org, dcm2eulr(RotMat_diff));
                end
                % if sAFlag == 0
                %     PosHF = FiFoUWBpos(end);
                %     HeadingH = FiFoUWBhead(end);
                % else
                    PosHF = EKFpos;
                    HeadingHF = EKFhead;
                    % PosHF = sA*(SLAMposN(1)+j*SLAMposN(3))+SLAMinitPos;
                    % % PosHF = (SLAMposN(1)+j*SLAMposN(3));
                    % HeadingH = HeadingSLAMN+angle(sA);
                % end
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





