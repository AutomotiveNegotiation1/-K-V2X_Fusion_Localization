function [PositionOut] = PositioningSystem_V3_1(PositionVector)

persistent s_time_prev prev_state IMUacc_c_fifo IMUgyro_c_fifo XhatUWB XhatIMU PrevPosHUWB PrevHeadingHUWB s_time_vec PosHR AnchorR UWBDataSet prevUWBDataSet xt_b yt_b P UWBAnc_Full
persistent grav PrevBetaHUWB PrevGammaHUWB s_time_IMU_prev PrevPosIMU PrevHeadingIMU cnt PosH_vec PosH_vec_stime PrevPosHF PrevHeadingHF HeadingH_vec
N = 10;
if isempty (s_time_prev)
    s_time_prev = 0;
    s_time_IMU_prev = 0;
    IMUacc_c_fifo = zeros(4,3);
    IMUgyro_c_fifo = zeros(4,3);
    XhatUWB = zeros(15,1);
    XhatIMU = zeros(1,15);
    
    prev_state = 0; % 1 : UWB, 2 : IMU
    PrevPosHUWB = 0+0*j;
    PrevHeadingHUWB = 0;
    PrevBetaHUWB = 0;
    PrevGammaHUWB = 0;

    PrevPosHF = 0+0*j;
    PrevHeadingHF = 0;

    s_time_vec = zeros(1,4);
    PosHR = zeros(1,10)+0*j;
    AnchorR= zeros(1,10);

    UWBDataSet = 0;
    prevUWBDataSet = 0;

    % xt_b = [-0.525 0.525 -0.525 0.525];
    % yt_b = [0.505 0.505 -0.505 -0.505];

    P = eye(15);
    UWBAnc_Full = 0;
    
    grav = 9.85;

    PrevPosIMU = 0;
    PrevHeadingIMU = 0;
    cnt = 0;
    PosH_vec = zeros(1,N)+0*j;
    PosH_vec_stime = zeros(1,N);
    HeadingH_vec = zeros(1,N);
end

% dT = 0.001;
kk = 1;
s_time = PositionVector(kk); kk = kk + 1;
SensorNum = PositionVector(kk); kk = kk + 1;
CCTVPosOn = 0;
GPSon = 0;

if SensorNum == 5
    
    dTtemp = s_time - s_time_IMU_prev;
    % if dTtemp > 1
        dT = 0.01;
    % else
    %     dT = 0.01;
    % end
    % if dTtemp > 10
    %     dT = 0.03;
    % else
    %     dT = dTtemp;
    % end

    IMUacc_c = PositionVector(3:5);
    IMUgyro_c = PositionVector(6:8);
    
    IMUacc_c_fifo = [IMUacc_c_fifo(2:4,:) ;IMUacc_c];
    IMUgyro_c_fifo = [IMUgyro_c_fifo(2:4,:) ;IMUgyro_c];

    GyroD = mean(IMUgyro_c_fifo)*pi/180;
    acc = mean(IMUacc_c_fifo);
    acc(3) = acc(3) + grav;

    if prev_state == 1   
        XhatIMU = transpose(XhatUWB);
        prev_state = 2;
        XhatIMU = PredEKF_3D_Simple(XhatIMU,acc,GyroD,dT);
    else
        XhatIMU = PredEKF_3D_Simple(XhatIMU,acc,GyroD,dT);

    end

    % if (0)
    if (prevUWBDataSet ~= 0) && (UWBDataSet ~= 0) && ((s_time-UWBDataSet > 0.02)||(UWBAnc_Full==1)) && (UWBDataSet - prevUWBDataSet < 1) && (PrevPosHUWB~=0)
        dt_uwb = UWBDataSet - prevUWBDataSet;
        Xbar = PredEKF_3D_Simple(XhatUWB,acc,GyroD,dt_uwb);
        A = makePredA_3D_Simple(XhatUWB,acc,GyroD,dt_uwb);
        [Zv,H] = EstEKF_Center_3D_Simple_1(Xbar);

        Q = eye(15)*1e-5;
        Q(1:6,1:6) = eye(6)*1e-5;
        Q = Q * 0.1;
        
        R = eye(6)*5e-3;

        cnt = 0;
        
        P = A*P*transpose(A)+Q;
        
        K = P*transpose(H)/(H*P*transpose(H)+R);
        if isnan(K)
            PosHF = XhatIMU(1)+XhatIMU(2)*j;
            HeadingHF = XhatIMU(10);
            % PrevHeadingIMU = HeadingHF;

            GammHF = XhatIMU(12);
            BetaHF = XhatIMU(11);
        else
            game = atan((acc(2)-Xbar(8))/(acc(3)-grav-Xbar(9)));
            bete = atan((acc(1)-Xbar(7))/sqrt((acc(2)-Xbar(8))^2+(acc(3)-grav-Xbar(9))^2));
            game = AngleMatching(Zv(3),game);
            bete = AngleMatching(Zv(2),bete);
            
            XhatUWB = real(transpose(Xbar) + K*(transpose([-PrevHeadingHUWB bete game real(PrevPosHUWB) imag(PrevPosHUWB) 0])-transpose(Zv)));
            P = P - K*H*P;

            prevUWBDataSet = UWBDataSet;
            prev_state = 1;
            PosHF = XhatUWB(1)+XhatUWB(2)*j;
            HeadingHF = XhatUWB(10);
            % PrevHeadingIMU = HeadingHF;
            GammHF = XhatUWB(12);
            BetaHF = XhatUWB(11);
            
        end
        s_time_IMU_prev = s_time;
    elseif (UWBDataSet ~= 0) && ((s_time-UWBDataSet > 0.02)||(UWBAnc_Full==1))
        prevUWBDataSet = UWBDataSet;
        XhatIMU(1) = real(PrevPosHUWB);
        XhatIMU(2) = imag(PrevPosHUWB);
        XhatIMU(10) = -PrevHeadingHUWB;
                
        XhatUWB = transpose(XhatIMU);
        prev_state = 1;
        PosHF = XhatUWB(1)+XhatUWB(2)*j;
        HeadingHF = XhatUWB(10);
        % PrevHeadingIMU = HeadingHF;
        GammHF = XhatUWB(12);
        BetaHF = XhatUWB(11); 
    elseif (CCTVPosOn) 

    elseif (GPSon)

    else
        PosHF = XhatIMU(1)+XhatIMU(2)*j;
        PrevPosIMU = PosHF;
        HeadingHF = XhatIMU(10);
        % PrevHeadingIMU = HeadingHF;
        GammHF = XhatIMU(12);
        BetaHF = XhatIMU(11); 
    end


    % PosH = Xbar(1)+Xbar(2)*j;
    % HeadingH = Xbar(7);
    % PosH = 0;
    % HeadingH = PrevHeadingHUWB;
    PrevHeadingHUWB = -HeadingHF;
    PrevBetaHUWB = BetaHF;
    PrevGammaHUWB = GammHF;
    UWBAnc_Full = 0;
    % PrevPosHUWB = PosHF;
    PrevPosIMU = PosHF;
    PrevHeadingIMU = HeadingHF;
    PosH = 0;
    HeadingH = 0;
elseif SensorNum == 6 %% GPS
    PosHF = 0;
    HeadingHF = 0;
        GammHF = 0;
    BetaHF = 0;
    PosH = 0;
    HeadingH = 0;
elseif SensorNum == 7 %% CCTV
    PosHF = 0;
    HeadingHF = 0;
        GammHF = 0;
    BetaHF = 0;
    PosH = 0;
    HeadingH = 0;
else
    
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
    [PosHT, Heading, UWBAnc_Full] = UWBPosition_V4_1(s_time, Ln, LnC, Nanchor, SensorNum, RxIDUWB, RxDistOrig, xain, yain, zain, xt_b, yt_b, zt_b, PrevPosHF, PrevHeadingHF);
    PrevHeading = PrevHeadingHUWB; 
    HeadingHT = real(meanAngle(-PrevHeadingIMU,Heading));
    
    if PosHT~=0
        PosH_vec(1:end-1)=PosH_vec(2:end);
        PosH_vec(end) = PosHT;

        HeadingH_vec(1:end-1)=HeadingH_vec(2:end);
        HeadingH_vec(end) = HeadingHT;
        
        PosH_vec_stime(1:end-1)=PosH_vec_stime(2:end);
        PosH_vec_stime(end) = s_time;
        
        if length(PosH_vec(PosH_vec==0)) == 0 
            [PolyPos_R] = polyfit(PosH_vec_stime,real(PosH_vec),2);
            PosH = polyval(PolyPos_R,s_time); 
            [PolyPos_I] = polyfit(PosH_vec_stime,imag(PosH_vec),2);
            PosH = PosH+j*polyval(PolyPos_I,s_time); 

            PolyHeading = polyfit(PosH_vec_stime,HeadingH_vec,2);
            HeadingH = polyval(PolyHeading,s_time);
        else
            PosH = PosHT;
            HeadingH = HeadingHT;
        end
    else
        PosH = 0;
        HeadingH = 0;
    end

    PrevHeadingHUWB = HeadingH;
    PosHT = PosH;
    
    if PosH ~= 0
        UWBDataSet = s_time;
        PrevPosHUWB = PosH;
        % PrevHeadingHUWB = HeadingH;
    else
        UWBDataSet = 0;
        % PrevPosHUWB = PrevPosIMU;
        % PrevHeadingHUWB = PrevHeadingIMU;
    end

    if abs(PosH) > 50
        dd = 1;
    end

    % PosHF = PrevPosHUWB;
    % HeadingHF = -PrevHeadingHUWB;
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





