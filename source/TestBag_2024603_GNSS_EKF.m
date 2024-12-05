clear all;
% close all;
% aa = rosbag('2023-07-26-18-35-13.bag');
% aa = rosbag('2023-08-07-11-03-21.bag');
% aa = rosbag('2023-08-07-14-34-54.bag');
% aa = rosbag('[zed_f9r]2023-08-31-17-58-37_fast.bag');
% aa = rosbag('[zed_f9r]2023-08-31-18-00-04_long_stop.bag');

% aa = rosbag('[zed_f9r]2023-08-31-17-56-41_slow.bag');
% aa = rosbag('[zed_f9r]2023-08-31-18-06-05_parking.bag');
% aa = rosbag('[zed_f9r]2023-12-07-13-09-48.bag');
% aa = rosbag('[zed_f9r]2023-12-05-18-16-11.bag');
% aa = rosbag('2024-04-02-16-21-39.bag');
% aa = rosbag('2024-02-16-17-58-34.bag');
% aa = rosbag('2024-03-05-17-00-06.bag');
% aa = rosbag('2024-03-12-16-21-39.bag');
% aa = rosbag('2024-03-19-21-57-37.bag');
% aa = rosbag('2024-03-28-21-04-23.bag');
% aa = rosbag('2024-04-01-17-55-40.bag');
% aa = rosbag('2024-04-01-16-48-31.bag');
% aa = rosbag('2024-04-02-10-39-34.bag');
% aa = rosbag('2024-04-02-15-54-11.bag');
% aa = rosbag('2024-04-12-16-31-41.bag');
% aa = rosbag('test.bag');
aa = rosbag('global_0530_track2_noImage.bag');

load('MapParams.mat','sA','SLAMposInit');
eul = dcm2eulr(sA);

IMUSel = 2; % 1 : Xsens,  2 : F9R

LenTot = 0;
GPSonT = 0;

for dfe = 1 : 4
    UWB{dfe} = select(aa,'Topic',['/dwm1001/anchor/ttyUWB',num2str(dfe-1,1)] );
    % UWB{dfe} = select(aa,'Topic',['/dwm1001/anchor/tag',num2str(dfe-1,1)] );

    LenTot = LenTot + size(UWB{dfe}.MessageList,1);
    UWBMsg{dfe} = readMessages(UWB{dfe},'DataFormat','struct');
end
gps_data = select(aa,'Topic','/zed_f9r/gnss_pvt');
gps_data_d = readMessages(gps_data,'DataFormat','struct');

if IMUSel == 1
    IMURoSV = select(aa,'Topic','zed_f9r/imu/angular_velocity');
    IMUVMsg = readMessages(IMURoSV,'DataFormat','struct');

    IMURoSA = select(aa,'Topic','/imu/acceleration');
    IMUAMsg = readMessages(IMURoSA,'DataFormat','struct');

    IMURoSM = select(aa,'Topic','/imu/mag');
    IMUMMsg = readMessages(IMURoSM,'DataFormat','struct');
else
    IMURoS = select(aa,'Topic','/zed_f9r/imu');
    IMUMsg = readMessages(IMURoS,'DataFormat','struct');
end

IMGpose = select(aa,'Topic','/orb_slam3/camera_pose');
IMGMsg = readMessages(IMGpose,'DataFormat','struct');

IMGHeadingInit = 0;
for kkg = 1 : 2144
    IMGpos(kkg,:) = [IMGMsg{kkg}.Pose.Position.X IMGMsg{kkg}.Pose.Position.Y IMGMsg{kkg}.Pose.Position.Z];
    IMGHeading(kkg,:) = [IMGMsg{kkg}.Pose.Orientation.X IMGMsg{kkg}.Pose.Orientation.Y IMGMsg{kkg}.Pose.Orientation.Z IMGMsg{kkg}.Pose.Orientation.W];
    Temp = IMGHeading(kkg,:);
    if Temp(1) ~= 0
        if (any(any(IMGHeadingInit == 0))) || (any(abs(IMGHeading(kkg,:)-IMGHeading(kkg-1,:)) > 1))
            DAinit = qua2dcm(Temp);
            IMGHeadingInit = DAinit';
        end
    end
    DCMH(kkg,:) = quat2eul(Temp);
end

% figure(9999);plot3(IMGpos(:,1),IMGpos(:,2),IMGpos(:,3),'.');
% figure(9998);hold off;plot(IMGpos(:,1),'b.');hold on;plot(IMGpos(:,2),'g.');plot(IMGpos(:,3),'r.')
% figure(9997);hold off;plot(DCMH(:,1),'b.');hold on;plot(DCMH(:,2),'g.');plot(DCMH(:,3),'r.');

LnC = 0;
Lp = 4;
Ln = 23;
kalman_on = 1; % seonghyun (2023.09.08)


%% Original

xt_b = [-0.595 0.595 -0.595 0.595];
yt_b = [0.74 0.74 -0.74 -0.74];


% xt_b = [+0.595 -0.595 0.595 -0.595];
% yt_b = [-0.74 -0.74 0.74 0.74];

% xt_b = [-0.595 0.595 0.595 -0.595];
% yt_b = [0.74 0.74 -0.74 -0.74];

% xt_b = [-0.525 0.525 -0.525 0.525];
% yt_b = [0.505 0.505 -0.505 -0.505];
zt_b = 1.9;
%zt_b = 1.53;


tag_pos_b = xt_b + j*yt_b;

AnchorIDmapH = [];
AnchorIDmap = [];
xa = [];
ya = [];
za = [];
k0 = ones(1,7);
RxDistTot = zeros(1,5);

gravity = 9.86;

gyro_phi = 0;
gyro_theta = 0;
gyro_delta_phi = 0;
gyro_delta_theta = 0;
gyro_delta_psi = 0;
% b_acc = mean(IMUacc);
% b_acc = mean(IMUacc(1:100,:));
b_acc = [0 0 0]';
b_acc_n = [0 0 0]';

% b_gyro = mean(IMUgyro(1:100,:));
b_gyro = [0 0 0];
pos = zeros(3,1); vel = zeros(3,1);
b_gyro_his = zeros(3,1);
l = 100;
m = 100;
acc_b_m = [0 0 0]';
acc_b_phi = 0;

acc_g_bias = [0 0 0]';
b_acc_b = [0 0 0]';
acc_b_kf_wob = [0 0 0]';
Cb2n_kf = eye(3);
kf_phi_m = 0;
kf_theta_m = 0;

Cb2n = eye(3);

Lav = 1;

IMUMode = 1;

%b_acc_o = [ -0.2927    0.1928   -0.0194];  %% original
b_acc = [ -0.2921    0.1928   -0.019];  %% original
b_head = zeros(3,1);

i = 1;
Acc_S = [0 0 0]';
Gyro_S = [0 0 0]';
dT = 0.1;
dt = 0.01;
P = 1;

Lv = 10;

IMUacc = zeros(2*Lv, 3);
IMUgyro = zeros(2*Lv, 3);
Acc_Acc = zeros(3,2*Lv );
Acc_Vel = zeros(3,2*Lv);
Acc_Pos = zeros(3,2*Lv);
Acc_PQR = zeros(3,2*Lv);
Acc_Steady = zeros(3,2*Lv);
init_pos.x = 0;
init_pos.y = 0;
init_pos.z = 0;
kl = 1;
k = 0;
s_time_prev = 0;
%for k = 1 : LenTot*3-300
% for k = 1 : 12754
state_acc = 0 ; % 0: init,  1 : Work
state_IMU = 0 ; % 0: init,  1 : Work
RxIDTot = zeros(1,4);

heading_est_t = 0;
tag_pos_est_t = zeros(1,4);

s_time_prev = 0;
anch_pos_o  = [-0.01-0.3i	7.85-0.3i	7.84+10.5i	0+15.1i	22.7-4i    24.6+16.3i];
% anch_pos_o  = [-0.01-0.3i	7.85-0.3i	7.84+10.5i	0+15.1i	22.5-3.9i    24.4+15.3i];

init_flag = 0;
TagPosBuff = zeros(4,20);
tag_pos_est = zeros(1,4);
heading_est = 0;
TagPos = [0 0 0 0];

% 2023.11.09_ljw0904
kf_psi =0;
cent_vel_est =  zeros(3,1 );
cent_pos_est =  zeros(3,1 );
b_acc_o = [ 0    0   0];


acc_b_theta = 0;


UWB_LS_Pos = zeros(3,2*Lv);
UWB_M_Pos = zeros(3,2*Lv);
UWB_M_Vel = zeros(3,2*Lv);
kf_phi = 0;
kf_theta = 0;
Reliability = 0;

gyro_psi = 0;

if IMUSel == 1
    IMUMsg = IMUVMsg;
end

IMUacc_c_fifo = zeros(4,3);
IMUgyro_c_fifo = zeros(4,3);

Xhat = [0 0 0 0 0 0 0 0 ]';
P = eye(8);
qv = 1;
Heading_Prev = 0;
alpe_t1 = 0;
xain = [];
yain = [];

IMU_tot = [];
RxDistTot = zeros(1,4,8);
qq = 1;

yain_lat = [37.4068930 37.4068480 37.4067600 37.4066730 37.406722 37.4067820 37.4068710 37.4069590 ];
xain_lon = [127.1018290 127.1018270 127.1018380 127.1018490 127.1021050 127.102097 127.1020980 127.1020860];
anchor_idh = ['5AA7';'4302';'1221';'991B';'5C28';'439D';'9B8F';'89A4'];
for kk = 1 : size(anchor_idh,1)
    anchor_idd(kk) = hex2dec(anchor_idh(kk,:));
end
% NumBuff = 100;
% SLAMtimeList = zeros(1,NumBuff);
% SLAMposList = zeros(3,NumBuff);
% UWBtimeList = zeros(1,NumBuff);
% UWBposList = zeros(3,NumBuff);
% SLAMtime = zeros(1,NumBuff);
% SLAMpos = zeros(NumBuff,3);
% SLAMposInit = 0;
% PosSLAMout = zeros(3,1);
        ParkingLotS= imread('B1_ParkingLot.jpg');
        for sx = 1:size(ParkingLotS,1)
            for sy = 1:size(ParkingLotS,2)
                % ParkingLot(sy,sx,:) = ParkingLotS(size(ParkingLotS,1)-sx+1,size(ParkingLotS,2)-sy+1,:);
                ParkingLot(sy,sx,:) = ParkingLotS(sx,size(ParkingLotS,2)-sy+1,:);
            end
        end

%while((length(UWBMsg{1})>=k0(1))&&(length(UWBMsg{2})>=k0(2))&&(length(UWBMsg{3})>=k0(3))&&(length(UWBMsg{4})>=k0(4))&&(length(IMUMsg)>=k0(5)))
while(length(UWBMsg{1})>=k0(1) || length(UWBMsg{2})>=k0(2) || length(UWBMsg{3})>=k0(3) || length(UWBMsg{4})>=k0(4) || length(IMUMsg)>=k0(5) || length(gps_data_d)>=k0(6) || size(IMGpos,1)>=k0(7)   )
    k = k + 1;
    for lk = 1 : 4
        if length(UWBMsg{lk})<k0(lk)
            s_time_a(lk) = 100000000000000000000;
        else
            s_time_a(lk) = double(UWBMsg{lk}{k0(lk)}.Header.Stamp.Sec)+double(UWBMsg{lk}{k0(lk)}.Header.Stamp.Nsec)/10^9;
        end
    end
    if length(IMUMsg)<k0(5)
        s_time_a(lk+1) = 100000000000000000000;
    else
        s_time_a(lk+1) = double(IMUMsg{k0(lk+1)}.Header.Stamp.Sec)+double(IMUMsg{k0(lk+1)}.Header.Stamp.Nsec)/10^9;
    end
    lk = lk + 1;
    if length(gps_data_d)<k0(6)
        s_time_a(lk+1) = 100000000000000000000;
    else
        s_time_a(lk+1) = double(gps_data_d{k0(lk+1)}.Header.Stamp.Sec)+double(gps_data_d{k0(lk+1)}.Header.Stamp.Nsec)/10^9;
    end

    lk = lk + 1;
    if size(IMGpos,1)<k0(7)
        s_time_a(lk+1) = 100000000000000000000;
    else
        s_time_a(lk+1) = double(IMGMsg{k0(lk+1)}.Header.Stamp.Sec)+double(IMGMsg{k0(lk+1)}.Header.Stamp.Nsec)/10^9;
    end

    [min_stamp_v, min_stamp_i] = min(s_time_a);
    s_time = min_stamp_v;

    % if min_stamp_i < 5
    %     min_stamp_i = 8;
    % end

    PositionVector = [];
    PositionVector(1) = s_time;
    PositionVector(2) = min_stamp_i;

    if min_stamp_i == 7

        SLAMpos(k,:) = IMGpos(k0(min_stamp_i),:);
        SLAMorient(k,:) = IMGHeading(k0(min_stamp_i),:);
        SLAMtime(k) = s_time;

        k0(min_stamp_i) = k0(min_stamp_i) + 1;
        PosSLAMout(:,k) = sA*transpose(SLAMpos(k,[1 3 2]))+SLAMposInit;

        dcmSLAM = qua2dcm(SLAMorient(k,:));
        temp = dcmSLAM*[1;0;0];
        edl = angle(temp(1)+j*temp(3));
        % HeadingSLAM(dfd) = (edl+eul(3));
        HeadingSLAMout(k) = (edl+pi);

        figure(2313);hold off;
        imshow(ParkingLot);
        Scale = 25.3;
        hold on;plot(Scale*(xa-0.3)+j*(size(ParkingLot,2)-Scale*ya-Scale*11),'ro');
        plot(Scale*(PosSLAMout(1,k)-0.3),size(ParkingLot,2)-Scale*PosSLAMout(2,k)-Scale*11,'ro')
        xarrow = -1*Scale*[-1 1 0 -1];
        yarrow = -1*Scale*[-2 -2 2 -2];
        arrowc = xarrow+j*yarrow;
        arrowHeading = arrowc*exp(-j*(HeadingSLAMout(k)));
        plot(real(arrowHeading)+Scale*PosSLAMout(1,k),imag((arrowHeading))+(size(ParkingLot,2)-Scale*PosSLAMout(2,k)-Scale*11));

        % figure(2313);hold on;quiver(PosSLAMout(1,k),PosSLAMout(2,k),temp(1),temp(3),0);
        % figure(3);plot(xa+j*ya,'ro');
        axis equal

    elseif min_stamp_i == 6

        FixType(k) = gps_data_d{k0(min_stamp_i)}.FixType;
        NumSV(k) = gps_data_d{k0(min_stamp_i)}.NumSV;
        Flag1(k) = gps_data_d{k0(min_stamp_i)}.Flags;
        Flag2(k) = gps_data_d{k0(min_stamp_i)}.Flags2;
        Lon(k) = gps_data_d{k0(min_stamp_i)}.Lon;
        Lat(k) = gps_data_d{k0(min_stamp_i)}.Lat;
        HeightGPS(k) = gps_data_d{k0(min_stamp_i)}.Height;
        HAcc(k) = gps_data_d{k0(min_stamp_i)}.HAcc;
        VAcc(k) = gps_data_d{k0(min_stamp_i)}.VAcc;
        HeadAcc(k) = gps_data_d{k0(min_stamp_i)}.HeadAcc;
        HeadingGPS(k) = gps_data_d{k0(min_stamp_i)}.Heading;

        k0(min_stamp_i) = k0(min_stamp_i) + 1;

    elseif min_stamp_i == 5

        if IMUSel == 1
            IMUacc_c  = [IMUAMsg{k0(5)}.Vector.X IMUAMsg{k0(5)}.Vector.Y IMUAMsg{k0(5)}.Vector.Z];
            IMUgyro_c = [IMUVMsg{k0(5)}.Vector.X IMUVMsg{k0(5)}.Vector.Y IMUVMsg{k0(5)}.Vector.Z];
        else
            IMUacc_c = [IMUMsg{k0(5)}.LinearAcceleration.X IMUMsg{k0(5)}.LinearAcceleration.Y IMUMsg{k0(5)}.LinearAcceleration.Z]*1;
            IMUgyro_c  = [IMUMsg{k0(5)}.AngularVelocity.X IMUMsg{k0(5)}.AngularVelocity.Y IMUMsg{k0(5)}.AngularVelocity.Z];
        end

        if IMUacc_c(3)>0
            IMUacc_c = [IMUacc_c(1) -IMUacc_c(2) -IMUacc_c(3)];
            IMUgyro_c = [IMUgyro_c(1) -IMUgyro_c(2) -IMUgyro_c(3)];
        end

        IMUacc_c_t(k0(5),:) = IMUacc_c;
        IMUgyro_c_t(k0(5),:) = IMUgyro_c;

        IMUacc_c_fifo = [IMUacc_c_fifo(2:4,:) ;IMUacc_c];
        IMUgyro_c_fifo = [IMUgyro_c_fifo(2:4,:) ;IMUgyro_c];

        PositionVector(3:5) = IMUacc_c;
        PositionVector(6:8) = IMUgyro_c;

        IMU_tot(k0(5),1:6) = [IMUacc_c IMUgyro_c];

        k0(5) = k0(5) + 1;

    elseif min_stamp_i<5

        PP = min_stamp_i;
        RxID = [];
        TT = UWBMsg{PP}{k0(PP)}.Id;

        for df = 1 : length(TT)

            [val,ids] = find(AnchorIDmap == hex2dec(TT{df}));

            if length(ids)>0
                RxID(df) = ids;
            else
                LnC = LnC + 1;
                AnchorIDmapH{LnC} = TT{df};
                AnchorIDmap(LnC) = hex2dec(AnchorIDmapH{LnC});
                RxID(df) = LnC;

                if GPSonT == 1
                    [valp,idsp] = find(anchor_idd == hex2dec(TT{df}));
                    xa(LnC) = xain_lon(idsp)*88427.2314400;
                    ya(LnC) = yain_lat(idsp)*111319.4907900;
                    za(LnC) = UWBMsg{PP}{k0(PP)}.Z(df);
                else
                    xa(LnC) = UWBMsg{PP}{k0(PP)}.X(df);
                    ya(LnC) = UWBMsg{PP}{k0(PP)}.Y(df);
                    za(LnC) = UWBMsg{PP}{k0(PP)}.Z(df);
                end
            end

        end
        RxDistOrig = zeros(1,4);
        RxDistOrig(1:length(RxID)) = UWBMsg{PP}{k0(PP)}.DistanceFromTag;

        RxDist = real(sqrt(RxDistOrig.^2-(za(1)-zt_b)^2));

        RxDistTot(k,PP,RxID) = RxDistOrig(1:length(RxID));

        Nanchor = length(RxID);
        RxIDUWB = zeros(1,Ln);
        RxIDUWB(1:Nanchor) = RxID;

        xain = zeros(1,Ln);
        xain(1:LnC) = xa;

        yain = zeros(1,Ln);
        yain(1:LnC) = ya;

        PositionVector = [PositionVector Ln LnC Nanchor RxIDUWB RxDistOrig xain(1:LnC) yain(1:LnC) za xt_b yt_b zt_b ];

        k0(PP) = k0(PP) + 1;
        qq = qq + 1;
    end

    PositionOut = PositioningSystem_V3_1(PositionVector);

    PosH(k) = PositionOut(1)+j*PositionOut(2);

    HeadingH(k) = PositionOut(4);
    GamH(k) = PositionOut(5);
    BetaH(k) = PositionOut(6);
    P = PositionOut(7);
    PosHUWB(k) = PositionOut(8)+j*PositionOut(9);
    HeadingHUWB(k) = PositionOut(10);
    PosHUWBtime(k) = s_time;

    if length(SLAMtime) < k
        SLAMtime(k) = 0;
    end



    % if SLAMtime(ggd)~=0
    %     SLAMtimeList = [SLAMtimeList(2:end) SLAMtime(ggd)];
    %     SLAMposList = [SLAMposList(:,2:end) transpose(SLAMpos(ggd,:))];
    % 
    %     if (SLAMposInit(1) == 0) 
    %         tempB = find(SLAMposList(1,:)~=0);
    %         tempC = find(UWBposList(1,:)~=0);
    %         if (length(tempB)~=0) && (length(tempC)~=0)
    %             [LsI] = min(find(SLAMtimeList(tempB(1)) < UWBtimeList(tempC)));
    %             [LfI] = max(find(SLAMtimeList(tempB(end)) > UWBtimeList(tempC)));
    %             Ls = tempC(LsI);
    %             Lf = tempC(LfI);
    % 
    %             if isempty(Ls)
    %                 Ls = 101;
    %             end
    % 
    %             if ((Lf - Ls) > 3) && (length(tempB)>3)
    %                 SLAMposInterp = [];
    %                 SLAMposInterp(1,:) = interp1(SLAMtimeList(tempB),SLAMposList(1,tempB),UWBtimeList(Ls:Lf));
    %                 SLAMposInterp(2,:) = interp1(SLAMtimeList(tempB),SLAMposList(3,tempB),UWBtimeList(Ls:Lf));
    %                 SLAMposInterp(3,:) = interp1(SLAMtimeList(tempB),SLAMposList(2,tempB),UWBtimeList(Ls:Lf));
    %                 TempDU = (UWBposList(:,Ls+1:Lf))-(UWBposList(:,Ls));
    %                 TempDS = (SLAMposInterp(:,2:end))-(SLAMposInterp(:,1));
    % 
    %                 [absTempDU Indx]= (sort(sum(TempDU.^2),'descend'));
    % 
    %                 if (sqrt(absTempDU(1)) > 10) && (ggd > 5000)
    %                     sA = TempDU*transpose(TempDS)*inv(TempDS*transpose(TempDS));
    %                     % sA = TempDU(:,Indx(1:max(length(Indx),10)))*transpose(TempDS(:,Indx(1:max(length(Indx),10))))*inv(TempDS(:,Indx(1:max(length(Indx),10)))*transpose(TempDS(:,Indx(1:max(length(Indx),10)))));
    %                     s = det(sA);
    %                     A = sA/s;
    %                     SLAMposInit = UWBposList(:,Lf)-sA*SLAMposInterp(:,end);
    %                     MaxabsTempDU = absTempDU(1);
    %                     PosSLAMout(:,ggd) = sA*transpose(SLAMpos(ggd,[1 3 2]))+SLAMposInit;
    %                     PrevSLAMpos = PosSLAMout(:,ggd);
    %                     RefUWBpos = UWBposList(:,Ls);
    %                     RefSLAMpos = SLAMposInterp(:,1);
    %                 end
    %             end
    %         end
    % 
    %     else
    %         tempB = find(SLAMposList(1,:)~=0);
    %         tempC = find(UWBposList(1,:)~=0);
    % 
    %         if (length(tempB)~=0) && (length(tempC)~=0)
    %             [LsI] = min(find(SLAMtimeList(tempB(1)) < UWBtimeList(tempC)));
    %             [LfI] = max(find(SLAMtimeList(tempB(end)) > UWBtimeList(tempC)));
    %             Ls = tempC(LsI);
    %             Lf = tempC(LfI);
    % 
    %             if isempty(Ls)
    %                 Ls = 101;
    %             end
    % 
    %             if ((Lf - Ls) > 3) && (length(tempB)>3)
    %                 SLAMposInterp = [];
    %                 SLAMposInterp(1,:) = interp1(SLAMtimeList(tempB),SLAMposList(1,tempB),UWBtimeList(Ls:Lf));
    %                 SLAMposInterp(2,:) = interp1(SLAMtimeList(tempB),SLAMposList(3,tempB),UWBtimeList(Ls:Lf));
    %                 SLAMposInterp(3,:) = interp1(SLAMtimeList(tempB),SLAMposList(2,tempB),UWBtimeList(Ls:Lf));
    %                 TempDU = (UWBposList(:,Ls+1:Lf))-(RefUWBpos);
    %                 TempDS = (SLAMposInterp(:,2:end))-(RefSLAMpos);
    % 
    %                 [absTempDU Indx]= (sort(sum(TempDU.^2),'descend'));
    % 
    %                 if (sqrt(absTempDU(1)) > 1.5*sqrt(MaxabsTempDU))
    %                     sA = TempDU*transpose(TempDS)*inv(TempDS*transpose(TempDS));
    %                 %     sA = TempDU(:,Indx(1:max(length(Indx),10)))*transpose(TempDS(:,Indx(1:max(length(Indx),10))))*inv(TempDS(:,Indx(1:max(length(Indx),10)))*transpose(TempDS(:,Indx(1:max(length(Indx),10)))));
    %                     s = det(sA);
    %                     A = sA/s;
    %                     SLAMposInit = UWBposList(:,Lf)-sA*SLAMposInterp(:,end);
    %                     MaxabsTempDU = absTempDU(1);
    %                     PosSLAMout(:,ggd) = sA*transpose(SLAMpos(ggd,[1 3 2]))+SLAMposInit;
    %                     PrevSLAMpos = PosSLAMout(:,ggd);
    % 
    %                     RefUWBpos = UWBposList(:,Ls);
    %                 end
    %             end
    %         end
    % 
    %         TempOut = sA*transpose(SLAMpos(ggd,[1 3 2]))+SLAMposInit;
    % 
    %         if sqrt(mean(abs(TempOut-PrevSLAMpos).^2))<3
    %             PosSLAMout(:,ggd) = TempOut;
    %             PrevSLAMpos = PosSLAMout(:,ggd);
    %         else
    %             SLAMposInit = 0;
    %             SLAMtimeList(1:end-1) = 0;
    %             SLAMposList(:,1:end-1) = 0;
    %         end
    % 
    %     end
    %     % if size(PosSLAMout,2)<ggd
    %     %     PosSLAMout(:,ggd) = UWBposList(:,end);
    %     % end
    % 
    %     % if SLAMposInit(1) ~= 0
    %     %     ReSLAMpos(:,ggd) = sA*transpose(SLAMpos(ggd,[1 3 2]))+SLAMposInit;
    %     % end
    % 
    % elseif (PosHUWBtime(ggd)~=0) && (PosHUWB(ggd)~=0)
    %     UWBtimeList = [UWBtimeList(2:end) PosHUWBtime(ggd)];
    %     UWBposList = [UWBposList(:,2:end) [real(PosHUWB(ggd)); imag(PosHUWB(ggd)); 0]];
    %     % PosSLAMout(:,ggd) = UWBposList(:,end);
    % end
    % SLAMposInitvec(1:3,ggd) = SLAMposInit;

end


figure(2313);plot(Scale*real(PosHUWB),size(ParkingLot,2)-Scale*imag(PosHUWB)-Scale*11,'bo')

if GPSonT == 1
    figure(30);hold off;
    geolimits([37.4056-0.002/5 37.4056+0.000/5],[127.1038-0.01/5 127.1038+0.01/5])
    geoplot(imag(PosH)/111319.4907900, real(PosH)/88427.2314400,'.');
    geolimits([37.4056-0.002/5 37.4056+0.000/5],[127.1038-0.01/5 127.1038+0.01/5])
    figure(30);hold on;
    geoplot(double(Lat)/1e7, double(Lon)/1e7,'r.');
    geolimits([37.4056-0.002/5 37.4056+0.000/5],[127.1038-0.01/5 127.1038+0.01/5])
else

    figure(124);hold off;plot((PosH),'b.');axis equal
    figure(125);hold off;plot(real(PosH),'b.');hold on;plot(imag(PosH),'r.')
    [xmap,ymap] = axis_convert(real(PosH),imag(PosH));
    [xmap_anchor,ymap_anchor] = axis_convert(xain,yain);

    figure(10);hold on;plot(xmap,1134-ymap,'r.');plot(xmap_anchor,1134-ymap_anchor,'rsquare')

    figure(126);hold off;plot(HeadingH,'b.');hold on;plot(-HeadingHUWB, 'r.');
    figure(124);hold on;plot((PosHUWB),'g.')

    figure(124);plot(xa,ya,'rsquare',MarkerSize=6);

    figure(125);hold on;plot(real(PosHUWB),'g.');hold on;plot(imag(PosHUWB),'k.')

    figure(130);hold off;plot((PosHUWB),'b.');axis equal
    %
    % figure(24);hold on;plot((PosH),'r.')
    % figure(25);hold on;plot(real(PosH),'g.');hold on;plot(imag(PosH),'k.')
    % figure(26);hold on;plot(HeadingH,'r.')

    figure(200);hold off;plot(RxDistTot(:,1,1),'b.');hold on;plot(RxDistTot(:,2,1),'r.');plot(RxDistTot(:,3,1),'g.');plot(RxDistTot(:,4,1),'k.');
    kkf = 2; figure(200+kkf);hold off;plot(RxDistTot(:,1,kkf),'b.');hold on;plot(RxDistTot(:,2,1),'r.');plot(RxDistTot(:,3,1),'g.');plot(RxDistTot(:,4,1),'k.');

    for kkf = 1 : Ln-2
        for dfg = 1 : 4
            if dfg == 1
                figure(200+kkf);hold off;plot(RxDistTot(:,dfg,kkf),'.');
            else
                figure(200+kkf);hold on;plot(RxDistTot(:,dfg,kkf),'.');
            end
        end
    end

    for kkf = 1 : Ln
        for dfg = 1 : 4
            figure(400);hold on;plot(RxDistTot(:,dfg,kkf),'.');
        end
    end

end
