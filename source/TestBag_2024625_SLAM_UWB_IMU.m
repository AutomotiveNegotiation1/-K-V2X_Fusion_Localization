clear all;
close all;

% aa = rosbag('map_v1_with_Lidar_test1.bag');   %% Last
% aa = rosbag('2024-07-23-17-17-40.bag');
aa = rosbag('2024-07-23-19-01-04.bag');
IMUSel = 1; % 1 : Zed2,  2 : F9R
LidarOn = 0;
LenTot = 0;
GPSonT = 0;
Front = 0;
GraphEvery = 0;

if Front==1
    CAMPos = (-0.065+0.77*j);
else
    CAMPos = (-0.06-0.77*j);
end

for dfe = 1 : 4
    UWB{dfe} = select(aa,'Topic',['/dwm1001/anchor/ttyUWB',num2str(dfe-1,1)] );
    LenTot = LenTot + size(UWB{dfe}.MessageList,1);
    UWBMsg{dfe} = readMessages(UWB{dfe},'DataFormat','struct');
end

gps_data = select(aa,'Topic','/zed_f9r/gnss_pvt');
gps_data_d = readMessages(gps_data,'DataFormat','struct');

if IMUSel == 1
    IMURoS = select(aa,'Topic','zed2/zed_node/imu/data');
    IMUMsg = readMessages(IMURoS,'DataFormat','struct');
else
    IMURoS = select(aa,'Topic','/zed_f9r/imu');
    IMUMsg = readMessages(IMURoS,'DataFormat','struct');
end


IMGpose = select(aa,'Topic','/orb_slam3/camera_pose');
IMGMsg = readMessages(IMGpose,'DataFormat','struct');

IMGHeadingInit = 0;
for kkg = 1 : length(IMGMsg)
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

figure(9999);plot3(IMGpos(:,1),IMGpos(:,2),IMGpos(:,3),'.');
figure(9998);hold off;plot(IMGpos(:,1),'b.');hold on;plot(IMGpos(:,2),'g.');plot(IMGpos(:,3),'r.')
figure(9997);hold off;plot(DCMH(:,1),'b.');hold on;plot(DCMH(:,2),'g.');plot(DCMH(:,3),'r.');

LnC = 0;
Lp = 4;
Ln = 23;
kalman_on = 1; % seonghyun (2023.09.08)


%% Original

xt_b = [-0.595 0.595 -0.595 0.595];
yt_b = [0.74 0.74 -0.74 -0.74];

zt_b = 2.3;


tag_pos_b = xt_b + j*yt_b;
AnchorIDmap = [];
k0 = ones(1,7);

qq = 1;
k = 1;

ParkingLotS= imread('B1_ParkingLot.jpg');
for sx = 1:size(ParkingLotS,1)
    for sy = 1:size(ParkingLotS,2)
        % ParkingLot(sy,sx,:) = ParkingLotS(size(ParkingLotS,1)-sx+1,size(ParkingLotS,2)-sy+1,:);
        ParkingLot(sy,sx,:) = ParkingLotS(sx,size(ParkingLotS,2)-sy+1,:);
    end
end

while(length(UWBMsg{1})>=k0(1) || length(UWBMsg{2})>=k0(2) || length(UWBMsg{3})>=k0(3) || length(UWBMsg{4})>=k0(4) || length(gps_data_d)>=k0(6) || size(IMGpos,1)>=k0(7)   )
    k = k + 1;

    % UWB 테그 수집 시간  (index 1~4)
    for lk = 1 : 4
        if length(UWBMsg{lk})<k0(lk)
            s_time_a(lk) = 100000000000000000000;
        else
            s_time_a(lk) = double(UWBMsg{lk}{k0(lk)}.Header.Stamp.Sec)+double(UWBMsg{lk}{k0(lk)}.Header.Stamp.Nsec)/10^9;
        end
    end
    
    % IMU 수집 시간 (index 5)
    if length(IMUMsg)<k0(5)
        s_time_a(lk+1) = 100000000000000000000;
    else
        s_time_a(lk+1) = double(IMUMsg{k0(lk+1)}.Header.Stamp.Sec)+double(IMUMsg{k0(lk+1)}.Header.Stamp.Nsec)/10^9;
    end

    % GPS 수집 시간 (index 6)
    lk = lk + 1;
    if length(gps_data_d)<k0(6)
        s_time_a(lk+1) = 100000000000000000000;
    else
        s_time_a(lk+1) = double(gps_data_d{k0(lk+1)}.Header.Stamp.Sec)+double(gps_data_d{k0(lk+1)}.Header.Stamp.Nsec)/10^9;
    end

    % SLAM 수집 시간 (index 7)
    lk = lk + 1;
    if size(IMGpos,1)<k0(7)
        s_time_a(lk+1) = 100000000000000000000;
    else
        s_time_a(lk+1) = double(IMGMsg{k0(lk+1)}.Header.Stamp.Sec)+double(IMGMsg{k0(lk+1)}.Header.Stamp.Nsec)/10^9;
    end

    % 처리 후보중 가장 오래된 센서값 선택
    [min_stamp_v, min_stamp_i] = min(s_time_a);
    s_time = min_stamp_v;

    PositionVector = [];
    PositionVector(1) = s_time;
    PositionVector(2) = min_stamp_i;

    RxTime(k) = s_time;
    if min_stamp_i == 7

        SLAMpos(k,:) = IMGpos(k0(min_stamp_i),:);
        SLAMorient(k,:) = IMGHeading(k0(min_stamp_i),:);
        SLAMtime(k) = s_time;

        k0(min_stamp_i) = k0(min_stamp_i) + 1;

        eult = quat2eul(SLAMorient(k,:));
        SLAMorient(k,:) = eul2quat([eult(1) eult(2)*-1 eult(3)]);
        PositionVector = [PositionVector SLAMpos(k,:) SLAMorient(k,:)];

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

        % PositionVector 정의 및 PositioningSystem함수에서 처리루틴 필요

    elseif min_stamp_i == 5

        IMUacc_c = [IMUMsg{k0(5)}.LinearAcceleration.X IMUMsg{k0(5)}.LinearAcceleration.Y IMUMsg{k0(5)}.LinearAcceleration.Z]*1;
        IMUgyro_c  = [IMUMsg{k0(5)}.AngularVelocity.X IMUMsg{k0(5)}.AngularVelocity.Y IMUMsg{k0(5)}.AngularVelocity.Z];

        if IMUSel == 2
            IMUgyro_c = IMUgyro_c*pi/180;
        else
            IMUori_c = [IMUMsg{k0(5)}.Orientation.X IMUMsg{k0(5)}.Orientation.Y IMUMsg{k0(5)}.Orientation.Z IMUMsg{k0(5)}.Orientation.W];
        end

        % Up/Down 위치 보정
        if IMUacc_c(3)>0
            IMUacc_c = [IMUacc_c(1) -IMUacc_c(2) -IMUacc_c(3)];
            IMUgyro_c = [IMUgyro_c(1) -IMUgyro_c(2) -IMUgyro_c(3)];
        end

        PositionVector(3:5) = IMUacc_c;
        PositionVector(6:8) = IMUgyro_c;

        if IMUSel == 1
            IMU_tot(k0(5),1:11) = [s_time IMUacc_c IMUgyro_c IMUori_c];
        else
            IMU_tot(k0(5),1:7) = [s_time IMUacc_c IMUgyro_c];
        end

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

    PositionOut = PositioningSystem_V5_1(PositionVector);

    PosH(k) = PositionOut(1)+j*PositionOut(2);

    HeadingH(k) = PositionOut(4);
    GamH(k) = PositionOut(5);
    BetaH(k) = PositionOut(6);
    P = PositionOut(7);
    PosHUWB(k) = PositionOut(8)+j*PositionOut(9);
    HeadingHUWB(k) = PositionOut(10);
    PosHUWBtime(k) = s_time;

    if GraphEvery == 1
        if (PosH(k) ~=0)
            figure(123411);hold off;
            imshow(ParkingLot);
            Scale = 24.8;
            xarrow = -1*Scale*[-0.8 0.8 0 -0.8];
            yarrow = -1*Scale*[-1 -1 1 -1];
            arrowc = xarrow+j*yarrow;
            arrowHeading = arrowc*exp(j*(HeadingH(k)));
            hold on;plot(real(arrowHeading)+Scale*real(PosH(k)),imag((arrowHeading))+(size(ParkingLot,2)-Scale*imag(PosH(k))-Scale*11));
            axis equal
        end
    end

end

figure(124);hold off;plot((PosH),'b.');axis equal
% figure(124);hold on;plot((PosHUWB)+(-0.065+0.74*j)*exp(j*(HeadingHUWB)),'r.');grid minor
figure(124);hold on;plot((PosHUWB),'r.');grid minor

% 
figure(126);hold off;plot(mod(abs(HeadingH),2*pi),'b.');hold on;plot(mod(HeadingHUWB,2*pi), 'r.');

indxSLAM = find(PosH~=0);
indxUWB = find(PosHUWBtime~=0);
figure(125);hold off;plot(RxTime(indxSLAM)-RxTime(indxSLAM(1)),real(PosH(indxSLAM)),'b.');hold on;plot(RxTime(indxSLAM)-RxTime(indxSLAM(1)),imag(PosH(indxSLAM)),'r.')
% figure(125);hold off;plot(real(PosH),'b.');hold on;plot(imag(PosH),'r.')
figure(125);hold on;plot(PosHUWBtime(indxUWB)-RxTime(indxSLAM(1)), real(PosHUWB(indxUWB)),'g.');hold on;plot(PosHUWBtime(indxUWB)-RxTime(indxSLAM(1)), imag(PosHUWB(indxUWB)),'k.')
% 
figure(225);hold off;plot(RxTime(indxSLAM)-RxTime(indxSLAM(1)),real(PosH(indxSLAM)),'b.');hold on;plot(RxTime(indxSLAM)-RxTime(indxSLAM(1)),imag(PosH(indxSLAM)),'r.')
% figure(225);hold off;plot(RxTime(indxSLAM)-RxTime(indxSLAM(1)),real(PosH(indxSLAM)-CAMPos*exp(j*(-HeadingH(indxSLAM)))),'b.');hold on;plot(RxTime(indxSLAM)-RxTime(indxSLAM(1)),imag(PosH(indxSLAM)-CAMPos*exp(j*(-HeadingH(indxSLAM)))),'r.')
figure(225);hold on;plot(PosHUWBtime(indxUWB)-RxTime(indxSLAM(1)), real(PosHUWB(indxUWB)),'g.');hold on;plot(PosHUWBtime(indxUWB)-RxTime(indxSLAM(1)), imag(PosHUWB(indxUWB)),'k.')

figure(325);hold off;plot(real(PosH(indxSLAM)),'b.');hold on;plot(imag(PosH(indxSLAM)),'r.')
% figure(125);hold off;plot(real(PosH),'b.');hold on;plot(imag(PosH),'r.')
figure(325);hold on;plot(real(PosHUWB(indxUWB)),'g.');hold on;plot(imag(PosHUWB(indxUWB)),'k.')
% 


grid on;
% 
% 
% LidarPose = select(aa,'Topic','/kdlidar_ros_pcl/pose');
% LidarPoseMsg = readMessages(LidarPose,'DataFormat','struct');
% LidarPosition = [];
% for gelidar = 1 : length(LidarPoseMsg)
%     LidarPosition(gelidar,:) = [LidarPoseMsg{gelidar}.Pose.Position.X LidarPoseMsg{gelidar}.Pose.Position.Y  LidarPoseMsg{gelidar}.Pose.Position.Z];
%     LidarTime(gelidar) = double(LidarPoseMsg{gelidar}.Header.Stamp.Sec)+double(LidarPoseMsg{gelidar}.Header.Stamp.Nsec)/10^9;
% end
% figure(326);hold off;plot(LidarPosition(:,1),'b.');hold on;plot(LidarPosition(:,2),'r.');
% [MSEest] = CalcMSE(PosH(500:30000), HeadingH(500:30000), PosHUWBtime(100:30000), LidarPosition(1:1200,:), LidarTime(1:1200),0)
% % [MSEestUWB] = CalcMSE(PosHUWB(100:30000), HeadingHUWB(100:30000), PosHUWBtime(100:30000), LidarPosition, LidarTime,1)
% 
