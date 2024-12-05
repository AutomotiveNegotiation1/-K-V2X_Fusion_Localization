function [Posn,Headn] = EKF_UWB_SLAM_2(PosUWB, HeadUWB, dPosSLAM, dHeadSLAM, QuatInit)

persistent PosP roll pitch yaw s H P Q R
persistent Hroll Hpitch Hyaw
persistent firstRun
persistent totHead totA totHUWB ParkingLot

if isempty(firstRun)
    roll = 0;
    pitch = 0;
    yaw = 0;
    Hroll = 0;
    Hpitch = 0;
    Hyaw = 0;

    s = 8.3;
    % H = [1 0 0 0 0 0 0 0 0 0 0 0 0;
    %      0 0 1 0 0 0 0 0 0 0 0 0 0;
    %      0 0 0 0 0 0 0 1 0 0 0 0 0];
    H = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0];

    PosP = zeros(1,3);

    Q = 1e-1*ones(16,16);
    Temp = Ajacob(ones(1,16));
    Q(Temp==0)=0;

    R = 100*eye(size(H,1),size(H,1));

    P = ones(16,16);
    P(Temp==0)=0;

    firstRun = 1;

    ParkingLot = uint8([]);
    ParkingLotS= imread('B1_ParkingLot.jpg');
    for sx = 1:size(ParkingLotS,1)
        for sy = 1:size(ParkingLotS,2)
            % ParkingLot(sy,sx,:) = ParkingLotS(size(ParkingLotS,1)-sx+1,size(ParkingLotS,2)-sy+1,:);
            ParkingLot(sy,sx,:) = ParkingLotS(sx,size(ParkingLotS,2)-sy+1,:);
        end
    end
    figure(123411);hold off;
    imshow(ParkingLot);
    % load('MapParams2.mat')
    % Tep = dcm2eulr(sA);
    % roll = Tep(1);
    % pitch = Tep(2);
    % yaw = Tep(3);
end

if (PosUWB~=0)
    if (PosP(1)==0) %|| (dPosSLAM(1)==0)
        PosP = [real(PosUWB);0;imag(PosUWB)];
        % Eul = quat2eul(QuatInit);
        Hroll = 0;
        Hpitch = HeadUWB;
        Hyaw = 0;
        % s = 8.3;
    elseif (dPosSLAM(1)==0)
        % Eul = quat2eul(QuatInit);
        % Hroll = Eul(1);
        % Hpitch = Eul(2);
        % Hyaw = Eul(3);
        % load("MapParams2.mat");
        Tep = dcm2eulr(sA);
        roll = Tep(1);
        pitch = Tep(2);
        yaw = Tep(3);


        Q = 1e-7*ones(16,16);
        Temp = Ajacob(ones(1,16));
        Q(Temp==0)=0;

        % roll = roll - dHeadSLAM(1);
        % pitch = pitch - dHeadSLAM(2);
        % yaw = yaw - dHeadSLAM(3);
    end
end

xhat = [PosP(1) PosP(2) PosP(3) dPosSLAM(1) dPosSLAM(2) dPosSLAM(3) roll pitch yaw s Hroll Hpitch Hyaw dHeadSLAM(1) dHeadSLAM(2) dHeadSLAM(3)];
xp = fx(xhat);

if (PosUWB~=0)
    % if (PosP(1)==0)
    %     PosP = [real(PosUWB);0;imag(PosUWB)];
    %     pitch = HeadUWB;
    %     s = 8.3;
    % end
    A = Ajacob(xhat);

    if (HeadUWB-Hpitch)>2*pi
        Nh = floor((HeadUWB-Hpitch)/(2*pi));
        HeadUWB = HeadUWB - 2*pi*Nh;
    elseif (Hpitch-HeadUWB) > 2*pi
        Nh = floor((Hpitch - HeadUWB)/(2*pi));
        HeadUWB = HeadUWB + 2*pi*Nh;
    end

    if (HeadUWB-Hpitch) > pi
        HeadUWB = HeadUWB -2*pi;
    elseif (HeadUWB-Hpitch) < -pi
        HeadUWB = HeadUWB + 2*pi;
    end

    z = [real(PosUWB);0;imag(PosUWB);0;HeadUWB;0];
    % z = [real(PosUWB);imag(PosUWB);HeadUWB];

    Pp = A*P*A'+Q;
    K = Pp*H'*inv(H*Pp*H'+R);

    x = xp + K*(z-H*xp);
    P = Pp - K*H*Pp;
else
    x = xp;
end

PosP = [x(1);x(2);x(3)];
roll = x(7);
pitch = x(8);
yaw = x(9);
s = x(10);
Hroll = x(11);
Hpitch = x(12);
Hyaw = x(13);

Posn = PosP(1)+j*PosP(3);
Headn = [Hroll Hpitch Hyaw];

totHead = [totHead [Hroll;Hpitch;Hyaw]];
totA = [totA [roll;pitch;yaw]];
totHUWB = [totHUWB HeadUWB];

% figure(123411);hold on;plot3(PosP(1),PosP(3),PosP(2),'ro');
figure(123411);hold off;
    imshow(ParkingLot);
Scale = 24.8;
% hold on;plot(Scale*(xa-0.3)+j*(size(ParkingLot,2)-Scale*ya-Scale*11),'ro');
% plot(Scale*(PosSLAMout(1,k)-0.3),size(ParkingLot,2)-Scale*PosSLAMout(2,k)-Scale*11,'ro')
xarrow = -1*Scale*[-0.8 0.8 0 -0.8];
yarrow = -1*Scale*[-1 -1 1 -1];
arrowc = xarrow+j*yarrow;
arrowHeading = arrowc*exp(j*(Hpitch));
hold on;plot(real(arrowHeading)+Scale*real(Posn),imag((arrowHeading))+(size(ParkingLot,2)-Scale*imag(Posn)-Scale*12));

% figure(2313);hold on;quiver(PosSLAMout(1,k),PosSLAMout(2,k),temp(1),temp(3),0);
% figure(3);plot(xa+j*ya,'ro');
axis equal
%------------------------
function xp = fx(xhat)
xn_1 = xhat(1);yn_1 = xhat(2);zn_1 = xhat(3);dx = xhat(4);dy = xhat(5);dz = xhat(6);
theta = xhat(7);gama = xhat(8);phi = xhat(9); sn_1 = xhat(10);
Htheta = xhat(11);Hgama = xhat(12);Hphi = xhat(13);
dHtheta = xhat(14); dHgama = xhat(15); dHphi = xhat(16);

xn = xn_1 + sn_1*(cos(theta)*cos(gama)*dx-sin(theta)*cos(phi)*dy+cos(theta)*sin(gama)*sin(phi)*dy+sin(theta)*sin(phi)*dz+cos(theta)*sin(gama)*cos(phi)*dz);
yn = yn_1 + sn_1*(sin(theta)*cos(gama)*dx+cos(theta)*cos(phi)*dy+sin(theta)*sin(gama)*sin(phi)*dy-cos(theta)*sin(phi)*dz+sin(theta)*sin(gama)*cos(phi)*dz);
zn = zn_1 + sn_1*(-sin(gama)*dx+cos(gama)*sin(phi)*dy+cos(gama)*cos(phi)*dz);
Hthetan = Htheta + dHtheta;
Hgaman = Hgama + dHgama;
Hphin = Hphi + dHphi;

xp = transpose([xn yn zn dx dy dz theta gama phi sn_1 Hthetan Hgaman Hphin dHtheta dHgama dHphi]);


%------------------------
function A = Ajacob(xhat)
xn_1 = xhat(1);yn_1 = xhat(2);zn_1 = xhat(3);dx = xhat(4);dy = xhat(5);dz = xhat(6);
theta = xhat(7);gama = xhat(8);phi = xhat(9); sn_1 = xhat(10);
Htheta = xhat(11);Hgama = xhat(12);Hphi = xhat(13);
dHtheta = xhat(14); dHgama = xhat(15); dHphi = xhat(16);

A = eye(length(xhat));
A(1,4) = sn_1*cos(theta)*cos(gama);
A(1,5) = sn_1*(-sin(theta)*cos(phi)+cos(theta)*sin(gama)*sin(phi));
A(1,6) = sn_1*(sin(theta)*sin(phi)+cos(theta)*sin(gama)*cos(phi));
A(1,7) = sn_1*(-sin(theta)*cos(gama)*dx-cos(theta)*cos(phi)*dy-sin(theta)*sin(gama)*sin(phi)*dy+cos(theta)*sin(phi)*dz-sin(theta)*sin(gama)*cos(phi)*dz);
A(1,8) = sn_1*(cos(theta)*(-sin(gama))*dx+cos(theta)*cos(gama)*sin(phi)*dy+cos(theta)*cos(gama)*cos(phi)*dz);
A(1,9) = sn_1*(sin(theta)*sin(phi)*dy+cos(theta)*sin(gama)*cos(phi)*dy+sin(theta)*cos(phi)*dz-cos(theta)*sin(gama)*sin(phi)*dz);
A(1,10) = (cos(theta)*cos(gama)*dx-sin(theta)*cos(phi)*dy+cos(theta)*sin(gama)*sin(phi)*dy+sin(theta)*sin(phi)*dz+cos(theta)*sin(gama)*cos(phi)*dz);
A(2,4) = sn_1*(sin(theta)*cos(gama));
A(2,5) = sn_1*(+cos(theta)*cos(phi)+sin(theta)*sin(gama)*sin(phi));
A(2,6) = sn_1*(-cos(theta)*sin(phi)+sin(theta)*sin(gama)*cos(phi));
A(2,7) = sn_1*(cos(theta)*cos(gama)*dx-sin(theta)*cos(phi)*dy+cos(theta)*sin(gama)*sin(phi)*dy+sin(theta)*sin(phi)*dz+cos(theta)*sin(gama)*cos(phi)*dz);
A(2,8) = sn_1*(sin(theta)*(-sin(gama))*dx+sin(theta)*cos(gama)*sin(phi)*dy+sin(theta)*cos(gama)*cos(phi)*dz);
A(2,9) = sn_1*(-cos(theta)*sin(phi)*dy+sin(theta)*sin(gama)*cos(phi)*dy-cos(theta)*cos(phi)*dz-sin(theta)*sin(gama)*sin(phi)*dz);
A(2,10) = (sin(theta)*cos(gama)*dx+cos(theta)*cos(phi)*dy+sin(theta)*sin(gama)*sin(phi)*dy-cos(theta)*sin(phi)*dz+sin(theta)*sin(gama)*cos(phi)*dz);
A(3,4) = sn_1*(-sin(gama));
A(3,5) = sn_1*(cos(gama)*sin(phi));
A(3,6) = sn_1*(cos(gama)*cos(phi));
A(3,8) = sn_1*(-cos(gama)*dx-sin(gama)*sin(phi)*dy-sin(gama)*cos(phi)*dz);
A(3,9) = sn_1*(cos(gama)*cos(phi)*dy-cos(gama)*sin(phi)*dz);
A(3,10) = (-sin(gama)*dx+cos(gama)*sin(phi)*dy+cos(gama)*cos(phi)*dz);
A(11,14) = 1;
A(12,15) = 1;
A(13,16) = 1;



