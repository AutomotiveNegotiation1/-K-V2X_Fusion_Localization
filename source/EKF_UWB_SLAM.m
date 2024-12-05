function [Posn,Headn] = EKF_UWB_SLAM(PosUWB, HeadUWB, dPosSLAM, dHeadSLAM, QuatInit)

persistent PosP roll pitch yaw s H P Q R  
persistent firstRun

if isempty(firstRun)
    roll = 0;
    pitch = 0;
    yaw = 0;
    s = 1;
    % H = [1 0 0 0 0 0 0 0 0 0 0 0 0;
    %      0 0 1 0 0 0 0 0 0 0 0 0 0;
    %      0 0 0 0 0 0 0 1 0 0 0 0 0];
    H = [1 0 0 0 0 0 0 0 0 0 0 0 0;
         0 1 0 0 0 0 0 0 0 0 0 0 0;
         0 0 1 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 1 0 0 0 0 0 0;
         0 0 0 0 0 0 0 1 0 0 0 0 0;
         0 0 0 0 0 0 0 0 1 0 0 0 0];

    PosP = zeros(1,3);
    
    Q = 1e-7*ones(13,13);
    Temp = Ajacob(ones(1,13));
    Q(Temp==0)=0;

    R = 10*eye(size(H,1),size(H,1));
    
    P = ones(13,13);
    P(Temp==0)=0;

    firstRun = 1;
end

if (PosUWB~=0)
    if (PosP(1)==0) %|| (dPosSLAM(1)==0)
        PosP = [real(PosUWB);0;imag(PosUWB)];
        Eul = quat2eul(QuatInit);
        roll = Eul(1);
        pitch = Eul(2);
        yaw = Eul(3);
        s = 8.3;
    elseif (dPosSLAM(1)==0)
        Eul = quat2eul(QuatInit);
        roll = Eul(1);
        pitch = Eul(2);
        yaw = Eul(3);
        
    end
end

xhat = [PosP(1) PosP(2) PosP(3) dPosSLAM(1) dPosSLAM(2) dPosSLAM(3) roll pitch yaw s dHeadSLAM(1) dHeadSLAM(2) dHeadSLAM(3)];
xp = fx(xhat);

if (PosUWB~=0)
    % if (PosP(1)==0)
    %     PosP = [real(PosUWB);0;imag(PosUWB)];
    %     pitch = HeadUWB;
    %     s = 8.3;
    % end
    A = Ajacob(xhat);

    if (HeadUWB-pitch)>2*pi
        Nh = floor((HeadUWB-pitch)/(2*pi));
        HeadUWB = HeadUWB - 2*pi*Nh;
    elseif (pitch-HeadUWB) > 2*pi
        Nh = floor((pitch - HeadUWB)/(2*pi));
        HeadUWB = HeadUWB + 2*pi*Nh;
    end

    if (HeadUWB-pitch) > pi 
        HeadUWB = HeadUWB -2*pi;
    elseif (HeadUWB-pitch) < -pi 
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

Posn = PosP(1)+j*PosP(3);
Headn = [roll pitch yaw];

figure(123411);hold on;plot3(PosP(1),PosP(3),PosP(2),'ro');

%------------------------
function xp = fx(xhat)
xn_1 = xhat(1);yn_1 = xhat(2);zn_1 = xhat(3);dx = xhat(4);dy = xhat(5);dz = xhat(6); 
theta = xhat(7);gama = xhat(8);phi = xhat(9); sn_1 = xhat(10); dtheta = xhat(11); dgama = xhat(12); dphi = xhat(13);

xn = xn_1 + sn_1*(cos(theta)*cos(gama)*dx-sin(theta)*cos(phi)*dy+cos(theta)*sin(gama)*sin(phi)*dy+sin(theta)*sin(phi)*dz+cos(theta)*sin(gama)*cos(phi)*dz);
yn = yn_1 + sn_1*(sin(theta)*cos(gama)*dx+cos(theta)*cos(phi)*dy+sin(theta)*sin(gama)*sin(phi)*dy-cos(theta)*sin(phi)*dz+sin(theta)*sin(gama)*cos(phi)*dz);
zn = zn_1 + sn_1*(-sin(gama)*dx+cos(gama)*sin(phi)*dy+cos(gama)*cos(phi)*dz);
thetan = theta + dtheta;
gaman = gama + dgama;
phin = phi + dphi;

xp = transpose([xn yn zn dx dy dz thetan gaman phin sn_1 dtheta dgama dphi]);


%------------------------
function A = Ajacob(xhat)
xn_1 = xhat(1);yn_1 = xhat(2);zn_1 = xhat(3);dx = xhat(4);dy = xhat(5);dz = xhat(6); 
theta = xhat(7);gama = xhat(8);phi = xhat(9); sn_1 = xhat(10); dtheta = xhat(11); dgama = xhat(12); dphi = xhat(13);

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
A(7,11) = 1;
A(8,12) = 1;
A(9,13) = 1;



