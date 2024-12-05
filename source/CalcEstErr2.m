


PosHinx = find(PosH~=0);

% PosHm = PosH(indxSLAM(1:20000));

PosHm = PosH(PosHinx);
PosHm = PosH(PosHinx)-(-0.065+0.74*j)*exp(-j*HeadingH(PosHinx));

% PosHinx = find(PosHUWB);
% PosHm = PosHUWB(PosHinx)-(-0.065+0.74*j)*exp(-j*HeadingHUWB(PosHinx));

% PosHm = PosH(PosHinx)-(0.10+0.76*j)*exp(-j*HeadingH(PosHinx));
LidarPositionc = LidarPosition(:,1)+j*LidarPosition(:,2);

[TimeDiff] = DistMaxPoint(PosHm(60:800), PosHUWBtime(PosHinx(60:800)), LidarPositionc, LidarTime);

% LidarInx = find(LidarTime>PosHUWBtime(PosHinx(1))-22.655);
Mine_pos(1,:) = interp1(PosHUWBtime(PosHinx)+TimeDiff,real(PosHm),LidarTime);
Mine_pos(2,:) = interp1(PosHUWBtime(PosHinx)+TimeDiff,imag(PosHm),LidarTime);
% Mine_pos(1,:) = interp1(PosHUWBtime(PosHinx)+22.355,real(PosHm),LidarTime);
% Mine_pos(2,:) = interp1(PosHUWBtime(PosHinx)+22.355,imag(PosHm),LidarTime);
MinePosc = Mine_pos(1,:)+j*Mine_pos(2,:);
figure(98);hold off;plot(real(MinePosc),'bo');hold on;plot(imag(MinePosc),'ro')

KK = MinePosc(64:end)-MinePosc(63);
QQ = transpose(LidarPositionc(64:end)-LidarPositionc(63));
KKinx = find(abs(KK)>15);
QQinx = find(abs(QQ)>15);
KKQQinx = [];
for gwg = 1 : length(KKinx)
    if length(find(QQinx==KKinx(gwg)))~=0
        KKQQinx = [KKQQinx KKinx(gwg)];
    end
end

sA = mean(KK(KKQQinx)./QQ(KKQQinx));
sA = sA/1;

MinPosOffset = mean(MinePosc(63:end)-sA*transpose(LidarPositionc(63:end)));

TransLidar = sA*LidarPositionc(3:end)+MinPosOffset;

figure(97);hold off;plot(TransLidar,'b.')
hold on;plot(MinePosc,'r.');axis equal


% SE = ((abs(transpose(TransLidar(2:end))-MinePosc(4:end)).^2));
% MSE = sqrt(mean(SE))
% figure(1019);plot(SE,'.')

cnterr = 1;
ErrV = [];
for gq = 60 : 800
    % for gq = 4 : length(MinePosc)-3
    Gd = abs(TransLidar(2:end)-MinePosc(gq));
    [Gdsortv,Gdsorti] = sort(Gd,'ascend');
    a = abs(TransLidar(Gdsorti(1)+1)-TransLidar(Gdsorti(2)+1));
    b = abs(MinePosc(gq)-TransLidar(Gdsorti(2)+1));
    c = abs(TransLidar(Gdsorti(1)+1)-MinePosc(gq));
    sumTri = (a + b + c) / 2;
    if (sumTri < 1) && (a~=0)
        TempV = sqrt(sumTri * (sumTri - a) * (sumTri - b) * (sumTri - c));
        if TempV>=0
            ErrV(cnterr) = 2 * TempV / a;
            cnterr = cnterr+1;
        end
    end
end
MSEest = sqrt(mean(ErrV.^2))
figure(1018);plot(ErrV,'.')

figure(1021);hold off;plot(real(TransLidar),'r.');
hold on;plot(real(MinePosc),'g.')
figure(1021);plot(imag(TransLidar),'b.');
hold on;plot(imag(MinePosc),'k.');grid minor

figure(1022);hold off;plot(atan2(imag(MinePosc),real(MinePosc)));hold on;plot(atan2(imag(TransLidar),real(TransLidar)),'r')

% figure(1023);hold off;plot(PosH,'.');hold on;plot(MinePosc,'r.')
figure(1024);hold off;plot(LidarTime(3:end),real(TransLidar),'r.');
hold on;plot(PosHUWBtime(PosHinx)+TimeDiff, real(PosHm),'g.')
figure(1024);plot(LidarTime(3:end),imag(TransLidar),'b.');
hold on;plot(PosHUWBtime(PosHinx)+TimeDiff,imag(PosHm),'k.');grid minor
% figure(1024);hold off;plot(LidarTime(3:end),real(TransLidar),'r.');
% hold on;plot(PosHUWBtime(PosHinx)+22.355, real(PosHm),'g.')
% figure(1024);plot(LidarTime(3:end),imag(TransLidar),'b.');
% hold on;plot(PosHUWBtime(PosHinx)+22.355,imag(PosHm),'k.');grid minor
