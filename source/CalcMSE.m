function [MSEest] = CalcMSE(PosH, HeadingH, PosHUWBtime, LidarPosition, LidarTime, UWBpos)

Front = 0;
PosHinx = find(PosH~=0);
if UWBpos~=1
    PosHm = PosH(PosHinx)-(-0.065+0.74*j)*exp(-j*HeadingH(PosHinx));
    if Front == 1
    PosHm = PosH(PosHinx)-(-0.065+0.77*j)*exp(-j*HeadingH(PosHinx));
    else
    PosHm = PosH(PosHinx)-(-0.06-0.77*j)*exp(j*HeadingH(PosHinx));
    end
else
    PosHm = PosH(PosHinx);
end

% PosHm = PosH(PosHinx)-(-0.165+0.54*j)*exp(-j*HeadingH(PosHinx));
LidarPositionc = LidarPosition(:,1)+j*LidarPosition(:,2);

[TimeDiff] = DistMaxPoint(PosHm, PosHUWBtime(PosHinx), LidarPositionc, LidarTime);

% LidarInx = find(LidarTime>PosHUWBtime(PosHinx(1))-22.655);
TemA = find(LidarTime<(PosHUWBtime(PosHinx(end))+TimeDiff));

Mine_pos(1,:) = interp1(PosHUWBtime(PosHinx)+TimeDiff,real(PosHm),LidarTime(TemA));
Mine_pos(2,:) = interp1(PosHUWBtime(PosHinx)+TimeDiff,imag(PosHm),LidarTime(TemA));
% Mine_pos(1,:) = interp1(PosHUWBtime(PosHinx)+22.355,real(PosHm),LidarTime);
% Mine_pos(2,:) = interp1(PosHUWBtime(PosHinx)+22.355,imag(PosHm),LidarTime);
MinePosc = Mine_pos(1,:)+j*Mine_pos(2,:);
figure(98);hold off;plot(real(MinePosc),'bo');hold on;plot(imag(MinePosc),'ro')
figure(98);plot(real(LidarPositionc(TemA)),'go');hold on;plot(imag(LidarPositionc(TemA)),'ko')


KK = MinePosc(64:end)-MinePosc(63);
QQ = transpose(LidarPositionc(64:TemA(end))-LidarPositionc(63));
[VK,KKinx] = sort(abs(KK),'descend');
[VQ,QQinx] = sort(abs(KK),'descend');

% KKinx = find(abs(KK)>10);
% QQinx = find(abs(QQ)>10);

KKQQinx = [];
%for gwg = 1 : length(KKinx)
for gwg = 1 : 100
    if length(find(QQinx==KKinx(gwg)))~=0
        KKQQinx = [KKQQinx KKinx(gwg)];
    end
end

sA = mean(KK(KKQQinx)./QQ(KKQQinx));
% sA = sA/abs(sA);

MinPosOffset = mean(MinePosc(63:end)-sA*transpose(LidarPositionc(63:TemA(end))));

TransLidar = sA*LidarPositionc(3:TemA(end))+MinPosOffset;

figure(98);hold off;plot(real(MinePosc),'bo');hold on;plot(imag(MinePosc),'ro')
hold on;plot(real(TransLidar),'go');hold on;plot(imag(TransLidar),'ko')

figure(97);hold off;plot(TransLidar,'b.')
hold on;plot(MinePosc,'r.');axis equal


% SE = ((abs(transpose(TransLidar(2:end))-MinePosc(4:end)).^2));
% MSE = sqrt(mean(SE))
% figure(1019);plot(SE,'.')

cnterr = 1;
ErrV = [];
for gq = 60 : length(MinePosc)
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

figure(1021);hold off;plot(LidarTime(3:TemA(end)),real(TransLidar),'r.');
hold on;plot(LidarTime(TemA),real(MinePosc),'g.')
figure(1021);plot(LidarTime(3:TemA(end)),imag(TransLidar),'b.');
hold on;plot(LidarTime(TemA),imag(MinePosc),'k.');grid minor

figure(1022);hold off;plot(LidarTime(TemA),atan2(imag(MinePosc),real(MinePosc)),'b.');hold on;plot(LidarTime(3:TemA(end)),atan2(imag(TransLidar),real(TransLidar)),'r.')

% figure(1023);hold off;plot(PosH,'.');hold on;plot(MinePosc,'r.')
% figure(1024);hold off;plot(LidarTime(3:end),real(TransLidar),'r.');
% hold on;plot(PosHUWBtime(PosHinx)+TimeDiff, real(PosHm),'g.')
% figure(1024);plot(LidarTime(3:end),imag(TransLidar),'b.');
% hold on;plot(PosHUWBtime(PosHinx)+TimeDiff,imag(PosHm),'k.');grid minor
% figure(1024);hold off;plot(LidarTime(3:end),real(TransLidar),'r.');
% hold on;plot(PosHUWBtime(PosHinx)+22.355, real(PosHm),'g.')
% figure(1024);plot(LidarTime(3:end),imag(TransLidar),'b.');
% hold on;plot(PosHUWBtime(PosHinx)+22.355,imag(PosHm),'k.');grid minor
