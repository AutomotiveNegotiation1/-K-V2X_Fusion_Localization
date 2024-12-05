

% SLAMposInterp(1,:) = interp1(SLAMtime(4600:25000),transpose(SLAMpos(4600:25000,1)),PosHUWBtime(5000:24000));
% SLAMposInterp(2,:) = interp1(SLAMtime(4600:25000),transpose(SLAMpos(4600:25000,3)),PosHUWBtime(5000:24000));
% SLAMposInterp(3,:) = interp1(SLAMtime(4600:25000),transpose(SLAMpos(4600:25000,2)),PosHUWBtime(5000:24000));

SLAMposTr = transpose(SLAMpos);

tempB = find(SLAMposTr(1,1:end)~=0);
tempC = find(PosHUWB~=0);

[LsI] = min(find(SLAMtime(tempB(1)) < PosHUWBtime(tempC)));
[LfI] = max(find(SLAMtime(tempB(end)) > PosHUWBtime(tempC)));
Ls = tempC(LsI);
Lf = tempC(LfI);

SLAMposInterp = [];
SLAMposInterp(1,:) = interp1(SLAMtime(tempB),SLAMposTr(1,tempB),PosHUWBtime(tempC(LsI:LfI)));
SLAMposInterp(2,:) = interp1(SLAMtime(tempB),SLAMposTr(3,tempB),PosHUWBtime(tempC(LsI:LfI)));
SLAMposInterp(3,:) = interp1(SLAMtime(tempB),SLAMposTr(2,tempB),PosHUWBtime(tempC(LsI:LfI)));


Y = transpose(SLAMposInterp(3,1:600)); A = [transpose(SLAMposInterp(1:2,1:600)) ones(600,1)];
X = inv(transpose(A)*A)*transpose(A)*Y;

tep = asin(X);
MatX = eulr2dcm([-tep(2) tep(1) 0]);
Xtr = MatX'*SLAMposInterp;

figure(2134);plot3(Xtr(1,:),Xtr(2,:),Xtr(3,:),'.')
figure(2135);plot3(SLAMposInterp(1,:),SLAMposInterp(2,:),SLAMposInterp(3,:),'.')

Y = transpose(Xtr(3,:)); A = [transpose(Xtr(1:2,:)) ones(size(Xtr,2),1)];
X = inv(transpose(A)*A)*transpose(A)*Y;

UWBposVoffset = (-0.065+j*0.74)*exp(j*(-HeadingHUWB+pi/2));
UWBposV = [(real(UWBposVoffset(tempC(LsI:LfI))+PosHUWB(tempC(LsI:LfI))));(imag(UWBposVoffset(tempC(LsI:LfI))+PosHUWB(tempC(LsI:LfI))));2.3*ones(1,(LfI-LsI+1))];

UWBposVc = (UWBposV(1,:)+j*UWBposV(2,:));
Xtrc = Xtr(1,:)+j*Xtr(2,:);

TempDU = (UWBposVc(1:end-1))-(UWBposVc(end));
TempDS = (Xtrc(1:end-1))-(Xtrc(end));
TempDU = [TempDU (UWBposVc(2:end))-(UWBposVc(1))];
TempDS = [TempDS (Xtrc(2:end))-(Xtrc(1))];
TempDU = [TempDU (UWBposVc([1:299,301:end]))-(UWBposVc(300))];
TempDS = [TempDS (Xtrc([1:299,301:end]))-(Xtrc(300))];

TempDUs = [UWBposV(1:2,:)];
TempDSs = [Xtr(1:2,:);ones(1,size(Xtr,2))];

Am = TempDUs*transpose(TempDSs)*inv(TempDSs*transpose(TempDSs));


% sA = TempDU*transpose(TempDS)*inv(TempDS*transpose(TempDS));
sA = mean(TempDU./TempDS);
s = abs(sA);
% s = abs(sA)*1.03;

theta = angle(sA/s);
% theta = angle(sA/s)+0.14;
% s = s*1.03;
% sA = s*[cos(theta) -sin(theta) 0 ; sin(theta) cos(theta) 0; 0 0 1]*MatX';
AmE = eye(3);
AmE(1:2,1:2) = Am(:,1:2);
sA = AmE*MatX';
SLAMposInit = [Am(:,3);0];
% SLAMposInit = mean(UWBposV-sA*SLAMposInterp,2);
% SLAMposInit = mean(UWBposV-sA*SLAMposInterp,2)+[-0.5; 0.5; 0];

TempTr = sA*SLAMposInterp+SLAMposInit;


UWBposVoffset = (-0.065+j*0.74)*exp(j*(-HeadingHUWB));
UWBposV = [(real(UWBposVoffset+PosHUWB));(imag(UWBposVoffset+PosHUWB));2.3*ones(1,length(UWBposVoffset))];

s = sqrt(sum(abs(sA*[1;0;0]).^2))
TempTrAll = sA*SLAMposTr([1 3 2],:)+SLAMposInit;

ParkingLot = uint8([]);
ParkingLotS= imread('B1_ParkingLot.jpg');
for sx = 1:size(ParkingLotS,1)
    for sy = 1:size(ParkingLotS,2)
        % ParkingLot(sy,sx,:) = ParkingLotS(size(ParkingLotS,1)-sx+1,size(ParkingLotS,2)-sy+1,:);
        ParkingLot(sy,sx,:) = ParkingLotS(sx,size(ParkingLotS,2)-sy+1,:);
    end
end


figure(2313);hold off;
imshow(ParkingLot); 
Scale = 24.6;
hold on;plot(Scale*xa+j*(size(ParkingLot,2)-Scale*ya-Scale*11),'ro');
plot(Scale*UWBposV(1,:),size(ParkingLot,2)-Scale*UWBposV(2,:)-Scale*11,'b.');hold on;plot(Scale*TempTrAll(1,:),size(ParkingLot,2)-Scale*TempTrAll(2,:)-Scale*11,'r.')
% figure(3);plot(xa+j*ya,'ro');
axis equal

figure(2316);plot3(TempTrAll(1,:),TempTrAll(2,:),-TempTrAll(3,:),'.')
figure(2317);plot3(SLAMposTr(1,:),SLAMposTr(3,:),-SLAMposTr(2,:),'.')

% figure(2313);hold on;plot(UWBposV(1,:),UWBposV(2,:),'b.');hold on;plot(TempTr(1,:),TempTr(2,:),'r.')
figure(2315);hold off;plot(UWBposV(1,:),UWBposV(2,:),'b.');hold on;plot(TempTrAll(1,:),TempTrAll(2,:),'r.')
axis equal



figure(2314);hold off;plot(PosHUWBtime(tempC),UWBposV(1,tempC),'b.');hold on;plot(PosHUWBtime(tempC),UWBposV(2,tempC),'r.');plot(PosHUWBtime(tempC(LsI:LfI)),TempTr(1,(LsI:LfI)),'g.');plot(PosHUWBtime(tempC(LsI:LfI)),TempTr(2,(LsI:LfI)),'k.')


save('MapParams3.mat','sA','SLAMposInit');