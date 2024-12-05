
% load('MapParams2.mat','sA','SLAMposInit');
load('MapParams4.mat','sA','SLAMposInit');

SLAMposTr = transpose(SLAMpos);
theta = 0.03;
sA = [cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1]*sA;
SLAMposInit = SLAMposInit + [0;1;0];

UWBposVoffset = (-0.065+0.74*j)*exp(-j*(HeadingHUWB+pi/2));
UWBposVoffset = (-0.06-0.77*j)*exp(-j*(HeadingHUWB+pi/2));

UWBposV = [(real(UWBposVoffset+PosHUWB));(imag(UWBposVoffset+PosHUWB));2.3*ones(1,length(UWBposVoffset))];

TempTrAll = sA*SLAMposTr([1 3 2],:)+SLAMposInit;

ParkingLot = uint8([]);
ParkingLotS= imread('B1_ParkingLot.jpg');
for sx = 1:size(ParkingLotS,1)
    for sy = 1:size(ParkingLotS,2)
        % ParkingLot(sy,sx,:) = ParkingLotS(size(ParkingLotS,1)-sx+1,size(ParkingLotS,2)-sy+1,:);
        ParkingLot(sy,sx,:) = ParkingLotS(sx,size(ParkingLotS,2)-sy+1,:);
    end
end


figure(2314);hold off;
imshow(ParkingLot); 
Scale = 25.3;
hold on;plot(Scale*xa+j*(size(ParkingLot,2)-Scale*ya-Scale*11),'ro');
plot(Scale*UWBposV(1,:)-Scale*0.3,size(ParkingLot,2)-Scale*UWBposV(2,:)-Scale*11,'b.');
hold on;plot(Scale*TempTrAll(1,:)-Scale*0.3,size(ParkingLot,2)-Scale*TempTrAll(2,:)-Scale*11,'r.')
% figure(3);plot(xa+j*ya,'ro');
axis equal

figure(2316);plot3(TempTrAll(1,:),TempTrAll(2,:),-TempTrAll(3,:),'.')
figure(2317);plot3(SLAMposTr(1,:),SLAMposTr(3,:),-SLAMposTr(2,:),'.')

figure(1231);hold off;plot(mod(HeadingHUWB,2*pi)-pi,'.')

eul = dcm2eulr(sA);
for dfd = 1 : size(SLAMorient,1)
    dcmSLAM = qua2dcm(SLAMorient(dfd,:));
    temp = dcmSLAM*[1;0;0];
    edl = angle(temp(1)+j*temp(3));
    HeadingSLAM(dfd) = (edl+eul(3));
    % HeadingSLAM(dfd) = (edl+pi);
end
figure(1231);hold on;plot(mod(HeadingSLAM,2*pi)-pi,'r.')


