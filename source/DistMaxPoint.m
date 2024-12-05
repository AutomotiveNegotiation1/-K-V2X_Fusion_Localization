function [TimeDiff] = DistMaxPoint(PosHm, PosHUWBtime, LidarPositionc, LidarTime)

ATp = abs(PosHm-transpose(PosHm));
BTp = abs(LidarPositionc-transpose(LidarPositionc));

[Ku,Lu] = size(ATp);
AT = zeros(Ku,Lu);
for dd = 1 : Ku
    for gg = dd+1 : Ku
        AT(dd,gg) = ATp(dd,gg);
    end
end

ATflat = AT(:);
[SortAtv, SortAti] = sort(ATflat,'descend');

[Kl,Ll] = size(BTp);
BT = zeros(Kl,Ll);
for dd = 1 : Kl
    for gg = dd+1 : Kl
        BT(dd,gg) = BTp(dd,gg);
    end
end

BTflat = BT(:);
[SortBtv, SortBti] = sort(BTflat,'descend');


for K = 1 : 100
    for L = 1 : 100
        ATk = mod(SortAti(K),Ku);
        ATl = (SortAti(K)-ATk)/Ku;

        BTk = mod(SortBti(L),Kl);
        BTl = (SortBti(L)-BTk)/Kl;

        TimeDiffErr(K,L) = abs((PosHUWBtime(ATl)-PosHUWBtime(ATk))-(LidarTime(BTl)-LidarTime(BTk)));
        TimeDiffV(K,L) = ((LidarTime(BTl)-PosHUWBtime(ATl))+(LidarTime(BTk)-PosHUWBtime(ATk)))/2;
    end
end

[minv, mini]= min(TimeDiffErr);
[minvv, minii] = min(minv);
KK = mini(minii);
LL = minii;

TimeDiff = TimeDiffV(KK,LL);