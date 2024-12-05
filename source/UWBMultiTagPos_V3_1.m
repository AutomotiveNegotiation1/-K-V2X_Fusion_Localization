function [PosHH, HeadingHH] = UWBMultiTagPos_V3_1(PosUWB2, xt_b, yt_b, xain, yain, DistMap, Ln)

Xt_b = xt_b+j*yt_b;
Xain = zeros(1,Ln)+0*j;
Xain(1:length(xain)) = xain+j*yain;

HeadingCands = zeros(1,20000)+0*j;
CenterCands = zeros(1,20000)+0*j;
TagCandA = zeros(20000,4)+0*j;
LessThan1m1 = zeros(1,20000);
ss = 1;
Temp = zeros(1,4)+0*j;
PrevAbsHeadingCandA = 100;
PrevCenterCandA = 0+0*j;
PrevHeadingCandA = 0+0*j;

for kk = 1 : length(PosUWB2{1})
    Temp(1) = PosUWB2{1}(kk);
                
    for ll = 1 : length(PosUWB2{2})
        Temp(2) = PosUWB2{2}(ll);
                
        for mm = 1 : length(PosUWB2{3})
            Temp(3) = PosUWB2{3}(mm);
                
            for nn = 1 : length(PosUWB2{4})
                Temp(4) = PosUWB2{4}(nn);

                if length(Temp(Temp==0)) == 0
                    CenterCandA = mean(Temp);
                    HeadingCandA = mean((Temp-CenterCandA)./Xt_b);
                elseif (Temp(1)~=0)&&(Temp(4)~=0)
                    CenterCandA = mean(Temp([1,4]));
                    HeadingCandA = mean((Temp([1,4])-CenterCandA)./Xt_b([1,4]));
                elseif (Temp(2)~=0)&&(Temp(3)~=0)
                    CenterCandA = mean(Temp([2,3]));
                    HeadingCandA = mean((Temp([2,3])-CenterCandA)./Xt_b([2,3]));
                elseif (Temp(1)~=0)&&(Temp(2)~=0)
                    HeadingCandA = (Temp(1)-Temp(2))/(Xt_b(1)-Xt_b(2));
                    CenterCandA = (Temp(1)+Temp(2))/2-(Xt_b(1)+Xt_b(2))/2*HeadingCandA/abs(HeadingCandA);
                elseif (Temp(2)~=0)&&(Temp(4)~=0)
                    HeadingCandA = (Temp(2)-Temp(4))/(Xt_b(2)-Xt_b(4));
                    CenterCandA = (Temp(2)+Temp(4))/2-(Xt_b(2)+Xt_b(4))/2*HeadingCandA/abs(HeadingCandA);
                elseif (Temp(3)~=0)&&(Temp(4)~=0)
                    HeadingCandA = (Temp(3)-Temp(4))/(Xt_b(3)-Xt_b(4));
                    CenterCandA = (Temp(3)+Temp(4))/2-(Xt_b(3)+Xt_b(4))/2*HeadingCandA/abs(HeadingCandA);
                elseif (Temp(3)~=0)&&(Temp(1)~=0)
                    HeadingCandA = (Temp(3)-Temp(1))/(Xt_b(3)-Xt_b(1));
                    CenterCandA = (Temp(3)+Temp(1))/2-(Xt_b(3)+Xt_b(1))/2*HeadingCandA/abs(HeadingCandA);


                else
                    CenterCandA = mean(Temp);
                    HeadingCandA = mean((Temp-CenterCandA)./Xt_b);
                end
                if abs(abs(HeadingCandA)-1)<=1
                    TagCandA(ss,:) = CenterCandA+Xt_b*HeadingCandA/abs(HeadingCandA);
                    DistErr = abs(DistMap(:,:,end)-abs(transpose(TagCandA(ss,:))-Xain));
                    [minv] = min(DistErr(DistErr~=0));
                    LessThan1m1(ss) = length(find(DistErr(:)<1) );
                    CenterCands(ss) = CenterCandA;
                    HeadingCands(ss) = HeadingCandA;
                    ss = ss + 1;
                else
                    if abs(abs(HeadingCandA)-1) < PrevAbsHeadingCandA
                        PrevAbsHeadingCandA = abs(abs(HeadingCandA)-1);
                        PrevCenterCandA = CenterCandA;
                        PrevHeadingCandA = HeadingCandA;
                    end
                end
            end
        end
    end
end
if (ss == 1) && (PrevAbsHeadingCandA == 100)
    PosHH = 0;
    HeadingHH = 0;
elseif (ss == 1)
    PosHH = PrevCenterCandA;
    HeadingHH = angle(PrevHeadingCandA);
else
    [SortV, SortI] = sort(LessThan1m1,'descend');
    Tnum = length(find(LessThan1m1==SortV(1)));

    sortedCenter = CenterCands(SortI(1:Tnum));
    sortedHeading = HeadingCands(SortI(1:Tnum)); 
    PosHH = mean(sortedCenter);
    HeadingHH = angle(mean(sortedHeading));

end

Fpos = PosHH+exp(j*HeadingHH)*Xt_b;
TempA = abs(transpose(Fpos) - Xain)-DistMap(:,:,end);

TempA(DistMap(:,:,end)==0) = 0;
TempB = TempA(:);
TempB(TempB==0) = [];
TTa = sqrt(mean(abs(TempB).^2));
if (TTa > 0.5) || (length(TempB)<11)
    PosHH = 0;
end

gg = 1;
