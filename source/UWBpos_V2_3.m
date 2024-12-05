function [Pos1,Pos2,Pos3] = UWBpos_V2_3(Nanchor, RxIDin, RxDistin, xain, yain, Ln)

RxID = RxIDin(1:Nanchor);
xa = xain(RxID);
ya = yain(RxID);

RxDist = RxDistin(1:Nanchor);

Pos2 = zeros(1,6)+0*j;
Pos3 = zeros(1,12)+0*j;

v = 1;
if Nanchor == 2
    [Pos] = TwoAnchPos5(xa, ya, RxDist);
    Pos1 = 0;
    Pos2(1) = Pos(1,1)+j*Pos(1,2);
    Pos2(2) = Pos(2,1)+j*Pos(2,2);
    Pos2(3:end) = [];
    Pos3 = Pos2;
elseif Nanchor >= 3
    [Pos] = ThreeAnchPos4(xa, ya, RxDist);
    Pos1 = Pos(1)+j*Pos(2);
    if isnan(Pos1)
        Pos1 = 0;
    end

    for k = 1 : Nanchor - 1
        for l = k+1: Nanchor
            if RxDist([k,l])~=0
            [Pos] = TwoAnchPos5(xa([k,l]), ya([k,l]), RxDist([k,l]));
            PosC1 = Pos(1,1)+j*Pos(1,2);
            PosC2 = Pos(2,1)+j*Pos(2,2);
            if abs(Pos1-PosC1)>abs(Pos1-PosC2)
                Pos2(v) = PosC2;
            else
                Pos2(v) = PosC1;
                
            end
            Pos3(2*v-1:2*v) = [PosC1 PosC2];
            v = v + 1;
            end
        end
    end


    Temp = abs(Pos3-transpose(Pos3)).^2;
    meanTempW = mean(Temp);

    L = length(Pos3);
    Pos2n = zeros(1,L/2)+0*j;

    for kk = 0 : L/2-1
        [v,vi] = min(meanTempW(2*kk+1:2*kk+2));
        % meanTemp(kk+1) = v;
        Pos2n(kk+1) = Pos3(2*kk+vi);
    end
    Pos2 = Pos2n;

else 
    Pos1 = 0;
    Pos2 = 0;
end

