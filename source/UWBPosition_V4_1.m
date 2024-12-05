function [PosUWBO, HeadingUWBO, UWBFull] = UWBPosition_V4_1(s_time, Ln, LnC, Nanchor, TagID, RxIDUWB, RxDistOrig, xain, yain, zain, xt_b, yt_b, zt_b, PrevPos, PrevHeading)

persistent s_time_prev DistMap PosUWB PosUWB2 PosMap k PrevPosUWB PrevHeadingUWB PosUWBN HeadingUWBN PosUWBe HeadingUWBe

if isempty (s_time_prev)
    s_time_prev = 0;
    k = 1;
    PosUWB = zeros(4,1)+0*j;
    PosUWB2 = cell(1,4);
    PosUWB2{1} = 0;
    PosUWB2{2} = 0;
    PosUWB2{3} = 0;
    PosUWB2{4} = 0;
    
    DistMap = zeros(4,Ln,100);
    PosMap = zeros(4,100)+0*j;

    PrevPosUWB = 0+0*j;
    PrevHeadingUWB = 0;
    PosUWBN = 0;
    HeadingUWBN = 0;
        PosUWBe = 0;
    HeadingUWBe = 0;
end

dT = s_time - s_time_prev;
RxDist = real(sqrt(RxDistOrig.^2-(zain(1)-zt_b)^2));

PrevPosH = PrevPos + (xt_b+j*yt_b)*exp(j*PrevHeading);
PrevPosUWBH = PrevPosUWB + (xt_b+j*yt_b)*exp(j*PrevHeadingUWB);

if dT > 0.05
    k = k + 1;
    DistMap(:,:,1:end-1) = DistMap(:,:,2:end);
    DistMap(:,:,end) = 0;
    DistMap(TagID,RxIDUWB,end) = RxDist;

    % DistMap(:,1:length(xain),end) = abs(transpose(PrevPosUWBH)-(xain+j*yain));
    % % 
    % for ggl = 1 : length(RxDist)
    %     if abs(DistMap(TagID,RxIDUWB(ggl),end) - RxDist(ggl)) < 0.5
    %         DistMap(TagID,RxIDUWB(ggl),end) = RxDist(ggl);
    %     else
    %         gdg = 1;
    %     end
    % end
    % [PosUWBN(k), HeadingUWBN(k), PosUWBe(k), HeadingUWBe(k)] = HeadingMat(DistMap(:,:,end-2:end-1), xain, yain, xt_b, yt_b, RxIDUWB);
else
    DistMap(TagID,RxIDUWB,end) = RxDist;
end

if (length(RxIDUWB)>1) 

    if Nanchor ~=0
        [Pos1, Pos2, Pos3] = UWBpos_V2_3(Nanchor, RxIDUWB, RxDist, xain, yain, Ln);
    else
        Pos1 = 0;
        Pos2 = 0;
        Pos3 = 0;
    end

else
    Pos1 = 0;
    Pos2 = 0;
    Pos3 = 0;
end

PosUWB(TagID) = Pos1;
TPos = zeros(1,length(Pos3))+0*j;
Xain = xain+j*yain;
sg = 1;
gs = 1;
Pos3T = zeros(1,length(Pos3))+0*j;

for kd = 1 : length(Pos3)/2
    if Pos3((kd-1)*2+1)~=Pos3((kd-1)*2+2)
        Pos3T(gs:gs+1) = Pos3((kd-1)*2+1:(kd-1)*2+2);
        gs = gs + 2;
    end
end

Pos3T(Pos3T==0)=[];

for ig = 1 : length(Pos3T)

    Temp = abs(RxDist-abs(Pos3T(ig)-Xain(RxIDUWB)));
    if (Temp<5)
        TPos(sg) = Pos3T(ig);
        sg = sg + 1;
    end
end

TPos(sg:end) = [];
if length(TPos)==0
    PosUWB2{TagID} = Pos3;
else
    % PosUWB2{TagID} = Pos3;
    PosUWB2{TagID} = TPos;
end

% DistFromPrev = abs(PrevPosH(TagID)-Pos2);
% DistFromPrevUWB = abs(PrevPosUWBH(TagID)-Pos2);
% 
% if (PrevPosUWBH(TagID)~=0) && (abs(PrevPosH(TagID)-PrevPosUWBH(TagID))<2)
%     [DistFromPrevSorted, sortedIndx] = sort(DistFromPrev+DistFromPrevUWB);
%     PosUWB2{TagID} = Pos2(sortedIndx(1:min(length(sortedIndx))));
% else
%     PosUWB2{TagID} = Pos2;
% end


if (checkFull_1(DistMap,2)>2)
    % if length(DistMap(:,:,end-1)~=0)==16
    % [PosUWBN(k), HeadingUWBN(k), PosUWBe(k), HeadingUWBe(k)] = HeadingMat_2(DistMap(:,:,end-5:end-1), xain, yain, xt_b, yt_b, RxIDUWB);
    
    %%%% 테스트를 위해 잠시 off
    [PosUWBO, HeadingUWBO] = UWBMultiTagPos_V3_1(PosUWB2, xt_b, yt_b, xain, yain, DistMap, Ln);
    % PosUWBO = (PosUWBO+PosUWBN(k))/2;
    % HeadingUWBO = HeadingUWBN(k);
    
    % [PosUWBO, HeadingUWBO] = UWBMultiTagPos_V3(PosUWB2, xt_b, yt_b, xain, yain, DistMap);
        % [PosUWBO, HeadingUWBO] = UWBMultiTagPos_V2(PosUWB2, xt_b, yt_b);

    UWBFull = 1;
else
    % [PosUWBO, HeadingUWBO] = UWBMultiTagPos_V2(PosUWB2, xt_b, yt_b);
    PosUWBO = 0;
    HeadingUWBO = 0;
    UWBFull = 0;
end

PrevPosUWB = PosUWBO;
PrevHeadingUWB = HeadingUWBO;
s_time_prev = s_time;

