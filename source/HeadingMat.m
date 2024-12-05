function [PosUWBN, HeadingUWBN, PosUWBe, HeadingUWBe] = HeadingMat(DistMap, xain, yain, xt_b, yt_b, RxIDUWBa)

persistent PrePosUWBN PreHeadingUWBN PrePosUWBe PreHeadingUWBe

if isempty(PrePosUWBN)
    PrePosUWBN = 0;
    PreHeadingUWBN = 0;
    PrePosUWBe = 0;
    PreHeadingUWBe = 0;
end

Xb = xt_b+j*yt_b;
Xain = xain+j*yain;

ExpPos = transpose(PrePosUWBN+Xb*exp(j*PreHeadingUWBN));
DistExp = abs((ExpPos)-Xain);

[M,N] = size(DistMap,[2,3]);
A = [];
B = [];
Bp = [];
Ap = [];
C = [];
D = [];
Xl = [];
RxIDUWB = RxIDUWBa(RxIDUWBa~=0);
Th = 0.8;
if PreHeadingUWBe ~=0
    Temp = abs(DistExp-DistMap(:,1:length(Xain),end));
    Temp(DistMap(:,1:length(Xain),end)==0)=0;
    DistMapTemp = DistMap(1:4,1:length(Xain),end);

    % DistMapTemp(Temp>1) = DistExp(Temp>1);
    DistMapTemp(Temp>Th) = 0;
    if length(DistMapTemp(:)~=0) > 5
        DistMap(1:4,1:length(Xain),end) = DistMapTemp;
    end

    Temp = abs(DistExp-DistMap(:,1:length(Xain),end-1));
    Temp(DistMap(:,1:length(Xain),end-1)==0)=0;
    DistMapTemp = DistMap(1:4,1:length(Xain),end-1);

    % DistMapTemp(Temp>1) = DistExp(Temp>1);
    DistMapTemp(Temp>Th) = 0;
    if length(DistMapTemp(:)~=0) > 5
        DistMap(1:4,1:length(Xain),end-1) = DistMapTemp;
    end
end

% DistMapT = DistMap;
% 
% DistMap = DistMapT;
% DistMap(2,RxIDUWB(1),1:2)=0;
% DistMap(4,RxIDUWB(2),1:2)=0;
% DistMap(4,RxIDUWB(3),1:2)=0;

for p = 1 : 4
    templ = 0;
    for m = 1 : length(xain)-1
            for n = m+1:length(xain)
                if (DistMap(p,m,end)~=0) && (DistMap(p,n,end)~=0) % && (DistMap(p,RxIDUWB(m),end-1)~=0) && (DistMap(p,RxIDUWB(n),end-1)~=0)
                    A = [A;-2*(xain(m)-xain(n)) -2*(yain(m)-yain(n)) -2*((xain(m)-xain(n))*xt_b(p)+(yain(m)-yain(n))*yt_b(p)) 2*(((xain(m)-xain(n))*yt_b(p))-(yain(m)-yain(n))*xt_b(p))];
                    B = [B;DistMap(p,m,end)^2-DistMap(p,n,end)^2-xain(m)^2+xain(n)^2-yain(m)^2+yain(n)^2];
                    %Bp = [Bp;DistMap(p,RxIDUWB(m),end-1)^2-DistMap(p,RxIDUWB(n),end-1)^2-xain(RxIDUWB(m))^2+xain(RxIDUWB(n))^2-yain(RxIDUWB(m))^2+yain(RxIDUWB(n))^2];
                    
                    templ = templ + 1;
                end

                if (DistMap(p,m,end)~=0) && (DistMap(p,n,end)~=0) && (DistMap(p,m,end-1)~=0) && (DistMap(p,n,end-1)~=0)
                    C = [C;-2*(xain(m)-xain(n)) -2*(yain(m)-yain(n))];
                    D = [D;DistMap(p,m,end-1)^2-DistMap(p,n,end-1)^2-(DistMap(p,m,end))^2+(DistMap(p,n,end)^2)];
                end
                if (DistMap(p,m,end-1)~=0) && (DistMap(p,n,end-1)~=0)
                    Ap = [Ap;-2*(xain(m)-xain(n)) -2*(yain(m)-yain(n)) -2*((xain(m)-xain(n))*xt_b(p)+(yain(m)-yain(n))*yt_b(p)) 2*(((xain(m)-xain(n))*yt_b(p))-(yain(m)-yain(n))*xt_b(p))];
                    Bp = [Bp;DistMap(p,m,end-1)^2-DistMap(p,n,end-1)^2-xain(m)^2+xain(n)^2-yain(m)^2+yain(n)^2];
                end
            end
    end
    if templ == 0
        Xl=[Xl [0;0]];
    else
        Xl(:,p) = inv(transpose(A(end-templ+1:end,1:2))*A(end-templ+1:end,1:2))*transpose(A(end-templ+1:end,1:2))*B(end-templ+1:end);
        % Xlerr(:,p)=B(end-templ+1:end)-A(end-templ+1:end,1:2)*Xl(:,p);
    end
end

Xlc = Xl(1,:)+j*Xl(2,:);
for kkp = 1 : 4
    TempT= abs(Xlc(kkp)-Xain) - DistMap(kkp,1:length(Xain),2);
    DistDiff(kkp,:) = TempT;
    TempT(DistMap(kkp,1:length(Xain),2)==0) = [];
    SigmaDistDiff(kkp) = std(TempT);
end

HeadingCand = 0;
% Xlc = Xl(1,:)+j*Xl(2,:);
Xt_b = xt_b + j*yt_b;

for kkp = 1 : 3
    for kkq = kkp + 1 : 4
        if Xlc(kkp)~=0 && Xlc(kkq)~=0
            HeadingCand = HeadingCand + 1;
            HeadingCandV(HeadingCand) = (Xlc(kkp)-Xlc(kkq))/(Xt_b(kkp)-Xt_b(kkq));
        end
    end
end




X = inv(transpose(A)*A)*transpose(A)*B;
% Xt = inv(transpose(A(1:end-7,:))*A(1:end-7,:))*transpose(A(1:end-7,:))*B(1:end-7);

Dtheta = inv(transpose(C)*C)*transpose(C)*D;
Ae = [A;Ap];
Be = [B;Bp];

Xe = inv(transpose(Ae)*Ae)*transpose(Ae)*Be;

% %%%%%%%%%%%%%%
% 
% color = ['b','r','g','y'];
% figure(1);hold off;
% plot(xain,yain,'rsquare');
% hold on;
% for kkk = 1 : size(DistMap,1)
%     for lll = 1 : length(xain)
%         r = DistMap(kkk,lll,1);
%         if r ~= 0
%             center = [xain(lll),yain(lll)];
% 
%             % 원 그리기
%             rectangle('Position', [center(1) - r, center(2) - r, 2*r, 2*r], 'Curvature', [1, 1],'EdgeColor',color(kkk));
%             axis equal; % x축과 y축의 크기를 동일하게 맞춤
%             title('원 그리기');
%             xlabel('x');
%             ylabel('y');
%         end
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if length(X)~=0
PosUWBN = X(1)+X(2)*j;
Test = X(3)^2+X(4)^2;
% if abs(Test-1)
HeadingUWBN = atan(abs(X(4)/X(3)));
if X(4)>0 && X(3)<0
    HeadingUWBN = pi-HeadingUWBN;
elseif X(4)<0 && X(3)<0
    HeadingUWBN = pi+HeadingUWBN;
elseif X(4)<0 && X(3)>0
    HeadingUWBN = 2*pi-HeadingUWBN;
else
    HeadingUWBN = HeadingUWBN;
end

else
    PosUWBN = 0;
    HeadingUWBN = 0;
end

if length(Xe)~=0
PosUWBe = Xe(1)+Xe(2)*j;
Test = Xe(3)^2+Xe(4)^2;
% if abs(Test-1)
HeadingUWBe = atan(abs(Xe(4)/Xe(3)));
if Xe(4)>0 && Xe(3)<0
    HeadingUWBe = pi-HeadingUWBe;
elseif Xe(4)<0 && Xe(3)<0
    HeadingUWBe = pi+HeadingUWBe;
elseif Xe(4)<0 && Xe(3)>0
    HeadingUWBe = 2*pi-HeadingUWBe;
else
    HeadingUWBe = HeadingUWBe;
end

else
    PosUWBe = 0;
    HeadingUWBe = 0;
end

Xb = xt_b+j*yt_b;
Xain = xain+j*yain;

ExpPos = transpose(PosUWBN+Xb*exp(j*HeadingUWBN));
% abs(ExpPos-Xain)
% DistMap(:,1:length(Xain),end)

ErrorB = abs(abs(ExpPos-Xain)-DistMap(:,1:length(Xain),end));
ErrorB(DistMap(:,1:length(Xain),end)==0) = 0;

    PrePosUWBN = PosUWBN;
    PreHeadingUWBN = HeadingUWBN;
    PrePosUWBe = PosUWBe;
    PreHeadingUWBe = HeadingUWBe;

