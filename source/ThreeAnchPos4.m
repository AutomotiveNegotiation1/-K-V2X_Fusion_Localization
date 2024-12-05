function [Pos]=ThreeAnchPos4(xa, ya, dist)
M = length(dist);
A = zeros(M-1,2);
y = zeros(M-1,1);


% if (ya(2)-y(1))*(xa(3)-xa(1)) ~= (ya(3)-y(1))*(xa(2)-xa(1))
    for k = 1 : M-1
        if (dist(k)~=0)&&(dist(M)~=0)
            A(k,:) = [-2*(xa(k)-xa(M)) -2*(ya(k)-ya(M))];
            y(k,1) = dist(k)^2-dist(M)^2-(xa(k)^2-xa(M)^2)-(ya(k)^2-ya(M)^2);
        end
    end
% else
%     
% end


% Pos = (A'*A)\(A'*y);
Pos = (A'*A)\A'*y;
if isnan(Pos(1)) ||isnan(Pos(2))
    Pos = [0;0];
end

% Prob = sqrt(mean(abs((xa - Pos(1)).^2 + (ya - Pos(2)).^2 - dist.^2)));

                      