function [Pos] = TwoAnchPos5(Xa, Ya, dist)
% Xa = [0 1];
% Ya = [0 0];
% dist = [1 1];

AA  = sqrt((Xa(1)-Xa(2))^2+(Ya(1)-Ya(2))^2);
B = dist(1);
C = dist(2);
% AA = 2;B=1;C=0.9
s = (AA+B+C)/2;
if (s*(s-AA)*(s-B)*(s-C)) > 0
    S = (sqrt(s*(s-AA)*(s-B)*(s-C)));
    d = 2*S/AA;

    % if (B^2-((B^2-C^2+AA^2)/(2*AA))^2) > 0
    %     d = sqrt(B^2-((B^2-C^2+AA^2)/(2*AA))^2);
    % else
    %     d = 0;
    % end

    A = [Ya(2)-Ya(1) -(Xa(2)-Xa(1));2*(Xa(2)-Xa(1)) 2*(Ya(2)-Ya(1))];
    Y1 = [d*sqrt((Ya(2)-Ya(1))^2+(Xa(2)-Xa(1))^2)+Xa(1)*Ya(2)-Xa(2)*Ya(1);(-C^2+B^2-Xa(1)^2+Xa(2)^2-Ya(1)^2+Ya(2)^2)];
    Y2 = [-d*sqrt((Ya(2)-Ya(1))^2+(Xa(2)-Xa(1))^2)+Xa(1)*Ya(2)-Xa(2)*Ya(1);(-C^2+B^2-Xa(1)^2+Xa(2)^2-Ya(1)^2+Ya(2)^2)];
    % Y1 = [d*sqrt((Ya(2)-Ya(1))^2+(Xa(2)-Xa(1))^2)+Xa(1)*Ya(2)-Xa(2)*Ya(1);(C^2-B^2-Xa(1)^2+Xa(2)^2-Ya(1)^2+Ya(2)^2)];
    % Y2 = [-d*sqrt((Ya(2)-Ya(1))^2+(Xa(2)-Xa(1))^2)+Xa(1)*Ya(2)-Xa(2)*Ya(1);(C^2-B^2-Xa(1)^2+Xa(2)^2-Ya(1)^2+Ya(2)^2)];

    X1 = inv(A'*A)*A'*Y1;
    X2 = inv(A'*A)*A'*Y2;

    % if X1 == X2
    %     Pos = [0;0];
    % else
    Pos = [X1';X2'];

else

    TT = mean(Xa);
    YY = mean(Ya);
    X1 = [TT;YY];
    Pos = [X1';X1'];
end
% end
