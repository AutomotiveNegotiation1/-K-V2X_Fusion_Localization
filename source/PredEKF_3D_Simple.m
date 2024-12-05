function [Xbar] = PredEKF_3D_Simple(Xhat,acc,p, dT)

Pxh = Xhat(1); Pyh = Xhat(2); Pzh = Xhat(3);
Vxh = Xhat(4); Vyh = Xhat(5); Vzh = Xhat(6); 
abxh = Xhat(7); abyh = Xhat(8); abzh = Xhat(9);
alph = Xhat(10); beth = Xhat(11); gamh = Xhat(12);
pbh = Xhat(13);qbh = Xhat(14);rbh = Xhat(15);

RotMat = [sin(alph)*cos(beth) sin(alph)*sin(beth)*sin(gamh)+cos(alph)*cos(gamh) sin(alph)*sin(beth)*cos(gamh)-cos(alph)*sin(gamh); ...
          cos(alph)*cos(beth) cos(alph)*sin(beth)*sin(gamh)-sin(alph)*cos(gamh) cos(alph)*sin(beth)*cos(gamh)+sin(alph)*sin(gamh); ...
          -sin(beth)          cos(beth)*sin(gamh)                               cos(beth)*cos(gamh)];

ZyroUpdate = [1 sin(gamh)*tan(beth) cos(gamh)*tan(beth); ...
              0 cos(gamh)           -sin(gamh); ...
              0 sin(gamh)/cos(beth) cos(gamh)/cos(beth)];

Pxyzb = [Pxh;Pyh;Pzh] + [Vxh;Vyh;Vzh]*dT + 0.5*RotMat*[(acc(1)-abxh);(acc(2)-abyh);(acc(3)-abzh)]*dT^2;
Pxb = Pxyzb(1);Pyb = Pxyzb(2);Pzb = Pxyzb(3);

Vxyzb = [Vxh;Vyh;Vzh] + RotMat*[(acc(1)-abxh);(acc(2)-abyh);(acc(3)-abzh)]*dT;
Vxb = Vxyzb(1);Vyb = Vxyzb(2);Vzb = Vxyzb(3);

albega = [gamh;beth;alph] + ZyroUpdate*[p(1)-pbh;p(2)-qbh;p(3)-rbh]*dT;
gamb = albega(1);betb = albega(2); alpb = albega(3);

% %%% X Y change
% Pxb = Pxh + Vxh*dT + 0.5*((acc(1)-abxh)*sin(alph)+(acc(2)-abyh)*cos(alph))*dT^2;
% Pyb = Pyh + Vyh*dT + 0.5*((acc(1)-abxh)*cos(alph)+(acc(2)-abyh)*(-sin(alph)))*dT^2;
% 
% Vxb = Vxh + ((acc(1)-abxh)*sin(alph)+(acc(2)-abyh)*cos(alph))*dT;
% Vyb = Vyh + ((acc(1)-abxh)*cos(alph)+(acc(2)-abyh)*(-sin(alph)))*dT;

%%% X Y original
% Pxb = Pxh + Vxh*dT + 0.5*((acc(1)-abxh)*cos(alph)*cos(betah)+(acc(2)-abyh)*(cos(alph)*sin(betah)*sin(gammah)-sin(alph)*cos(gammah))+(acc(3)-abzh)*(cos(alph)*sin(betah)*cos(gammah)+sin(alph)*sin(gammah)))*dT^2;
% Pyb = Pyh + Vyh*dT + 0.5*((acc(1)-abxh)*sin(alph)*cos(betah)+(acc(2)-abyh)*(sin(alph)*sin(betah)*sin(gammah)+cos(alph)*cos(gammah))+(acc(3)-abzh)*(sin(alph)*sin(betah)*cos(gammah)-cos(alph)*sin(gammah)))*dT^2;
% Pzb = Pzh + Vzh*dT + 0.5*(-1*(acc(1)-abxh)*sin(betah)+(acc(2)-abyh)*(cos(betah)*sin(gammah))+(acc(3)-abzh)*(cos(betah)*cos(gammah)))*dT^2;
% 
% Vxb = Vxh + ((acc(1)-abxh)*cos(alph)*cos(betah)+(acc(2)-abyh)*(cos(alph)*sin(betah)*sin(gammah)-sin(alph)*cos(gammah))+(acc(3)-abzh)*(cos(alph)*sin(betah)*cos(gammah)+sin(alph)*sin(gammah)))*dT;
% Vyb = Vyh + ((acc(1)-abxh)*sin(alph)*cos(betah)+(acc(2)-abyh)*(sin(alph)*sin(betah)*sin(gammah)+cos(alph)*cos(gammah))+(acc(3)-abzh)*(sin(alph)*sin(betah)*cos(gammah)-cos(alph)*sin(gammah)))*dT;
% Vzb = Vzh + (-1*(acc(1)-abxh)*sin(betah)+(acc(2)-abyh)*(cos(betah)*sin(gammah))+(acc(3)-abzh)*(cos(betah)*cos(gammah)))*dT;


abxb = abxh; abyb = abyh; abzb = abzh;

pbb = pbh; qbb = qbh ; rbb = rbh;


%%%%%%%% 발산 방지%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (abs(Vxb)>100) || abs(Vyb>100) || abs(Vzb>100) 
    Vxb = 6;
    Vyb = 6;
    Vzb = 0;
    Pxyzb = [Pxh;Pyh;Pzh] ;
    Pxb = Pxyzb(1);Pyb = Pxyzb(2);Pzb = Pxyzb(3);
    abxb = 0;
    abyb = 0;
    abzb = 0;
    alpb = 0;
    betb = 0;
    gamb = 0;
    Xbar = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
else
    Xbar = [Pxb Pyb Pzb Vxb Vyb Vzb abxb abyb abzb alpb betb gamb pbb qbb rbb];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Xbar = [Pxb Pyb Pzb Vxb Vyb Vzb abxb abyb abzb alpb betb gamb pbb qbb rbb];



