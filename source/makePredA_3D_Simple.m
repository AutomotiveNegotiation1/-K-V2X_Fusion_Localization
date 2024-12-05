function [A] = makePredA_3D_Simple(Xhat,acc,p, dT)

Pxh = Xhat(1); Pyh = Xhat(2); Pzh = Xhat(3);
Vxh = Xhat(4); Vyh = Xhat(5); Vzh = Xhat(6); 
abxh = Xhat(7); abyh = Xhat(8); abzh = Xhat(9);
alph = Xhat(10); beth = Xhat(11); gamh = Xhat(12);
pbh = Xhat(13);qbh = Xhat(14);rbh = Xhat(15);

N = length(Xhat);

A = eye(N);

RotMat = [sin(alph)*cos(beth)  sin(alph)*sin(beth)*sin(gamh)+cos(alph)*cos(gamh)  sin(alph)*sin(beth)*cos(gamh)-cos(alph)*sin(gamh); ...
          cos(alph)*cos(beth)  cos(alph)*sin(beth)*sin(gamh)-sin(alph)*cos(gamh)  cos(alph)*sin(beth)*cos(gamh)+sin(alph)*sin(gamh); ...
          -sin(beth)           cos(beth)*sin(gamh)                                cos(beth)*cos(gamh)];
RotMatRalp = [cos(alph)*cos(beth) cos(alph)*sin(beth)*sin(gamh)-sin(alph)*cos(gamh)  cos(alph)*sin(beth)*cos(gamh)+sin(alph)*sin(gamh); ...
             -sin(alph)*cos(beth) -sin(alph)*sin(beth)*sin(gamh)-cos(alph)*cos(gamh) -sin(alph)*sin(beth)*cos(gamh)+cos(alph)*sin(gamh); ...
             0                    0                                                  0];
RotMatRbet = [-sin(alph)*sin(beth)  sin(alph)*cos(beth)*sin(gamh) sin(alph)*cos(beth)*cos(gamh); ...
              -cos(alph)*sin(beth)  cos(alph)*cos(beth)*sin(gamh) cos(alph)*cos(beth)*cos(gamh); ...
              -cos(beth)            -sin(beth)*sin(gamh)          -sin(beth)*cos(gamh)];
RotMatRgam = [0          sin(alph)*sin(beth)*cos(gamh)-cos(alph)*sin(gamh) -sin(alph)*sin(beth)*sin(gamh)-cos(alph)*cos(gamh); ...
              0          cos(alph)*sin(beth)*cos(gamh)+sin(alph)*sin(gamh) -cos(alph)*sin(beth)*sin(gamh)+sin(alph)*cos(gamh); ...
              0          cos(beth)*cos(gamh)                               -cos(beth)*sin(gamh)];


ZyroUpdate = [1 sin(gamh)*tan(beth) cos(gamh)*tan(beth); ...
              0 cos(gamh) -sin(gamh); ...
              0 sin(gamh)/cos(beth) cos(gamh)/cos(beth)];

ZyroUpdateRalp = [0 0 0; ...
                  0 0 0; ...
                  0 0 0];

ZyroUpdateRbet = [0 sin(gamh)/cos(beth)^2           cos(gamh)/cos(beth)^2; ...
                  0 0                               0; ...
                  0 sin(gamh)*sin(beth)/cos(beth)^2 cos(gamh)*sin(beth)/cos(beth)^2];

ZyroUpdateRgam = [0 cos(gamh)*tan(beth) -sin(gamh)*tan(beth); ...
                  0 -sin(gamh)          -cos(gamh); ...
                  0 cos(gamh)/cos(beth) -sin(gamh)/cos(beth)];


A(1:3,4:6) = eye(3)*dT;

A(1,7) = 0.5*dT^2*RotMat(1,:)*[-1; 0; 0];
A(1,8) = 0.5*dT^2*RotMat(1,:)*[0; -1; 0];
A(1,9) = 0.5*dT^2*RotMat(1,:)*[0; 0; -1];

A(1,10) = 0.5*dT^2*RotMatRalp(1,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];
A(1,11) = 0.5*dT^2*RotMatRbet(1,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];
A(1,12) = 0.5*dT^2*RotMatRgam(1,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];

A(2,7) = 0.5*dT^2*RotMat(2,:)*[-1; 0; 0];
A(2,8) = 0.5*dT^2*RotMat(2,:)*[0; -1; 0];
A(2,9) = 0.5*dT^2*RotMat(2,:)*[0; 0; -1];

A(2,10) = 0.5*dT^2*RotMatRalp(2,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];
A(2,11) = 0.5*dT^2*RotMatRbet(2,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];
A(2,12) = 0.5*dT^2*RotMatRgam(2,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];

A(3,7) = 0.5*dT^2*RotMat(3,:)*[-1; 0; 0];
A(3,8) = 0.5*dT^2*RotMat(3,:)*[0; -1; 0];
A(3,9) = 0.5*dT^2*RotMat(3,:)*[0; 0; -1];

A(3,10) = 0.5*dT^2*RotMatRalp(3,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];
A(3,11) = 0.5*dT^2*RotMatRbet(3,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];
A(3,12) = 0.5*dT^2*RotMatRgam(3,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];

A(4,7) = dT*RotMat(1,:)*[-1; 0; 0];
A(4,8) = dT*RotMat(1,:)*[0; -1; 0];
A(4,9) = dT*RotMat(1,:)*[0; 0; -1];

A(4,10) = dT*RotMatRalp(1,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];
A(4,11) = dT*RotMatRbet(1,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];
A(4,12) = dT*RotMatRgam(1,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];

A(5,7) = dT*RotMat(2,:)*[-1; 0; 0];
A(5,8) = dT*RotMat(2,:)*[0; -1; 0];
A(5,9) = dT*RotMat(2,:)*[0; 0; -1];

A(5,10) = dT*RotMatRalp(2,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];
A(5,11) = dT*RotMatRbet(2,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];
A(5,12) = dT*RotMatRgam(2,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];

A(6,7) = dT*RotMat(3,:)*[-1; 0; 0];
A(6,8) = dT*RotMat(3,:)*[0; -1; 0];
A(6,9) = dT*RotMat(3,:)*[0; 0; -1];

A(6,10) = dT*RotMatRalp(3,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];
A(6,11) = dT*RotMatRbet(3,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];
A(6,12) = dT*RotMatRgam(3,:)*[acc(1)-abxh; acc(2)-abyh; acc(3)-abzh];

A(10,12) = 1+dT*ZyroUpdateRgam(3,:)*[p(1)-pbh;p(2)-qbh;p(3)-rbh];
A(10,11) = dT*ZyroUpdateRbet(3,:)*[p(1)-pbh;p(2)-qbh;p(3)-rbh];
A(10,10) = dT*ZyroUpdateRalp(3,:)*[p(1)-pbh;p(2)-qbh;p(3)-rbh];

A(10,13) = dT*ZyroUpdate(3,:)*[-1; 0; 0];
A(10,14) = dT*ZyroUpdate(3,:)*[0; -1; 0];
A(10,15) = dT*ZyroUpdate(3,:)*[0; 0; -1];

A(11,12) = dT*ZyroUpdateRgam(2,:)*[p(1)-pbh;p(2)-qbh;p(3)-rbh];
A(11,11) = 1+dT*ZyroUpdateRbet(2,:)*[p(1)-pbh;p(2)-qbh;p(3)-rbh];
A(11,10) = dT*ZyroUpdateRalp(2,:)*[p(1)-pbh;p(2)-qbh;p(3)-rbh];

A(11,13) = dT*ZyroUpdate(2,:)*[-1; 0; 0];
A(11,14) = dT*ZyroUpdate(2,:)*[0; -1; 0];
A(11,15) = dT*ZyroUpdate(2,:)*[0; 0; -1];

A(12,12) = dT*ZyroUpdateRgam(1,:)*[p(1)-pbh;p(2)-qbh;p(3)-rbh];
A(12,11) = dT*ZyroUpdateRbet(1,:)*[p(1)-pbh;p(2)-qbh;p(3)-rbh];
A(12,10) = 1+dT*ZyroUpdateRalp(1,:)*[p(1)-pbh;p(2)-qbh;p(3)-rbh];

A(12,13) = dT*ZyroUpdate(1,:)*[-1; 0; 0];
A(12,14) = dT*ZyroUpdate(1,:)*[0; -1; 0];
A(12,15) = dT*ZyroUpdate(1,:)*[0; 0; -1];











