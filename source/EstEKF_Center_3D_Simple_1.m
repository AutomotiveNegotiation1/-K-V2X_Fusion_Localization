function [zhat, H] = EstEKF_Center_3D_Simple_1(Xhat)

Pxh = Xhat(1); Pyh = Xhat(2); Pzh = Xhat(3);
Vxh = Xhat(4); Vyh = Xhat(5); Vzh = Xhat(6); 
abxh = Xhat(7); abyh = Xhat(8); abzh = Xhat(9);
alph = Xhat(10); beth = Xhat(11); gamh = Xhat(12);
pbh = Xhat(13);qbh = Xhat(14);rbh = Xhat(15);

Pxe = Pxh;
Pye = Pyh;
Pze = Pzh;

zhat = zeros(1,6);
zhat(1) = alph;
zhat(2) = beth;
zhat(3) = gamh;

zhat(4) = Pxe;
zhat(5) = Pye;
zhat(6) = Pze;

H = zeros(6,15);
H(1,10) = 1;
H(2,11) = 1;
H(3,12) = 1;

H(4,1) = 1;
H(5,2) = 1;
H(6,3) = 1;


% H(2,7) = -xb(1)*sin(alph)+xb(2)*cos(alph);
% H(3,7) = -xb(1)*cos(alph)-xb(2)*sin(alph);


