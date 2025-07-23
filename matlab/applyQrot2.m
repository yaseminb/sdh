function [a]=applyQrot2(a, qtrans, ang)

a=myrot(0,0,deg2rad(ang))*[a(1:3);1];
a=quaternion2matrix(qtrans(4:7))*[a(1:3);1];
a=mytrans(qtrans(1),qtrans(2),qtrans(3))*[a(1:3);1];

ekle=quaternion2matrix(qtrans(4:7))*[0 0 121-25 1]';
ekle=ekle(1:3);

a=mytrans(ekle(1),ekle(2),ekle(3))*[a(1:3);1];
a=a(1:3);

