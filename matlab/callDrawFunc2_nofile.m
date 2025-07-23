function callDrawFunc2_nofile(nohand,skeleton,tactile, joints, qtrans)
 
cmap=mycolor(0);
renk1='c';
renk2='b';
gcr=[1 1 1];
empty=[1 1 1];
lwb=3;%1.1;
lws=3;%1.1;
 
ang=60;
 
if skeleton
[d0,d1,d2,ds0,ds1,ds2,ds0end,ds1end,ds2end]=drawSDH4_S_quatAndSkeleton(cmap,skeleton,tactile,0,0, joints,qtrans, ang, nohand);
else
[d0,d1,d2,ds0,ds1,ds2,ds0end,ds1end,ds2end]=drawSDH4_S_quat(cmap,tactile,0,0, joints,qtrans, ang, 1, 1,1);
end