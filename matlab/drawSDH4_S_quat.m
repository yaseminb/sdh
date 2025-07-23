function [d0,d1,d2,ds0,ds1,ds2,ds0end,ds1end,ds2end]=drawSDH4_S_quat(cmap,tactile, drawn,...
    drawsp, joint,qtrans, ang, nohand, drawDistPatch, drawProxPatch,darkness)
%yaseminb@kth.se, Yasemin Bekiroglu

disp('right.................................');
if nargin <11
    darkness=0.1;
end
hold on;
draw=0;
notrans=0;

joint=deg2rad(joint);

theta_rot = joint(1);
theta_p2  = joint(2);
theta_d2  = joint(3);
theta_p1  = joint(4);
theta_d1  = joint(5);
theta_p0  = joint(6);
theta_d0  = joint(7);


if nohand==1
    load('SDHmodel.mat');
vp(2,:)=vp(2,:)-17.5;
vd(2,:)=vd(2,:)-104;


%links:---------------------------------------------------------------------------------
for i=1:size(vl,2)
    a=(LinkToWrist0_(theta_rot)*[vl(:,i);1]);
    a=applyQrot2(a,qtrans,ang)  ;
    vl0(:,i)=a(1:3);
end

for i=1:size(vl,2)
    a=(LinkToWrist1_()*[vl(:,i);1]);
    a=applyQrot2(a,qtrans,ang)  ;
    vl1(:,i)=a(1:3);
end

for i=1:size(vl,2)
    a=(LinkToWrist2_(theta_rot)*[vl(:,i);1]);
    a=applyQrot2(a,qtrans,ang)  ;
    vl2(:,i)=a(1:3);
end

%--prox:
for i=1:size(vp,2)
    a=(LinkToWrist0_(theta_rot)*ProxToLink(theta_p0)*[vp(:,i);1]);
    a=applyQrot2(a,qtrans,ang)  ; 
    vp0(:,i)=a(1:3);
end

for i=1:size(vp,2)
    a=(LinkToWrist1_*ProxToLink(theta_p1)*[vp(:,i);1]);
    a=applyQrot2(a,qtrans,ang)  ; 
    vp1(:,i)=a(1:3);
end

for i=1:size(vp,2)
    a=(LinkToWrist2_(theta_rot)*ProxToLink(theta_p2)*[vp(:,i);1]);
    a=applyQrot2(a,qtrans,ang)  ;
    vp2(:,i)=a(1:3);
end

%distals:
for i=1:size(vd,2)
    a=((LinkToWrist0_(theta_rot)*ProxToLink(theta_p0)*DistToProx(theta_d0))*[vd(:,i);1]);
    a=applyQrot2(a,qtrans,ang)  ;  
    vd0(:,i)=a(1:3);
end

for i=1:size(vd,2)
    a=((LinkToWrist1_()*ProxToLink(theta_p1)*DistToProx(theta_d1))*[vd(:,i);1]);
   a=applyQrot2(a,qtrans,ang)  ;
    vd1(:,i)=a(1:3);
end

for i=1:size(vd,2)
    a=((LinkToWrist2_(theta_rot)*ProxToLink(theta_p2)*DistToProx(theta_d2))*[vd(:,i);1]);
    a=applyQrot2(a,qtrans,ang)  ;  
    vd2(:,i)=a(1:3);
end



%for wrist:
for i=1:size(vb,2)
    a=[vb(:,i);1];
    a=applyQrot2(a,qtrans,ang)  ;    
    vb(:,i)=a(1:3);
end

hold on;
drawVerticesAndFaces(vl0, f3l, 0, [darkness darkness darkness])
hold on;
drawVerticesAndFaces(vl1, f3l, 0, [darkness darkness darkness])
hold on;
drawVerticesAndFaces(vl2, f3l, 0, [darkness darkness darkness ])
hold on;
drawVerticesAndFaces(vb, f3b, 0, [0.5 0.5 0.5])

hold on;
drawVerticesAndFaces(vp1, f3p, 0, [darkness darkness darkness])
hold on;
drawVerticesAndFaces(vp0, f3p, 0, [darkness darkness darkness])
hold on;
drawVerticesAndFaces(vp2, f3p, 0, [darkness darkness darkness])

hold on;
drawVerticesAndFaces(vd1, f3d, 0, [darkness darkness darkness])
hold on;
drawVerticesAndFaces(vd0, f3d, 0, [darkness darkness darkness])
hold on;
drawVerticesAndFaces(vd2, f3d, 0, [darkness darkness darkness])

end


%sensor points:---------------------------------------------------------------

prox=ProxSensorPoints_Sfunc(0);
prox(:,2)=prox(:,2)-17.5;
[corn3P, facesP]=ProxSensorPoints_Sfunc_Corners(0);
corn3P(:,2)=corn3P(:,2)-17.5;
prox(:,1)=-prox(:,1); % just reverse the order of the first dim

clear a;
%--prox sensor:
for i=1:14*6
    a=(LinkToWrist0_(theta_rot)*ProxToLink(theta_p0)*[prox(i,1:3)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    pr0(:,i)=a(1:3);
end

for i=1:14*6
    a=(LinkToWrist1_*ProxToLink(theta_p1)*[prox(i,1:3)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    pr1(:,i)=a(1:3);
end

for i=1:14*6
    a=(LinkToWrist2_(theta_rot)*ProxToLink(theta_p2)*[prox(i,1:3)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    pr2(:,i)=a(1:3);
end

%distal sensor:
dist=SensorPoints_Sfunc(0);
[corn3, faces]=SensorPoints_Sfunc_Corners(0);
dist(:,2)=dist(:,2)-104;
corn3(:,2)=corn3(:,2)-104;

dist(:,1)=-dist(:,1); % just reverse the order of the first dim

for i=1:68
    a=((LinkToWrist0_(theta_rot)*ProxToLink(theta_p0)*DistToProx(theta_d0))*[dist(i,1:3)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    ds0(:,i)=a(1:3);
end

for i=1:68
    a=((LinkToWrist1_()*ProxToLink(theta_p1)*DistToProx(theta_d1))*[dist(i,1:3)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    ds1(:,i)=a(1:3);
end

for i=1:68
    a=((LinkToWrist2_(theta_rot)*ProxToLink(theta_p2)*DistToProx(theta_d2))*[dist(i,1:3)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    ds2(:,i)=a(1:3);
end

if drawsp==1
hold on;
plot3(ds0(1,:), ds0(2,:), ds0(3,:),'r.', 'LineWidth', 12);hold on;
plot3(ds1(1,:), ds1(2,:), ds1(3,:),'g.', 'LineWidth', 12);hold on;
plot3(ds2(1,:), ds2(2,:), ds2(3,:),'b.', 'LineWidth', 12);hold on;

plot3(pr0(1,:), pr0(2,:), pr0(3,:),'r.', 'LineWidth', 12);hold on;
plot3(pr1(1,:), pr1(2,:), pr1(3,:),'g.', 'LineWidth', 12);hold on;
plot3(pr2(1,:), pr2(2,:), pr2(3,:),'b.', 'LineWidth', 12);hold on;
end

%plot the tactile cells-distal:---------------------------------------------

for i=1:size(corn3,1)
    a=((LinkToWrist0_(theta_rot)*ProxToLink(theta_p0)*DistToProx(theta_d0))*[corn3(i,:)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    ds0patch(:,i)=a(1:3);
end

for i=1:size(corn3,1)
    a=((LinkToWrist1_()*ProxToLink(theta_p1)*DistToProx(theta_d1))*[corn3(i,:)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    ds1patch(:,i)=a(1:3);
end

for i=1:size(corn3,1)
    a=((LinkToWrist2_(theta_rot)*ProxToLink(theta_p2)*DistToProx(theta_d2))*[corn3(i,:)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    ds2patch(:,i)=a(1:3);
end


[d0,d1,d2,p0,p1,p2]=getTexel(tactile);
maxd=max(tactile);

if drawDistPatch==1
for i=1:68
col=getcolor(cmap,d0(i),maxd);

whos=faces(i,:);
hold on;   patch('Faces',[1 2 3 4],'Vertices',ds0patch(:,whos)' ,'FaceColor',col,'DiffuseStrength',1,'AmbientStrength',1,'BackFaceLighting','lit','FaceLighting','none');
col=getcolor(cmap,d1(i),maxd);
hold on;   patch('Faces',[1 2 3 4],'Vertices',ds1patch(:,whos)','FaceColor',col,'DiffuseStrength',1,'AmbientStrength',1,'BackFaceLighting','lit','FaceLighting','none');
col=getcolor(cmap,d2(i),maxd);
hold on;   patch('Faces',[1 2 3 4],'Vertices',ds2patch(:,whos)','FaceColor',col,'DiffuseStrength',1,'AmbientStrength',1,'BackFaceLighting','lit','FaceLighting','none');

end
end
%plot tactile cells-proximal

%--prox sensor:
for i=1:size(corn3P,1)
    a=(LinkToWrist0_(theta_rot)*ProxToLink(theta_p0)*[corn3P(i,1:3)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    pr0patch(:,i)=a(1:3);
end

for i=1:size(corn3P,1)
    a=(LinkToWrist1_*ProxToLink(theta_p1)*[corn3P(i,1:3)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    pr1patch(:,i)=a(1:3);
end

for i=1:size(corn3P,1)
    a=(LinkToWrist2_(theta_rot)*ProxToLink(theta_p2)*[corn3P(i,1:3)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    pr2patch(:,i)=a(1:3);
end


if drawProxPatch==1
for i=1:84
whos=facesP(i,:);
col=getcolor(cmap,p0(i),maxd);
hold on;   patch('Faces',[1 2 3 4],'Vertices',pr0patch(:,whos)','FaceColor',col,'DiffuseStrength',1, 'AmbientStrength',1,'BackFaceLighting','lit','FaceLighting','none');
col=getcolor(cmap,p1(i),maxd);
hold on;   patch('Faces',[1 2 3 4],'Vertices',pr1patch(:,whos)','FaceColor',col,'DiffuseStrength',1, 'AmbientStrength',1,'BackFaceLighting','lit','FaceLighting','none');
col=getcolor(cmap,p2(i),maxd);
hold on;   patch('Faces',[1 2 3 4],'Vertices',pr2patch(:,whos)','FaceColor',col,'DiffuseStrength',1, 'AmbientStrength',1,'BackFaceLighting','lit','FaceLighting','none');
%input('bak');
end
end
%draw normals:-----------------------------------------------------------

len=5;
[ends]=ProxSensorPoints_Sfunc_Normals(len, 0);
ends(:,2)=ends(:,2)-17.5;

%--prox sensor end points:
for i=1:14*6
    a=(LinkToWrist0_(theta_rot)*ProxToLink(theta_p0)*[ends(i,1:3)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    pr0end(:,i)=a(1:3);
end

for i=1:14*6
    a=(LinkToWrist1_*ProxToLink(theta_p1)*[ends(i,1:3)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    pr1end(:,i)=a(1:3);
end

for i=1:14*6
    a=(LinkToWrist2_(theta_rot)*ProxToLink(theta_p2)*[ends(i,1:3)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    pr2end(:,i)=a(1:3);
end

if drawn==1
hold on;arrow(pr2',pr2end','Length',0);
hold on;arrow(pr1',pr1end','Length',0);
hold on;arrow(pr0',pr0end','Length',0);
end


[endsD]=SensorPoints_Sfunc_Normals(len,0);
endsD(:,2)=endsD(:,2)-104;

for i=1:68
    a=((LinkToWrist0_(theta_rot)*ProxToLink(theta_p0)*DistToProx(theta_d0))*[endsD(i,1:3)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    ds0end(:,i)=a(1:3);
end

for i=1:68
    a=((LinkToWrist1_()*ProxToLink(theta_p1)*DistToProx(theta_d1))*[endsD(i,1:3)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    ds1end(:,i)=a(1:3);
end

for i=1:68
    a=((LinkToWrist2_(theta_rot)*ProxToLink(theta_p2)*DistToProx(theta_d2))*[endsD(i,1:3)';1]);
    a=applyQrot2(a,qtrans,ang)  ;
    ds2end(:,i)=a(1:3);
end

if drawn==1
hold on;arrow(ds2',ds2end','Length',0);
hold on;arrow(ds1',ds1end','Length',0);
hold on;arrow(ds0',ds0end','Length',0);
end


end