function [ends]=SensorPoints_Sfunc_Normals(len,dr)

[distal, distalas]=SensorPoints_Sfunc(0) ;
%distal is transformed sensor points so that they fall onto the distal
%sensor, distalas is before the transformation

center=[25.7816914564 -52.1226743057 	]; %y z

center=[8.5000 center(1) center(2);...
5.1000  center(1) center(2);...
1.7000  center(1) center(2);...
-1.7000 center(1) center(2);...
-5.1000 center(1) center(2);...
-8.5000 center(1) center(2);];


order=[1 7 13 19 25 31 37 43 ;...
            2 8 14 20 26 32 38 44 ;...
            3 9 15 21 27 33 39 45 ;...
            4 10 16 22 28 34 40 46 ;...
            5 11 17 23 29 35 41 47 ;...
            6 12 18 24 30 36 42 48 ];
        
        order2=[ 49 53 57 61 65;...
                       50 54 58 62 66;...
                       51 55 59 63 67;...
                       52 56 60 64 68];
order=order';
                   order2=order2';
                   
for j=1:6                   
for i=1:8
normalv(order(i,j),:)=-center(j,:)+distalas(order(i,j),:);
end
end


for i=1:5
normalv(order2(i,1),:)=-center(2,:)+distalas(order2(i,1),:);
normalv(order2(i,2),:)=-center(3,:)+distalas(order2(i,2),:);
normalv(order2(i,3),:)=-center(4,:)+distalas(order2(i,3),:);
normalv(order2(i,4),:)=-center(5,:)+distalas(order2(i,4),:);
end


for i=1:size(normalv,1)
    if norm(normalv(i,:))>0        
    normalv(i,:)=normalv(i,:)/norm(normalv(i,:));
    end
        
end

normalv=normalv*len;


starts=distalas;
ends=distalas+normalv;


if dr==1
plot3(distalas(:,1), distalas(:,2), distalas(:,3),'r.', 'LineWidth', 12);
hold on;arrow(starts,ends,'Length',0);
title ('before transform')
axis equal
end
 
%transform the end points to the distal phalanx
for i=1:size(ends,1)
  tempo=myrot(0,deg2rad(180),0)*[ends(i,1:3)';1];
  ends(i,1:3)=tempo(1:3);
  end
  
  for i=1:size(ends,1)
  tempo=myrot(deg2rad(11),0,0)*[ends(i,1:3)';1];
  ends(i,1:3)=tempo(1:3);
  end
  
  ends(:,2)=ends(:,2)+17.5+86.5+17.5;  
  ends(:,3)=ends(:,3)-9;

  
if dr==1
  starts=distal;
figure;
plot3(distal(:,1), distal(:,2), distal(:,3),'r.', 'LineWidth', 12);
hold on;arrow(starts,ends,'Length',0);
title ('transformed to be on the sensor')

axis equal
  
end
  
