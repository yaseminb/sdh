function [ends,normals]=ProxSensorPoints_Sfunc_Normals(len,dr)

prox=ProxSensorPoints_Sfunc(0) ;

normalPoints=prox;
normalPoints(:,3)=normalPoints(:,3)-len;

normals=normalPoints-prox;

starts=prox;
ends=prox+normals;

if dr
plot3(prox(:,1), prox(:,2), prox(:,3),'r.', 'LineWidth', 12);
hold on;plot3(ends(:,1), ends(:,2), ends(:,3),'r.', 'LineWidth', 12);
end


