function drawSkeletonVF(vl0, f3l, color,lw)

if nargin<4
    lw=1;
end

FV.vertices=vl0'; 
FV.faces=f3l;  


patch(FV,'EdgeColor','k','FaceColor', color,'LineWidth',lw,'DiffuseStrength',1,...
    'AmbientStrength',1,'BackFaceLighting','lit','FaceLighting','none');

end
