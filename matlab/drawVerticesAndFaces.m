function drawVerticesAndFaces(vl0, f3l, mode, color)

FV.vertices=vl0'; 
FV.faces=f3l';  

if mode
   patch(FV,'EdgeColor','none','FaceColor', [0.5 0.5 0.5],'FaceAlpha', 0.1, 'LineWidth', 0.1); camlight
else
   patch(FV,'EdgeColor','none','FaceColor', color,'FaceAlpha', 0.1, 'LineWidth', 0.1); camlight
end

end
