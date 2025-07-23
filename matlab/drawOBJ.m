 
function [v, f3]=drawOBJ(objname, draw, notrans)


[v,f3,f4]=loadawobj(objname);
  
  
  if draw
  FV.vertices=v'; 
  FV.faces=f3';  
  patch(FV,'EdgeColor','none','FaceColor', [0.5 0.5 0.5],'FaceAlpha', 0.1, 'LineWidth', 0.1); camlight
  end
  
  
  if notrans
  FV.vertices=v'; 
  FV.faces=f3';  
  patch(FV ,'FaceColor', [0.5 0.5 0.5],'FaceAlpha', 0.1, 'LineWidth', 0.1); camlight
  end
  
end