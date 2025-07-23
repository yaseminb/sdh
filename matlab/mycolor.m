
function [col]=mycolor(showcolorbar)

disp('send 1 to see the colorbar');
if nargin<1
    showcolorbar=0;
end


init=1;
kac=256*4;
goal1=0.543; goal2=0; goal3=0;

c1=linspace(init,goal1,kac);
c2=linspace(init,goal2,kac);
c3=linspace(init,goal3,kac);


col=[c1' c2' c3'];

colormap(col)
if showcolorbar
colorbar
axis off;
end

end

