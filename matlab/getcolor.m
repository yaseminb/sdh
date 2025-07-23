function [col]=getcolor(cmap,tactile,maxd)

if maxd==0
    col=cmap(1,:);
else

dsize=size(cmap,1)-1;
index=floor(dsize*tactile/maxd+1);
col=cmap(index,:);

end
end