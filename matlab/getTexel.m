function [d0s,d1s,d2s,p0s,p1s,p2s]=getTexel(tactile)

dsize=size(tactile,2);

f1=tactile(1:dsize/3);
f2=tactile(dsize/3+1:2*dsize/3);
f3=tactile(2*dsize/3+1:3*dsize/3);

p2i=f1(1:84);
d2i=f1(85:end);
p1i=f2(1:84);
d1i=f2(85:end);
p0i=f3(1:84);
d0i=f3(85:end);

 [d0s]=convertDistal(d0i);
 [d1s]=convertDistal(d1i);
 [d2s]=convertDistal(d2i);
 
 [p0s]=convertProximal(p0i);
 [p1s]=convertProximal(p1i);
 [p2s]=convertProximal(p2i);
 
end