draw=1;
notrans=0;
col=[0.6 0 0];

[vd, f3d]=drawOBJ('distal.obj', draw, notrans);

axis equal;
d_c1=[12 86.5+17.5 0];
d_c2=[-12 86.5+17.5 0];
d_c3=[-12 86.5+17.5+17.5+48.902 0];
d_c4=[12 86.5+17.5+17.5+48.902 0];
hold on;

patch('Faces',[1 2 3 4],'Vertices',[d_c1;d_c2;d_c3;d_c4]...
     ,'FaceColor',col,'DiffuseStrength',1,'AmbientStrength',...
     1,'BackFaceLighting','lit','FaceLighting','none');

r_base_corner=[-38.105 0 25];
g_base_corner=[19.053 -33 25];
b_base_corner=[19.053 33 25];
link_c1=[22.4 0 0];
link_c2=[-22.4 0 0];
link_c3=[-22.4 16.5 0];
link_c4=[+22.4 16.5 0];
p_c1=[12 17.5 0];
p_c2=[-12 17.5 0];
p_c3=[-12 86.5+17.5 0];
p_c4=[12 86.5+17.5 0];
d_c1=[12 86.5+17.5 0];
d_c2=[-12 86.5+17.5 0];
d_c3=[-12 86.5+17.5+17.5+48.902 0];
d_c4=[12 86.5+17.5+17.5+48.902 0];
