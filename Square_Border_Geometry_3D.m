%Particle Soup Square Border Geometry 3D
%Leif Wesche

function [xyz0] = Square_Border_Geometry_3D(b, res)

%Bottom Bounds z=-b
dum=[linspace(-b, b, res)];
x0bottom=[]; y0bottom=[];
for i=1:res
x0bottom=[x0bottom, dum];
y0bottom=[y0bottom, dum(i)*ones(1,res)];
end
z0bottom=-b*ones(1,res^2);
xyz0bottom=[x0bottom; y0bottom; z0bottom];

%Top Bounds z=b
dum=[linspace(-b, b, res)];
x0top=[]; y0top=[];
for i=1:res
x0top=[x0top, dum];
y0top=[y0top, dum(i)*ones(1,res)];
end
z0top=b*ones(1,res^2);
xyz0top=[x0top; y0top; z0top];

%Left Side Bounds x=-b
x0left=[-b*ones(1,res^2)];
y0left=[]; z0left=[];
for i=1:res
y0left=[y0left, dum];
z0left=[z0left, dum(i)*ones(1,res)];
end
xyz0left=[x0left; y0left; z0left];

%Right Side Bounds x=b
x0right=[b*ones(1,res^2)];
y0right=[]; z0right=[];
for i=1:res
y0right=[y0right, dum];
z0right=[z0right, dum(i)*ones(1,res)];
end
xyz0right=[x0right; y0right; z0right];

%Back Side Bounds y=-b
y0back=[-b*ones(1,res^2)];
x0back=[]; z0back=[];
for i=1:res
x0back=[x0back, dum];
z0back=[z0back, dum(i)*ones(1,res)];
end
xyz0back=[x0back; y0back; z0back];

%Front Side Bounds y=b
y0front=[b*ones(1,res^2)];
x0front=[]; z0front=[];
for i=1:res
x0front=[x0front, dum];
z0front=[z0front, dum(i)*ones(1,res)];
end
xyz0front=[x0front; y0front; z0front];



%Combine Bounds
xyz0=[xyz0left, xyz0right, xyz0back, xyz0front, xyz0bottom, xyz0top];
x0=xyz0(1, :); y0=xyz0(2, :); z0=xyz0(3, :);

end