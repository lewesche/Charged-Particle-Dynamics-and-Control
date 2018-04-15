%Particle Soup (3D)
%Leif Wesche

clear all
close all
clc
addpath('C:\Users\ASUS\Desktop\Github\Particle Soup')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.0333;        %Set Time Step (seconds)
run_time=50;    %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[1, 1, 1, 1.5, 0.75];        %Particle Mass
qi=[1, 1, 1.25, 1.5, 1];       %Charge (Positive Only)
q_variance=0;         %Max Random Charge Variance
xi=[-0.5, 0, 3, -2, 1];          %Initial X Position
yi=[0, 0.5, 0, -1, 3];            %Initial Y Position
zi=[-0.5, 0.5, -1, 3, 1];            %Initial Z Position
vxi=[1, -1, 0, 0, 1];          %Initial X Velocity
vyi=[1, -1, 1, 0, 0];          %Initial Y Velocity
vzi=[1, -1, 0, -1, -1];          %Initial Z Velocity

q_box=60;       %Total Distributed Charge of Borders
res=20;         %Total Number of Discrete Border Points
b=4;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Math  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

q_box=q_box/4; res=res/4;

[xyz0] = Square_Border_Geometry_3D(b, res);

q0=q_box/res*ones(1,length(xyz0));

qmax=[qi, q0]; qmax=max(qmax); qmax=qmax+q_variance; 
qmin=[qi, q0]; qmin=min(qmin); qmin=qmin-q_variance; 
mmax=max(m);

n=length(m);

xyz=[xi; yi; zi];
vxyz=[vxi; vyi; vzi];


F=[];
[sx, sy, sz]=sphere(15);
fig=figure('Position', [0, 0, 1080, 1080], 'color', [0, 0, 0]);
for i=t
    tc=round(i);
    %Apply Charge Variance
    if mod(i,0.25) == 0
        for j=[1:n]
        var=(rand-0.5)*q_variance;
        q(j)=qi(j)+var;
        end
    end
    
    [xyz, vxyz] = Particle_Dynamics_3D(m, q, xyz, vxyz, xyz0, q0, i, dt);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Animation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    hold on; axis([-b-.2, b+.2, -b-.2, b+.2, -b-.2, b+.2])
    for j=[1:length(xyz0)]  
        s=surf(xyz0(1,j)+sx/7, xyz0(2,j)+sy/7, xyz0(3,j)+sz/7);
        s.FaceColor=([.01 .95 1]*q0(j)/qmax).^(1/4);  s.EdgeColor = 'none';
    end
    for j=[1:n]  
        s=surf(xyz(1,j)+sx*m(j)/mmax/2.5, xyz(2,j)+sy*m(j)/mmax/2.5, xyz(3,j)+sz*m(j)/mmax/2.5); 
        s.FaceColor=([.01 .95 1]*q(j)/qmax).^(1/4);  s.EdgeColor = 'none';
    end
    grid on;
    camlight; lighting phong; view([30+10*i, 30+10*i])
    title(['Particle Soup (3D)      ', 'Time Elapsed:' num2str(tc), 'Sec'], 'color', 'w')
    set(gca, 'Color', 'k', 'GridColor', 'w', 'XAxisLocation', 'origin'); hold off
    pause(dt); clf
    
end


