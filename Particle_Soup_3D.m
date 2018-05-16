%Particle Soup (3D)
%Leif Wesche

clear all
close all
clc
addpath('C:\Users\ASUS\Desktop\Github\Particle Soup')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=1/30;        %Set Time Step (seconds)
run_time=40;    %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[1];        %Particle Mass
qi=[4];       %Charge (Positive Only)
q_variance=0;         %Max Random Charge Variance
xi=[0];          %Initial X Position
yi=[-1];            %Initial Y Position
zi=[2];            %Initial Z Position
vxi=[1];          %Initial X Velocity
vyi=[0];          %Initial Y Velocity
vzi=[0];          %Initial Z Velocity

q_box=500;       %Total Distributed Charge of Borders
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
    
    %3D Plot
    hold on; axis([-b-.2, b+.2, -b-.2, b+.2, -b-.2, b+.2])
    for j=[1:length(xyz0)]  
        s=surf(xyz0(1,j)+sx/7, xyz0(2,j)+sy/7, xyz0(3,j)+sz/7);
        s.FaceColor=([.01 .88 1]*q0(j)/qmax).^(1/2);  s.EdgeColor = 'none';
    end
    for j=[1:n]  
        s=surf(xyz(1,j)+sx*m(j)/mmax/2.5, xyz(2,j)+sy*m(j)/mmax/2.5, xyz(3,j)+sz*m(j)/mmax/2.5); 
        s.FaceColor=([.01 .88 1]*q(j)/qmax).^(1/2);  s.EdgeColor = 'none';
    end
    grid off; axis off;
    camlight; lighting phong; 
    view([-90+90*cos(i/14+pi/4), 90*cos(i/14+pi/4)])
    title(['Particle Soup (3D)      ', 'Time Elapsed:' num2str(tc), 'Sec'], 'color', 'w')
    set(gca, 'Color', 'k', 'GridColor', 'w', 'XAxisLocation', 'origin'); hold off
    pause(dt/2);  clf


    %EZ plot
%     hold on; axis([-b-.2, b+.2, -b-.2, b+.2, -b-.2, b+.2])
%     for j=[1:length(xyz0)]  
%         plot3(xyz0(1,j), xyz0(2,j), xyz0(3,j), 'bo', 'linewidth', 5)
%     end
%     for j=[1:n]  
%         plot3(xyz(1,j), xyz(2,j), xyz(3,j), 'go', 'linewidth', 8)
%     end
%     grid on;
%     view([-90+90*cos(i/1), 90*cos(i/1)])
%     title(['Particle Soup (3D)      ', 'Time Elapsed:' num2str(tc), 'Sec'], 'color', 'w')
%     set(gca, 'Color', 'k', 'GridColor', 'w', 'XAxisLocation', 'origin'); hold off
%     pause(dt); clf
    
%F=[F, getframe(fig)]; clf
end

%v=VideoWriter('Particle_Soup_3D_3.avi','Uncompressed AVI');
%open(v)
%writeVideo(v,F)
%close(v)





